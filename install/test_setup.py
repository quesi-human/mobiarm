"""Installer tests: no SSH, apt, hardware access or ROS node startup."""
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('project_setup',Path(__file__).with_name('setup.py'))
setup=importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


class InstallerTests(unittest.TestCase):
    def test_copy_preserves_calibration_state_and_original_project(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); source=root/'source'; target=root/'target'; backup=root/'backup'
            for base in (source,target):
                (base/'calibration').mkdir(parents=True)
            (source/'arm_cli.py').write_text('new')
            (target/'arm_cli.py').write_text('old')
            (source/'calibration/park.json').write_text('snapshot')
            (target/'calibration/park.json').write_text('current calibration')
            (source/'state').mkdir()
            (source/'state/arm_state.json').write_text('{"holding":true}')
            setup.copy_project(source,target,backup)
            self.assertEqual((target/'arm_cli.py').read_text(),'new')
            self.assertEqual((backup/'arm_cli.py').read_text(),'old')
            self.assertEqual((target/'calibration/park.json').read_text(),'current calibration')
            self.assertFalse((target/'state').exists())
            self.assertEqual((source/'calibration/park.json').read_text(),'snapshot')

    def test_payload_contains_code_but_no_old_runtime_state(self):
        with tempfile.TemporaryDirectory() as folder:
            destination=Path(folder)/'payload.tar.gz'
            setup.make_payload(destination)
            with tarfile.open(destination) as tf:
                names=tf.getnames()
                self.assertIn('install/setup.py',names)
                self.assertIn('robot/requirements.txt',names)
                self.assertIn('robot/so101-mobile-pick/arm_cli.py',names)
                self.assertFalse(any('/state/' in n or '/mission_control/' in n or '/.git/' in n for n in names))

    def test_extract_rejects_path_escape(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); archive=root/'bad.tar'
            with tarfile.open(archive,'w') as tf:
                member=tarfile.TarInfo('../escaped'); member.size=1
                tf.addfile(member,io.BytesIO(b'x'))
            with self.assertRaises(RuntimeError):
                setup.extract(archive,root/'output')
            self.assertFalse((root/'escaped').exists())

    def test_launcher_uses_deployment_without_overriding_explicit_arguments(self):
        with tempfile.TemporaryDirectory(prefix='project with spaces ') as folder:
            root=Path(folder)
            (root/'controller').mkdir(); (root/'installation').mkdir()
            launcher=root/'execute_robot.py'
            launcher.write_bytes((setup.ROOT/'execute_robot.py').read_bytes())
            (root/'controller/execute_robot.py').write_text('import sys,json; print(json.dumps(sys.argv))')
            record={'host':'robot@host','remote_project':'/home/robot/project with spaces/so101-mobile-pick',
                    'remote_arm_python':'/home/robot/project/arm-venv/bin/python',
                    'remote_env':'/home/robot/project/setup_env.sh','domain_id':36,'model':'waffle_pi'}
            (root/'installation/deployment.json').write_text(json.dumps(record))
            result=subprocess.check_output([sys.executable,str(launcher),'--host','explicit@host'],text=True)
            args=json.loads(result)
            self.assertEqual(args.count('--host'),1)
            self.assertEqual(args[args.index('--host')+1],'explicit@host')
            self.assertIn(record['remote_project'],args)
            self.assertIn(record['remote_arm_python'],args)

    def test_ros_shell_sources_paths_with_spaces_safely(self):
        with tempfile.TemporaryDirectory(prefix='ros setup ') as folder:
            root=Path(folder); ros=root/'ros.bash'; overlay=root/'overlay.bash'
            ros.write_text('export INSTALL_TEST_ROS=yes\n')
            overlay.write_text('export INSTALL_TEST_OVERLAY=yes\n')
            with patch.object(setup,'ROS',ros):
                value=subprocess.check_output(setup.ros_shell([sys.executable,'-c',
                    'import os; print(os.environ["INSTALL_TEST_ROS"],os.environ["INSTALL_TEST_OVERLAY"])'],overlay),text=True)
            self.assertEqual(value.strip(),'yes yes')


if __name__=='__main__':
    unittest.main()
