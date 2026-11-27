#!/usr/bin/env python3
"""Run the organized controller without changing the original Desktop project."""
from pathlib import Path
import json
import os
import runpy
import sys

if __name__ == '__main__':
    root = Path(__file__).resolve().parent
    python = root/'.venv/bin/python'
    if python.is_file() and Path(sys.prefix).resolve() != (root/'.venv').resolve():
        os.execv(str(python),[str(python),str(Path(__file__).resolve()),*sys.argv[1:]])
    deployment = root/'installation/deployment.json'
    if deployment.is_file():
        settings=json.loads(deployment.read_text())
        defaults={'--host':settings['host'],'--remote-project':settings['remote_project'],
                  '--remote-arm-python':settings['remote_arm_python'],'--remote-env':settings['remote_env'],
                  '--domain-id':settings['domain_id'],'--model':settings['model']}
        for flag,value in defaults.items():
            if not any(arg==flag or arg.startswith(flag+'=') for arg in sys.argv[1:]):
                sys.argv[1:1]=[flag,str(value)]
    env = root/'laptop/setup_env.sh'
    if env.is_file():
        os.environ['PJT_SOARM_LOCAL_ENV']=str(env)
    controller = root / 'controller'
    sys.path.insert(0, str(controller))
    runpy.run_path(str(controller / 'execute_robot.py'), run_name='__main__')
