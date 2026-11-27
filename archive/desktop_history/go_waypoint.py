import argparse
import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.signals import SignalHandlerOptions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("target", help="workbench 또는 homepoint")
    args = parser.parse_args()

    path = Path(__file__).with_name("waypoints.yaml")
    points = yaml.safe_load(path.read_text(encoding="utf-8"))

    if not isinstance(points, dict) or args.target not in points:
        parser.error(f"등록되지 않은 목적지: {args.target}")

    point = points[args.target]
    if not isinstance(point, dict):
        parser.error("목적지 설정이 올바르지 않습니다.")

    for key in ("x", "y", "yaw"):
        value = point.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            parser.error(f"{args.target}.{key}에 실제 숫자를 입력하세요.")

    if point.get("frame_id") != "map":
        parser.error("frame_id는 map이어야 합니다.")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator()
    running = False

    try:
        print("Nav2 준비 대기 중...")
        nav.waitUntilNav2Active()

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = float(point["x"])
        goal.pose.position.y = float(point["y"])
        goal.pose.orientation.z = math.sin(point["yaw"] / 2)
        goal.pose.orientation.w = math.cos(point["yaw"] / 2)

        print(f"{args.target} 이동 요청: {point}")
        running = True
        if nav.goToPose(goal) is False:
            running = False
            print("이동 목표가 거절됐습니다.")
            return 1

        started = time.monotonic()
        while not nav.isTaskComplete():
            if time.monotonic() - started > 120:
                print("120초 초과: 이동 취소")
                nav.cancelTask()
                running = False
                return 1

        running = False
        result = nav.getResult()
        print(f"이동 결과: {result.name}")
        return 0 if result == TaskResult.SUCCEEDED else 1

    except KeyboardInterrupt:
        if running:
            nav.cancelTask()
        print("사용자 취소")
        return 130
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
