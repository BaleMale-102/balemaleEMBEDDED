#!/usr/bin/env python3
"""
go_home.py - 홈(마커 0)으로 복귀 (RETURN_HOME 상태)

사용법:
  python3 go_home.py

이 스크립트는 RETURN_HOME 상태로 전환하여 마커 0으로 돌아갑니다.
주행 완료 후 홈 복귀 시나리오 테스트용.
"""

import subprocess

def main():
    cmd = "ros2 topic pub --once /mission/test_cmd std_msgs/String \"data: 'HOME'\""

    print("Sending RETURN_HOME command...")
    subprocess.run(cmd, shell=True)
    print("Command sent. Robot will return to marker 0.")
    print("\nMonitor with: ros2 topic echo /mission/state")

if __name__ == '__main__':
    main()
