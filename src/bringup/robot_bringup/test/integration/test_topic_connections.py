#!/usr/bin/env python3
"""
test_topic_connections.py - Topic Connection Verification

Verifies that all expected topics are being published and subscribed.
Run with: python3 test_topic_connections.py
"""

import subprocess
import sys
import time
from typing import Dict, List, Tuple

# Expected topics and their message types
EXPECTED_TOPICS = {
    # Perception
    '/perception/markers': 'robot_interfaces/msg/MarkerArray',
    '/perception/tracked_marker': 'robot_interfaces/msg/TrackedMarker',
    '/perception/lane_status': 'robot_interfaces/msg/LaneStatus',
    '/perception/debug/marker_image': 'sensor_msgs/msg/Image',
    '/perception/debug/lane_image': 'sensor_msgs/msg/Image',
    
    # Control
    '/control/cmd_vel': 'geometry_msgs/msg/Twist',
    '/control/enable_drive': 'std_msgs/msg/Bool',
    '/motor/command': 'robot_interfaces/msg/MotorCommand',
    
    # Mission
    '/mission/state': 'std_msgs/msg/String',
    '/mission/target_marker': 'std_msgs/msg/Int32',
    '/mission/turn_target_rad': 'std_msgs/msg/Int32',
    '/mission/marker_reached': 'std_msgs/msg/Int32',
    '/mission/turn_done': 'std_msgs/msg/Bool',
    
    # Server
    '/server/task_cmd': 'robot_interfaces/msg/MissionCommand',
    '/server/task_status': 'robot_interfaces/msg/MissionStatus',
    
    # Diagnostics
    '/driving/state': 'robot_interfaces/msg/DrivingState',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/odom/wheel': 'nav_msgs/msg/Odometry',
}

# Expected node names
EXPECTED_NODES = [
    '/marker_detector',
    '/marker_tracker',
    '/lane_detector',
    '/motion_controller',
    '/wheel_controller',
    '/mission_manager',
    '/server_bridge',
]


def run_command(cmd: List[str], timeout: int = 5) -> Tuple[int, str]:
    """Run a command and return exit code and output."""
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.returncode, result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return -1, "Command timed out"
    except Exception as e:
        return -1, str(e)


def get_active_topics() -> Dict[str, str]:
    """Get currently active topics and their types."""
    code, output = run_command(['ros2', 'topic', 'list', '-t'])
    if code != 0:
        return {}
    
    topics = {}
    for line in output.strip().split('\n'):
        if line and ' ' in line:
            parts = line.split()
            if len(parts) >= 2:
                topic = parts[0]
                msg_type = parts[1].strip('[]')
                topics[topic] = msg_type
    return topics


def get_active_nodes() -> List[str]:
    """Get currently active nodes."""
    code, output = run_command(['ros2', 'node', 'list'])
    if code != 0:
        return []
    return [n.strip() for n in output.strip().split('\n') if n.strip()]


def check_topic_publishing(topic: str, timeout: float = 2.0) -> bool:
    """Check if a topic is actively publishing."""
    code, output = run_command(
        ['ros2', 'topic', 'hz', topic, '--window', '3'],
        timeout=int(timeout + 1)
    )
    return 'average rate' in output.lower()


def main():
    print("=" * 60)
    print("ROS2 Topic Connection Verification")
    print("=" * 60)
    print()
    
    # Check nodes
    print("Checking active nodes...")
    active_nodes = get_active_nodes()
    
    print(f"\nExpected nodes: {len(EXPECTED_NODES)}")
    print(f"Active nodes: {len(active_nodes)}")
    print()
    
    nodes_ok = 0
    nodes_missing = []
    
    for node in EXPECTED_NODES:
        if node in active_nodes:
            print(f"  [OK] {node}")
            nodes_ok += 1
        else:
            print(f"  [MISSING] {node}")
            nodes_missing.append(node)
    
    print()
    
    # Check topics
    print("Checking active topics...")
    active_topics = get_active_topics()
    
    print(f"\nExpected topics: {len(EXPECTED_TOPICS)}")
    print(f"Active topics: {len(active_topics)}")
    print()
    
    topics_ok = 0
    topics_missing = []
    topics_wrong_type = []
    
    for topic, expected_type in EXPECTED_TOPICS.items():
        if topic in active_topics:
            actual_type = active_topics[topic]
            if expected_type in actual_type or actual_type in expected_type:
                print(f"  [OK] {topic}")
                topics_ok += 1
            else:
                print(f"  [WRONG TYPE] {topic}")
                print(f"      Expected: {expected_type}")
                print(f"      Actual: {actual_type}")
                topics_wrong_type.append(topic)
        else:
            print(f"  [MISSING] {topic}")
            topics_missing.append(topic)
    
    print()
    
    # Summary
    print("=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"Nodes:  {nodes_ok}/{len(EXPECTED_NODES)} OK")
    print(f"Topics: {topics_ok}/{len(EXPECTED_TOPICS)} OK")
    
    if nodes_missing:
        print(f"\nMissing nodes ({len(nodes_missing)}):")
        for n in nodes_missing:
            print(f"  - {n}")
    
    if topics_missing:
        print(f"\nMissing topics ({len(topics_missing)}):")
        for t in topics_missing:
            print(f"  - {t}")
    
    if topics_wrong_type:
        print(f"\nWrong type topics ({len(topics_wrong_type)}):")
        for t in topics_wrong_type:
            print(f"  - {t}")
    
    print()
    
    # Return exit code
    if nodes_missing or topics_missing or topics_wrong_type:
        print("RESULT: FAIL")
        return 1
    else:
        print("RESULT: PASS")
        return 0


if __name__ == '__main__':
    sys.exit(main())
