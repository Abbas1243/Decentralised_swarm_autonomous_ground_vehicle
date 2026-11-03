"""Usage, once the simulation is running:
    ros2 run swarm_robot join_leave_cli robot_4 join 2.0 2.0
    ros2 run swarm_robot join_leave_cli robot_4 leave
"""
import sys
import rclpy
from swarm_interfaces.srv import SetRobotActive


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return

    robot_id, action = sys.argv[1], sys.argv[2]
    x = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

    rclpy.init()
    node = rclpy.create_node('join_leave_cli')
    client = node.create_client(SetRobotActive, '/swarm/set_robot_active')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('swarm_supervisor service not available -- is the sim running?')
        return

    req = SetRobotActive.Request()
    req.robot_id = robot_id
    req.active = (action == 'join')
    req.spawn_position.x = x
    req.spawn_position.y = y

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    print(future.result().message)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
