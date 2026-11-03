"""Runs as the plugin for a single dedicated Robot node in the world with
`supervisor TRUE` and `controller "<extern>"`. It never moves itself -- it
just gives ROS2 the ability to teleport OTHER robots in and out of the arena,
which is how robots "join" and "leave" the swarm at runtime without needing
to hot-load nodes into a running Webots simulation.

A robot that's "parked" far outside every other robot's neighbor_radius is
invisible to the swarm algorithm automatically (NeighborManager's distance
filter takes care of it) -- so parking/activating is all this needs to do.
"""

import rclpy
from swarm_interfaces.srv import SetRobotActive

PARK_POSITION = (100.0, 100.0, 0.05)  # far outside any reasonable neighbor_radius


class SwarmSupervisorDriver:
    def init(self, webots_node, properties):
        self.supervisor = webots_node.robot  # this Robot node has supervisor=TRUE

        # NOTE: do NOT call rclpy.init() here -- the webots_ros2_driver
        # 'driver' process already initializes rclpy before loading this
        # plugin. Calling init() again raises and crashes the process.
        self.node = rclpy.create_node('swarm_supervisor')
        self.node.create_service(SetRobotActive, '/swarm/set_robot_active', self._on_request)
        self.node.get_logger().info('swarm_supervisor ready: /swarm/set_robot_active')

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def _on_request(self, request, response):
        target = self.supervisor.getFromDef(request.robot_id)
        if target is None:
            response.success = False
            response.message = f'no robot DEF named {request.robot_id} in the world'
            return response

        translation_field = target.getField('translation')
        if request.active:
            p = request.spawn_position
            translation_field.setSFVec3f([p.x, p.y, 0.05])
            response.message = f'{request.robot_id} activated (joined swarm)'
        else:
            translation_field.setSFVec3f(list(PARK_POSITION))
            response.message = f'{request.robot_id} parked (left swarm)'

        target.resetPhysics()
        response.success = True
        return response
