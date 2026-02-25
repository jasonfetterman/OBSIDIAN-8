import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.heartbeat_pub = self.create_publisher(
            String,
            '/obsidian/heartbeat',
            10
        )

        self.motion_pub = self.create_publisher(
            String,
            '/obsidian/motion_command',
            10
        )

        self.core_state_sub = self.create_subscription(
            String,
            '/obsidian/core_state',
            self.core_state_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.publish_commands)

        self.core_state = "UNKNOWN"
        self.phase = "INIT"   # INIT → STAND → WALK

        self.get_logger().info("Mission Node Started")

    def publish_commands(self):

        if self.core_state == "SAFE_MODE":
            motion_request = "STOP"

        else:
            if self.phase == "INIT":
                motion_request = "STAND"

                if self.core_state == "STAND":
                    self.phase = "WALK_PHASE"

            elif self.phase == "WALK_PHASE":
                motion_request = "WALK"

            else:
                motion_request = "STOP"

        # Heartbeat
        hb = String()
        hb.data = f"STATE:{motion_request}"
        self.heartbeat_pub.publish(hb)

        # Motion command
        motion = String()
        motion.data = motion_request
        self.motion_pub.publish(motion)

    def core_state_callback(self, msg):
        if msg.data != self.core_state:
            self.core_state = msg.data
            self.get_logger().info(f"Core State Update: {self.core_state}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
