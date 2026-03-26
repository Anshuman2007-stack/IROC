import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CommFailsafe(Node):

    def __init__(self):
        super().__init__('comm_failsafe')
        self.TIMEOUT = 1.0        # seconds to detect link loss
        self.HOVER_TIME = 10.0    # seconds before landing

        self.last_heartbeat = self.get_clock().now()
        self.state = "NORMAL"
        self.hover_start = None

        self.create_subscription(
            String,
            '/heartbeat',
            self.heartbeat_cb,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/failsafe/comm',
            10
        )

        self.create_timer(0.2, self.monitor)

    def heartbeat_cb(self, msg):
        self.last_heartbeat = self.get_clock().now()

        if self.state != "NORMAL":
            self.get_logger().info("COMMUNICATION RESTORED")
            self.state = "NORMAL"
            self.hover_start = None
            self.publish_state("NORMAL")

    def monitor(self):
        now = self.get_clock().now()
        dt = (now - self.last_heartbeat).nanoseconds / 1e9

    
        if dt > self.TIMEOUT:

            if self.state == "NORMAL":
                self.get_logger().warn("COMM LOST SO HOVER")
                self.state = "HOVER"
                self.hover_start = now
                self.publish_state("HOVER")

            elif self.state == "HOVER":
                elapsed = (now - self.hover_start).nanoseconds / 1e9

                if elapsed > self.HOVER_TIME:
                    self.get_logger().error("COMM NOT RECOVERED → LAND")
                    self.state = "LAND"
                    self.publish_state("LAND")

  
    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommFailsafe()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
