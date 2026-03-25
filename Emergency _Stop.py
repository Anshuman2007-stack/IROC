import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mavros_msgs.srv import CommandBool


class EmergencyNode(Node):

    def __init__(self):
        super().__init__('emergency_node')

        self.create_subscription(
            Bool,
            '/emergency_stop',
            self.cb,
            10
        )

  
        self.pub = self.create_publisher(
            String,
            '/failsafe/emergency',
            10
        )

        self.arm_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")

    def cb(self, msg):
        if msg.data:
            self.get_logger().error("EMERGENCY TRIGGERED")

            
            msg = String()
            msg.data = "EMERGENCY"
            self.pub.publish(msg)

            
            self.disarm()

    def disarm(self):
        req = CommandBool.Request()
        req.value = False

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.response)

    def response(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().error("MOTORS DISARMED")
            else:
                self.get_logger().error("Disarm failed")
        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
