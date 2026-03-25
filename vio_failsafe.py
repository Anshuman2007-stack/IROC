import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mavros_msgs.srv import SetMode


class FailsafeManager(Node):

    def __init__(self):
        super().__init__('failsafe_manager')
        self.RTH_BAT = 30.0
        self.LAND_BAT = 15.0

        self.FAIL_COV = 0.1
        self.TIMEOUT = 0.3

        self.last_vio_time = self.get_clock().now()

        self.battery_state = "NORMAL"
        self.vio_state = "NORMAL"
        self.current_status = None

        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_cb,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.vio_cb,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/failsafe/status',
            10
        )


        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for mode service...")

        self.create_timer(0.2, self.monitor)

    def battery_cb(self, msg):
        battery = msg.percentage * 100.0

        if battery <= self.LAND_BAT:
            self.battery_state = "LAND"
        elif battery <= self.RTH_BAT:
            self.battery_state = "RTH"
        else:
            self.battery_state = "NORMAL"


    def vio_cb(self, msg):
        self.last_vio_time = self.get_clock().now()

        cov = msg.pose.covariance
        avg_cov = (cov[0] + cov[7] + cov[14]) / 3.0

        if avg_cov > self.FAIL_COV:
            self.vio_state = "HOVER"
        else:
            self.vio_state = "NORMAL"

  
    def monitor(self):
        
        now = self.get_clock().now()
        dt = (now - self.last_vio_time).nanoseconds / 1e9

        if dt > self.TIMEOUT:
            self.vio_state = "HOVER"

        
        new_status = self.decide_state()

        
        if new_status != self.current_status:
            self.current_status = new_status

            msg = String()
            msg.data = new_status
            self.status_pub.publish(msg)

            self.get_logger().warn(f"🚨 STATUS → {new_status}")

            self.execute_action(new_status)


    def decide_state(self):
        if self.battery_state == "LAND":
            return "LAND"
        elif self.battery_state == "RTH":
            return "RTH"
        elif self.vio_state == "HOVER":
            return "HOVER"
        else:
            return "NORMAL"

    def execute_action(self, state):
        if state == "LAND":
            self.set_mode("AUTO.LAND")

        elif state == "RTH":
            self.set_mode("AUTO.RTL")

        elif state == "HOVER":
            self.set_mode("AUTO.LOITER")

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_response)

    def mode_response(self, future):
        try:
            res = future.result()
            if res.mode_sent:
                self.get_logger().info("Mode changed successfully")
            else:
                self.get_logger().error("Mode change failed")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FailsafeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
