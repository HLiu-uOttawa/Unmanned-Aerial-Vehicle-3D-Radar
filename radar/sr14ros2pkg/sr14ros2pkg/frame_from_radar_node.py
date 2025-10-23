#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sr14ros2pkg.radar_sdk.RadarModule import ImstRadarModule

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')

        # Declare parameters
        self.declare_parameter('ip', '192.168.0.2')
        self.declare_parameter('port', 1024)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Publishers
        self.pub_td = self.create_publisher(Float32MultiArray, 'radar/td_data', 10)
        self.pub_fd = self.create_publisher(Float32MultiArray, 'radar/fd_data', 10)
        self.pub_tracker = self.create_publisher(String, 'radar/tracker', 10)

        # Create radar instance
        self.radar = ImstRadarModule()
        self.radar.etherParams.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.radar.etherParams.port = self.get_parameter('port').get_parameter_value().integer_value

        self.get_logger().info(f"Connecting radar at {self.radar.etherParams.ip}:{self.radar.etherParams.port} ...")
        
        self.radar.Connect("Ethernet")
        if not self.radar.connected:
            self.get_logger().error("Failed to connect to radar.")
            rclpy.shutdown()
            return

        # Timer to read and publish data
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(f"Radar connected. Publishing at {rate:.1f} Hz.")

    def timer_callback(self):
        try:
            # --- TD Data ---
            self.radar.GetTdData("UP-Ramp")
            td_msg = Float32MultiArray()
            td_msg.data = [float(x) for x in self.radar.TD_Data.data]
            self.pub_td.publish(td_msg)

            # --- FD Data ---
            self.radar.GetFdData("UP-Ramp")
            fd_msg = Float32MultiArray()
            fd_msg.data = [float(x) for x in self.radar.FD_Data.data]
            self.pub_fd.publish(fd_msg)

            # --- Tracker Results ---
            self.radar.AtMeasurement()
            text = f"{self.radar.AT_Targets.nTargets} targets detected."
            for i in range(self.radar.AT_Targets.nTargets):
                text += f" [ID={self.radar.AT_Targets.ID[i]}, Dist={self.radar.AT_Targets.dist[i]:.2f} m, " \
                        f"Speed={self.radar.AT_Targets.speed[i]:.2f} m/s]"
            tracker_msg = String()
            tracker_msg.data = text
            self.pub_tracker.publish(tracker_msg)

        except Exception as e:
            self.get_logger().error(f"Radar read failed: {e}")

    def destroy_node(self):
        """Ensure radar disconnects on shutdown."""
        if self.radar.connected:
            self.radar.Disconnect()
            self.get_logger().info("Radar disconnected.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down radar node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
