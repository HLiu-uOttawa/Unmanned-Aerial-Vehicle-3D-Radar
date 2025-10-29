#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import os
import numpy as np
import pandas as pd
from datetime import datetime

from sr14ros2pkg.radar_sdk.RadarModule import ImstRadarModule
# from sr14ros2pkg.TDData import TDData
# from sr14ros2pkg.msg import TDFrame


def print_data_to_file(td_data, folder_location: str, timestamp: datetime = None):
    """
    Save one frame of TD data to a .txt file with timestamp in the name.

    Args:
        td_data (np.ndarray): The radar TD data (e.g., shape (1024, 4)).
        folder_location (str, optional): Directory to save file. Defaults to './data'.
        timestamp (datetime, optional): Timestamp for this frame. Defaults to current time.
    """
    if folder_location is None:
        folder_location = "./data"
    if timestamp is None:
        timestamp = datetime.now()
    os.makedirs(folder_location, exist_ok=True)

    timestamp_str = timestamp.strftime('%Y-%m-%d_%H-%M-%S.%f')[:-3]
    file_name = f"TD_{timestamp_str}.txt"
    file_path = os.path.join(folder_location, file_name)
    
    # Create a pandas DataFrame
    df = pd.DataFrame(td_data, columns=["I1", "Q1", "I2", "Q2"])

    # Create a header for the file
    header = (
        "Unit of the Time Domain Samples:\t[V]\n"
        "=======================================================\n"
        "<I1>\t<Q1>\t<I2>\t<Q2>\n\n"
    )

    # Write header and DataFrame to a .txt file
    with open(file_path, "w") as file:
        file.write(header)  # Write the header to the file
        # Use to_csv with tab separator and no index or header, float formatted to four decimals
        df.to_csv(file, sep='\t', index=False, header=False, float_format="%.4f")
        
class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')

        # Declare parameters
        self.declare_parameter('ip', '192.168.0.2')
        self.declare_parameter('port', 1024)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Publishers
        self.pub_td_data = self.create_publisher(Float32MultiArray, 'radar/td_data', 10)
        self.pub_fd_data = self.create_publisher(Float32MultiArray, 'radar/fd_data', 10)
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

            time = pd.Timestamp.now()
            n_samples = 1024
            td_data = []
            n = 0

            for ch in range(4):      # maximum possible channels = 4
                if self.radar.sysParams.active_RX_ch & (1<<ch):
                    ind1 = n*n_samples
                    ind2 = ind1 + n_samples
                    n += 1
                    td_data.append(self.radar.TD_Data.data[ind1:ind2])
                else:
                    td_data.append([0]*n_samples)

            # Application data with shape (4, 1024) -- I1 Time, Q1 Tim, I2 Time, Q2 Time
            td_data_amplitude =  np.array(td_data)
            # Covert from amplitude to the voltage values, per the manual's calculation
            td_data_voltage = (3 * td_data_amplitude) / ((2.**12) * 4 * self.radar.sysParams.t_ramp)


            print_data_to_file(td_data=td_data_voltage.T, folder_location='./data', timestamp=datetime.now())

            # td_msg.data = td_data_voltage.T
            # td_msg.time = pd.Timestamp.now()
      



            
            # self.get_logger().info(f"TD msg size: {len(td_msg.data)}")


            # self.pub_td_data.publish(td_msg)

            # --- FD Data ---
            self.radar.GetFdData("UP-Ramp")
            fd_msg = Float32MultiArray()
            fd_msg.data = [float(x) for x in self.radar.FD_Data.data]
            self.pub_fd_data.publish(fd_msg)

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

    # def get_td_data_voltage() -> TDData:

    #     # Application data with shape (4, 1024) -- I1 Time, Q1 Tim, I2 Time, Q2 Time
    #     td_data_amplitude =  np.array(td_data)
    #     # Covert from amplitude to the voltage values, per the manual's calculation
    #     td_data_voltage = (3 * td_data_amplitude) / ((2.**12) * 4 * radar_module.sysParams.t_ramp)
    #     # Transpose into shape (1024, 4), for easier handling of range bins and return
    #     return TDData(td_data_voltage.T, time)


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
