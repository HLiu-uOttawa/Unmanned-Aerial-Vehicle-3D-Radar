#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Sr14TdFrame

import os
from datetime import datetime
import numpy as np
import pandas as pd

class RadarFrameSaver(Node):
    def __init__(self):
        super().__init__('radar_frame_save_node')

        self.subscription = self.create_subscription(
                    Sr14TdFrame,              # Type of the message
                    '/sr14/td_data',          # Topic name
                    self.listener_callback,   # Callback function
                    10                        # QoS
                )
        self.get_logger().info('RadarFrameSaver started, listening on /sr14/td_data')

    def listener_callback(self, msg: Sr14TdFrame):
        stamp = msg.header.stamp

        len_ch1 = len(msg.ch1)
        len_ch2 = len(msg.ch2)
        len_ch3 = len(msg.ch3)
        len_ch4 = len(msg.ch4)

        self.get_logger().info(
                    f"Received radar frame at {stamp.sec}.{stamp.nanosec:09d} "
                    f"| len(ch1–4): {len_ch1}, {len_ch2}, {len_ch3}, {len_ch4}"
                )
        
        self.get_logger().info(f"ch1 head: {msg.ch1[:5]}")

        # 1) Assemble a 2D array with shape (N, 4)
        td_data = self._assemble_td_numpy(msg)
        if td_data is None:
            # Logs are printed in _assemble_td_numpy()
            return
    
        # 2) header.stamp -> datetime (for filename)
        ts = self._stamp_to_datetime(stamp)

        # 3) save to TXT file
        save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        try:
            self.save_to_txt_file(td_data, save_dir, ts)
        except Exception as e:
            self.get_logger().error(f"Failed to save TXT: {e}")

        # Call this function here if saving as BIN is needed.

        # try:
        #     self.save_to_bin_file(td_data, save_dir, ts)
        # except Exception as e:
        #     self.get_logger().error(f"Failed to save BIN: {e}")

    def _assemble_td_numpy(self, msg: Sr14TdFrame):
        """Combine the four channels into a NumPy array of shape (N, 4) and perform a length consistency check."""
        ch1, ch2, ch3, ch4 = list(msg.ch1), list(msg.ch2), list(msg.ch3), list(msg.ch4)
        lengths = [len(ch1), len(ch2), len(ch3), len(ch4)]
        if min(lengths) == 0:
            self.get_logger().warn("One or more channels are empty; skip saving.")
            return None
        if len(set(lengths)) != 1:
            self.get_logger().error(f"Channel lengths mismatch: {lengths}; skip saving.")
            return None
        try:
            td = np.column_stack([ch1, ch2, ch3, ch4]).astype(np.float32, copy=False)
            return td
        except Exception as e:
            self.get_logger().error(f"Failed to stack channels into numpy array: {e}")
            return None

    def _stamp_to_datetime(self, stamp) -> datetime:
            """Convert builtin_interfaces/Time to a Python datetime (local timezone)."""
            # ROS time consists of epoch seconds plus nanoseconds; convert to UTC first, then to local time.
            dt = datetime.fromtimestamp(stamp.sec + stamp.nanosec * 1e-9, tz=timezone.utc).astimezone()
            return dt

    def save_to_txt_file(self, td_data: np.ndarray, folder_location: str, timestamp: datetime = None):
        """
        Save one frame of TD data to a .txt file (columns represent 4 channels, rows represent sampling points).
        td_data: (N, 4) float32
        """
        if not folder_location:
            folder_location = "./data"
        os.makedirs(folder_location, exist_ok=True)

        if timestamp is None:
            timestamp = datetime.now().astimezone()

        timestamp_str = timestamp.strftime('%Y-%m-%d_%H-%M-%S.%f')[:-3]
        file_name = f"TD_{timestamp_str}.txt"
        file_path = os.path.join(folder_location, file_name)

        header_str = (
            "Unit of the Time Domain Samples:\t[V]\n"
            "=======================================================\n"
            "<CH1>\t<CH2>\t<CH3>\t<CH4>\n\n"
        )

        # Use pandas to write as a tab-separated file.
        df = pd.DataFrame(td_data, columns=["CH1", "CH2", "CH3", "CH4"])
        with open(file_path, "w") as f:
            f.write(header_str)
            df.to_csv(f, sep='\t', index=False, header=False, float_format="%.4f")

        self.get_logger().info(f"Saved TD frame to: {file_path}")

    def save_to_bin_file(self, td_data: np.ndarray, folder_location: str, timestamp: datetime = None):
        """
        Optional: Save as binary (float32, stored row-wise as CH1..CH4 in sequence).
        """
        if not folder_location:
            folder_location = "./data"
        os.makedirs(folder_location, exist_ok=True)

        if timestamp is None:
            timestamp = datetime.now().astimezone()

        timestamp_str = timestamp.strftime('%Y-%m-%d_%H-%M-%S.%f')[:-3]
        file_name = f"TD_{timestamp_str}.bin"
        file_path = os.path.join(folder_location, file_name)

        td_data.astype(np.float32, copy=False).tofile(file_path)
        self.get_logger().info(f"Saved TD frame (binary float32) to: {file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = RadarFrameSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()