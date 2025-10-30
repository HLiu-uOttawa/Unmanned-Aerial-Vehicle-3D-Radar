#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Sr14TdFrame

import os
from datetime import datetime

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

    def save_to_txt_file(td_data, folder_location: str, timestamp: datetime = None):
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

    def save_to_bin_file(td_data, folder_location: str, timestamp: datetime = None):
        ...

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