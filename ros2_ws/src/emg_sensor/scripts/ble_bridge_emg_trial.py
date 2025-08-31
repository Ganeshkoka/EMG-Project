#!/usr/bin/env python3
import asyncio, rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from bleak import BleakClient, BleakScanner

ADDR    = "F7:DC:B8:38:6D:F6"
UART_RX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
ADAPTER = "hci1"

class BLEHandler:
    """
    This class handles only the BLE logic (scanning, connecting, handling data).
    It does not inherit from rclpy.node.Node.
    """
    def __init__(self, ros_node: Node):
        self.node = ros_node
        self.pub = self.node.create_publisher(UInt8MultiArray, "emg_raw", 10)

    async def run_ble_loop(self):
        self.node.get_logger().info("Starting BLE loop...")
        while rclpy.ok():
            try:
                self.node.get_logger().info("Scanning for device...")
                device = await BleakScanner.find_device_by_address(ADDR, timeout=20.0, adapter=ADAPTER)
                if device is None:
                    self.node.get_logger().error(f"Device with address {ADDR} not found. Retrying in 10s...")
                    await asyncio.sleep(10)
                    continue

                self.node.get_logger().info(f"Connecting to {device.name} ({device.address})")
                async with BleakClient(device, adapter=ADAPTER) as client:
                    if client.is_connected:
                        self.node.get_logger().info(f"Connected to {device.name}")
                        await client.start_notify(UART_RX, self.handle_ble_data)
                        # Keep the connection alive while ROS is running
                        while client.is_connected and rclpy.ok():
                            await asyncio.sleep(1.0)
                        self.node.get_logger().warn("Device disconnected.")
                    else:
                        self.node.get_logger().error("Failed to connect.")

            except Exception as e:
                self.node.get_logger().error(f"BLE loop error: {e}")
                await asyncio.sleep(5) # Wait before retrying

    def handle_ble_data(self, _sender, data: bytearray):
        """
        This is the callback that receives data from the nRF board.
        """
        self.node.get_logger().info(f'Received: {data.hex()}')
        # Create a ROS message and publish it
        msg = UInt8MultiArray(data=list(data))
        self.pub.publish(msg)


async def main(args=None):
    rclpy.init(args=args)

    # Create the ROS node first. This is the guaranteed safe way.
    ble_bridge_node = Node("ble_bridge")

    # Now create the BLE handler and pass the node to it.
    ble_handler = BLEHandler(ble_bridge_node)

    # Use a separate thread for the ROS spinner
    ros_thread = threading.Thread(target=rclpy.spin, args=(ble_bridge_node,), daemon=True)
    ros_thread.start()

    # Run the main asyncio BLE loop in the main thread
    try:
        await ble_handler.run_ble_loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        ble_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # need to import threading for this to work
    import threading
    asyncio.run(main())