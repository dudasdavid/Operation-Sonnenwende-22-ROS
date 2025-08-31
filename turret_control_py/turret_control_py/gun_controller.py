#!/usr/bin/env python3
import rclpy
from rclpy.node import Node     # Import ROS2 Node as parent for our own node class
from std_msgs.msg import UInt8  # Import message types for publishing and subscribing
from turret_interfaces.msg import GunFeedback

import threading
import time
import serial
import turret_control_py.servo_packets as servo_packets  # Import the servo_packets module

class GunControlNode(Node):
    def __init__(self):
        super().__init__("gun_controller_node")

        self.serialPortGun = '/dev/ttyAMA4'

        self.serialGun = serial.Serial(
            port=self.serialPortGun,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.serialGun.close()
        self.serialGun.open()
        self.serialGun.isOpen()

        self.packetBufferGun = []

        self.shootingStartTime = time.time()
        self.shootingCooldown = 3.0  # seconds, cooldown time for shooting

        self.gun_comm_timer = self.create_timer(2, self.gun_packet_callback)

        self.get_logger().info("Turret controller has been started.")

        self.gun_feedback_publisher = self.create_publisher(GunFeedback, 'turret_gun_feedback', 10)
        self.gunFeedbackMsg = GunFeedback()

        self.shoot_subscriber = self.create_subscription(UInt8, 'turret_shoot_request', self.request_shoot_callback, 10)

        # Start a separate thread for spinning
        self.running = True
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def request_shoot_callback(self, msg):
        if time.time() - self.shootingStartTime > self.shootingCooldown:
            if msg.data > 5:
                self.get_logger().info("Shooting request is too high, limiting to 5 shots.")
                msg.data = 5
            self.get_logger().info(f"Shooting request received for {msg.data} shots.")
            self.shootingStartTime = time.time()
            self.packetBufferGun.append(bytearray("SHOT:" + str(msg.data) + "\r", "utf-8"))
        else:
            self.get_logger().info("Shooting is on cooldown, please wait.")

    def gun_packet_callback(self):

        self.packetBufferGun.append(bytearray("AMMO\r", "utf-8"))
        self.packetBufferGun.append(bytearray("DIST\r", "utf-8"))
        self.packetBufferGun.append(bytearray("MAG\r", "utf-8"))

        while len(self.packetBufferGun) != 0:
            out = []

            packet = self.packetBufferGun.pop(0) 
            #print(packet)
            self.serialGun.write(packet)

            time.sleep(0.1)

            #print(self.serialGun.inWaiting())
            while self.serialGun.inWaiting() > 0:
                out.append(self.serialGun.read(1))

            if out != []:
                #print("Response:")
                #print(b"".join(out).decode("utf-8").rstrip("\r\n"))
                if "AMMO:" in b"".join(out).decode("utf-8"):
                    try:
                        ammo = int(b"".join(out).decode("utf-8").split("AMMO:")[1].split("\r")[0])
                        self.gunFeedbackMsg.ammo = ammo
                        #self.get_logger().info(f"Ammo: {ammo}")
                    except ValueError:
                        self.get_logger().error("Invalid AMMO message:")
                        self.get_logger().error(b"".join(out).decode("utf-8").split(":")[1].split("\r")[0])
                        self.get_logger().error(b"".join(out).decode("utf-8"))
                elif "DIST:" in b"".join(out).decode("utf-8"):
                    try:
                        distance = int(b"".join(out).decode("utf-8").split("DIST:")[1].split("\r")[0])
                        self.gunFeedbackMsg.distance_mm = distance
                        #self.get_logger().info(f"Distance: {distance} mm")
                    except ValueError:
                        self.get_logger().error("Invalid DIST message:")
                        self.get_logger().error(b"".join(out).decode("utf-8").split(":")[1].split("\r")[0])
                        self.get_logger().error(b"".join(out).decode("utf-8"))
                elif "MAG:" in b"".join(out).decode("utf-8"):
                    magazine = b"".join(out).decode("utf-8").split("MAG:")[1].split("\r")[0].strip()
                    if magazine == "True":
                        magazine = True
                    else:
                        magazine = False
                    self.gunFeedbackMsg.magazine = magazine
                    #self.get_logger().info(f"Magazine: {magazine}")
                elif "VOLT:" in b"".join(out).decode("utf-8"):
                    voltage = float(b"".join(out).decode("utf-8").split("VOLT:")[1].split("\r")[0])
                    self.gunFeedbackMsg.voltage = voltage
                    #self.get_logger().info(f"Voltage: {voltage} V")
                    
                else:
                    self.get_logger().error("Received unknown gun packet response: " + b"".join(out).decode("utf-8"))
            
                self.gun_feedback_publisher.publish(self.gunFeedbackMsg)
            else:
                pass
                #print("Empty gun packet response")

            time.sleep(0.1)  # Give some time for the gun to process the command


    def run(self):
        self.get_logger().info("Gun control node is running.")

        while rclpy.ok():
            time.sleep(1)

        self.running = False

    def stop(self):
        self.running = False
        self.spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = GunControlNode() # node is now a custom class based on ROS2 Node

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()