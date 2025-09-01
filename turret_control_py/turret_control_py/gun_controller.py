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

        self.serialPort = '/dev/ttyAMA4'

        self.serial = serial.Serial(
            port=self.serialPort,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.serial.close()
        self.serial.open()
        self.serial.isOpen()

        self.packetBuffer = []

        self.shootingStartTime = time.time()
        self.shootingCooldown = 3.0  # seconds, cooldown time for shooting

        self.comm_timer = self.create_timer(1, self.packet_callback)

        self.get_logger().info("Gun controller has been started.")

        self.feedback_publisher = self.create_publisher(GunFeedback, 'turret_gun_feedback', 10)
        self.feedbackMsg = GunFeedback()

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
            self.packetBuffer.append(bytearray("SHOT:" + str(msg.data) + "\r", "utf-8"))
        else:
            self.get_logger().info("Shooting is on cooldown, please wait.")

    def packet_callback(self):

        self.packetBuffer.append(bytearray("STAT\r", "utf-8"))

        while len(self.packetBuffer) != 0:
            out = []

            packet = self.packetBuffer.pop(0) 
            self.serial.write(packet)

            time.sleep(0.05)

            timeoutStart = time.time()
            communicationError = False
            while b"".join(out[-5:]) != b"END\r\n":
                #print(b"".join(out[-5:]))
                out.append(self.serial.read(1))
                if time.time() - timeoutStart > 2.0:
                    self.get_logger().error("Timeout waiting for gun response.")
                    communicationError = True
                    break

            if communicationError:
                self.get_logger().warning("Skipping processing of this gun packet due to communication error.")
                continue

            if out != []:
                #print("Response:")
                #print(b"".join(out).decode("utf-8").rstrip("\r\n"))
                if b"".join(out[0:5]) == b"STAT:":
                    #self.get_logger().info("Gun status packet received.")
                    #self.get_logger().info(b"".join(out).decode("utf-8").rstrip("\r\n"))
                    arrayToProcess = b"".join(out).decode("utf-8").rstrip("\r\n")[5:].split(";")
                    #print(arrayToProcess)
                    ammo = int(arrayToProcess[0])
                    distance = int(arrayToProcess[1])
                    voltage = int(arrayToProcess[2]) * 0.1  # convert to volts
                    magazine = arrayToProcess[3]
                    if magazine == "True":
                        magazine = True
                    else:
                        magazine = False
                    
                    self.feedbackMsg.ammo = ammo
                    self.feedbackMsg.distance_mm = distance
                    self.feedbackMsg.voltage = voltage
                    self.feedbackMsg.magazine = magazine

                    self.get_logger().info(f"Gun Feedback - Ammo: {ammo}, Distance: {distance} mm, Voltage: {voltage:.1f} V, Magazine: {magazine}")


                else:
                    self.get_logger().error("Received unknown gun packet response: " + b"".join(out).decode("utf-8"))
        
                self.feedback_publisher.publish(self.feedbackMsg)
            else:
                self.get_logger().error("No response received from gun.")

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