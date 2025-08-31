#!/usr/bin/env python3
import rclpy
from rclpy.node import Node     # Import ROS2 Node as parent for our own node class
from std_msgs.msg import Float32  # Import message types for publishing and subscribing
from turret_interfaces.srv import EnableTurret, DisableTurret  # Import service messages 
from sensor_msgs.msg import JointState

import threading
import time
import math
import serial
import turret_control_py.servo_packets as servo_packets  # Import the servo_packets module

class TurretControlNode(Node):
    def __init__(self):
        super().__init__("turret_controller_node")

        self.serialPortPan = '/dev/ttyAMA2'
        self.serialPortTilt = '/dev/ttyAMA3'

        self.serialPan = serial.Serial(
            port=self.serialPortPan,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.serialPan.close()
        self.serialPan.open()
        self.serialPan.isOpen()

        self.serialTilt = serial.Serial(
            port=self.serialPortTilt,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.serialTilt.close()
        self.serialTilt.open()
        self.serialTilt.isOpen()

        self.packetBufferPan = []
        self.packetBufferTilt = []

        self.encoderReadCycleTime = 0.05  # seconds, how often to read the encoders

        self.turretEnabled = True  # Flag to check if turret is enabled
        self.movementCooldown = 0.5  # seconds, cooldown time for movement commands

        self.panIsMove = False
        self.panSteps = 0
        self.panAngle = 0.0
        self.panSafeAngle = 0.0
        self.panAngleError = 0.0
        self.lastPanEncoderTime = time.time()   
        self.lastPanEncoderReadTime = time.time()
        self.panStoppedTime = time.time()
        self.tiltIsMove = False
        self.tiltSteps = 0
        self.tiltAngle = 0.0
        self.tiltSafeAngle = 0.0
        self.tiltAngleError = 0.0
        self.lastTiltEncoderTime = time.time()
        self.lastTiltEncoderReadTime = time.time()
        self.tiltStoppedTime = time.time()

        self.pan_comm_timer = self.create_timer(0.1, self.pan_packet_callback)
        self.tilt_comm_timer = self.create_timer(0.1, self.tilt_packet_callback)

        self.publisher_timer = self.create_timer(0.1, self.publisher_callback)
        self.get_logger().info("Turret controller has been started.")

        self.pan_angle_publisher = self.create_publisher(Float32, 'turret_pan_angle_feedback', 10)
        self.tilt_angle_publisher = self.create_publisher(Float32, 'turret_tilt_angle_feedback', 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.angleMsg = Float32()

        self.pan_angle_subscriber = self.create_subscription(Float32, 'turret_pan_angle_request', self.request_pan_angle_callback, 10)
        self.tilt_angle_subscriber = self.create_subscription(Float32, 'turret_tilt_angle_request', self.request_tilt_angle_callback, 10)

        self.enableService = self.create_service(EnableTurret, 'turret_enable', self.turret_enable_callback)
        self.disableService = self.create_service(DisableTurret, 'turret_disable', self.turret_disable_callback)

        # Start a separate thread for spinning
        self.running = True
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def turret_enable_callback(self, request, response):
        self.turretEnabled = True
        packet = servo_packets.enableMotor("pan", 1)
        self.packetBufferPan.append(packet)
        packet = servo_packets.homeMotor("pan")
        self.packetBufferPan.append(packet)
        packet = servo_packets.enableMotor("tilt", 1)
        self.packetBufferTilt.append(packet)
        packet = servo_packets.homeMotor("tilt")
        self.packetBufferTilt.append(packet)
        return response

    def turret_disable_callback(self, request, response):
        self.turretEnabled = False
        packet = servo_packets.enableMotor("pan", 0)
        self.packetBufferPan.append(packet)
        packet = servo_packets.enableMotor("tilt", 0)
        self.packetBufferTilt.append(packet)
        return response

    def request_pan_angle_callback(self, msg):
        if msg.data > 30:
            self.get_logger().info("Pan angle request is too high, limiting to 30 degrees.")
            msg.data = 30.0
        elif msg.data < -30:
            self.get_logger().info("Pan angle request is too low, limiting to -30 degrees.")
            msg.data = -30.0

        panDeltaAngle = msg.data - self.panSafeAngle  # convert turret pan angle to motor angle

        self.get_logger().info(f"Pan angle request: {msg.data}, current pan angle: {self.panSafeAngle}, delta: {panDeltaAngle}")

        if abs(panDeltaAngle) > 0.1:
            if panDeltaAngle < 0:
                packet = servo_packets.moveToAngle("pan", "ccw", int(abs(panDeltaAngle) * 4 * 64 / 1.8), 10)
            else:
                packet = servo_packets.moveToAngle("pan", "cw", int(abs(panDeltaAngle) * 4 * 64 / 1.8), 10)
            self.panSafeAngle += panDeltaAngle
            self.panIsMove = True
            self.packetBufferPan.append(packet)

        else:
            if self.panIsMove == False and time.time() - self.panStoppedTime > self.movementCooldown:
                self.get_logger().info(f"Pan safe angle updated from: {self.panSafeAngle} to: {self.panAngle / 4.0}")
                self.panSafeAngle = self.panAngle / 4.0

    def request_tilt_angle_callback(self, msg):
        if msg.data > 30:
            self.get_logger().info("Tilt angle request is too high, limiting to 30 degrees.")
            msg.data = 30.0
        elif msg.data < -30:
            self.get_logger().info("Tilt angle request is too low, limiting to -30 degrees.")
            msg.data = -30.0

        tiltDeltaAngle = msg.data - self.tiltSafeAngle  # convert turret tilt angle to motor angle

        self.get_logger().info(f"Tilt angle request: {msg.data}, current tilt angle: {self.tiltSafeAngle}, delta: {tiltDeltaAngle}")

        if abs(tiltDeltaAngle) > 0.1:
            if tiltDeltaAngle < 0:
                packet = servo_packets.moveToAngle("tilt", "ccw", int(abs(tiltDeltaAngle) * 5 * 64 / 1.8), 10)
            else:
                packet = servo_packets.moveToAngle("tilt", "cw", int(abs(tiltDeltaAngle) * 5 * 64 / 1.8), 10)
            self.tiltSafeAngle += tiltDeltaAngle
            self.tiltIsMove = True
            self.packetBufferTilt.append(packet)

        else:
            if self.tiltIsMove == False and time.time() - self.tiltStoppedTime > self.movementCooldown:
                self.get_logger().info(f"Tilt safe angle updated from: {self.tiltSafeAngle} to: {self.tiltAngle / 5.0}")
                self.tiltSafeAngle = self.tiltAngle / 5.0

    def publisher_callback(self):
        self.angleMsg.data = self.panSafeAngle
        self.pan_angle_publisher.publish(self.angleMsg)
        self.angleMsg.data = self.tiltSafeAngle
        self.tilt_angle_publisher.publish(self.angleMsg)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['pan_joint', 'tilt_joint']
        msg.position = [
            math.radians(self.panSafeAngle),
            math.radians(self.tiltSafeAngle)
        ]
        msg.velocity = []
        msg.effort = []
        
        self.joint_publisher.publish(msg)

    def pan_packet_callback(self):

        #read encoders only in every few hundreds ms
        if time.time() - self.lastPanEncoderReadTime > self.encoderReadCycleTime and self.turretEnabled:

            if self.panIsMove == False:
                packet = servo_packets.readAngle("pan")
                #packet = servo_packets.readEncoder("pan")
                self.packetBufferPan.append(packet)
                #packet = servo_packets.readSteps("pan")
                #self.packetBufferPan.append(packet)

            self.lastPanEncoderReadTime = time.time()

        try:

            if len(self.packetBufferPan) == 0:
                out = []
                while self.serialPan.inWaiting() > 0:
                    out.append(self.serialPan.read(1))

                if len(out) > 0:
                    print("Received data without packet in buffer:", out)
                    for i, byte in enumerate(out):
                        if int.from_bytes(byte) == int(0xe1):
                            if int.from_bytes(out[i+1]) == 2:
                                print("Pan movement finished: ", out[1])
                                self.panIsMove = False
                                self.panStoppedTime = time.time()
                        else:
                            print("Wrong data buffer:", out)


            while len(self.packetBufferPan) != 0:

                out = []

                packet = self.packetBufferPan.pop(0) 
                #print(packet)
                self.serialPan.write(packet)

                time.sleep(0.01)

                #print(self.serial.inWaiting())
                while self.serialPan.inWaiting() > 0:
                    out.append(self.serialPan.read(1))

                if out != '':

                    #TODO: checksum check
                    
                    #print(out)
                    # check if packet motor address matches with response motor address, just in case...
                    if self.panIsMove:
                        if int.from_bytes(out[0]) == int(0xe1):
                            if int.from_bytes(out[1]) == 1:
                                print("Pan movement started: ", out[1])
                                continue
                            elif int.from_bytes(out[1]) == 2:
                                print("Pan movement finished: ", out[1])
                                self.panIsMove = False
                                self.panStoppedTime = time.time()
                                continue
                            else:
                                raise TypeError("Unknown movement response on pan motor:", out)
                        else:
                            raise TypeError("Invalid motor ID in pan packet response", packet, out)

                    if int.from_bytes(out[0]) == packet[0]:
                        if hex(packet[0]) == "0xe1":
                            if hex(packet[1]) == "0x33":   # read the steps from the encoder
                                self.panSteps = servo_packets.processSteps("pan",out)
                                self.lastPanEncoderTime = time.time()
                            elif hex(packet[1]) == "0x30": # read the angle from the encoder
                                print(servo_packets.processEncoder("pan",out))
                            elif hex(packet[1]) == "0x36": # read the angle from the encoder
                                self.panAngle = servo_packets.processAngle("pan",out)
                                self.lastPanEncoderTime = time.time()
                            elif hex(packet[1]) == "0x39": # read the angle from the encoder
                                self.panAngleError = servo_packets.processAngleError("pan",out)
                            elif hex(packet[1]) == "0xfd": # run the motor to certain steps
                                pass #TODO
                            elif hex(packet[1]) == "0xf6": # run the motor with constant speed
                                pass #TODO 0x01 success
                            elif hex(packet[1]) == "0xf7": # stop the motor
                                pass #TODO 0x01 success
                            else:
                                raise TypeError("Unknown packet to process", packet, out)
                        else:
                            raise TypeError("Invalid motor ID", packet, out)
                    else:
                        raise TypeError("Motor ID mismatch!", packet, out)
                    pass
                else:
                    #print("empty")
                    pass

        except Exception as error:
                    print("Comm error occurred:", error)

    def tilt_packet_callback(self):

        #read encoders only in every few hundreds ms
        if time.time() - self.lastTiltEncoderReadTime > self.encoderReadCycleTime and self.turretEnabled:

            if self.tiltIsMove == False:
                packet = servo_packets.readAngle("tilt")
                self.packetBufferTilt.append(packet)
                #packet = servo_packets.readSteps("tilt")
                #self.packetBufferTilt.append(packet)

            self.lastTiltEncoderReadTime = time.time()

        try:

            if len(self.packetBufferTilt) == 0:
                out = []
                while self.serialTilt.inWaiting() > 0:
                    out.append(self.serialTilt.read(1))

                if len(out) > 0:
                    print("Received data without packet in buffer:", out)
                    for i, byte in enumerate(out):
                        if int.from_bytes(byte) == int(0xe0):
                            if int.from_bytes(out[i+1]) == 2:
                                print("Tilt movement finished: ", out[1])
                                self.tiltIsMove = False
                                self.tiltStoppedTime = time.time()
                        else:
                            print("Wrong data buffer:", out)


            while len(self.packetBufferTilt) != 0:

                out = []

                packet = self.packetBufferTilt.pop(0) 
                #print(packet)
                self.serialTilt.write(packet)

                time.sleep(0.01)

                #print(self.serial.inWaiting())
                while self.serialTilt.inWaiting() > 0:
                    out.append(self.serialTilt.read(1))

                if out != '':

                    #TODO: checksum check
                    
                    #print(out)
                    # check if packet motor address matches with response motor address, just in case...
                    if self.tiltIsMove:
                        if int.from_bytes(out[0]) == int(0xe0):
                            if int.from_bytes(out[1]) == 1:
                                print("Tilt movement started: ", out[1])
                                continue
                            elif int.from_bytes(out[1]) == 2:
                                print("Tilt movement finished: ", out[1])
                                self.tiltIsMove = False
                                self.tiltStoppedTime = time.time()
                                continue
                            else:
                                raise TypeError("Unknown movement response on tilt motor:", out)
                        else:
                            raise TypeError("Invalid motor ID in tilt packet response", packet, out)

                    if int.from_bytes(out[0]) == packet[0]:
                        if hex(packet[0]) == "0xe0":
                            if hex(packet[1]) == "0x33":   # read the steps from the encoder
                                self.tiltSteps = servo_packets.processSteps("tilt",out)
                                self.lastTiltEncoderTime = time.time()
                            elif hex(packet[1]) == "0x36": # read the angle from the encoder
                                self.tiltAngle = servo_packets.processAngle("tilt",out)
                                self.lastTiltEncoderTime = time.time()
                            elif hex(packet[1]) == "0x39": # read the angle from the encoder
                                self.tiltAngleError = servo_packets.processAngleError("tilt",out)
                            elif hex(packet[1]) == "0xfd": # run the motor to certain steps
                                pass #TODO
                            elif hex(packet[1]) == "0xf6": # run the motor with constant speed
                                pass #TODO 0x01 success
                            elif hex(packet[1]) == "0xf7": # stop the motor
                                pass #TODO 0x01 success
                            else:
                                raise TypeError("Unknown packet to process", packet, out)
                        else:
                            raise TypeError("Invalid motor ID", packet, out)
                    else:
                        raise TypeError("Motor ID mismatch!", packet, out)
                    pass
                else:
                    #print("empty")
                    pass

        except Exception as error:
                    print("Comm error occurred:", error)


    def run(self):
        self.get_logger().info("Turret control node is running.")

        packet = servo_packets.enableMotor("pan", 1)
        self.packetBufferPan.append(packet)
        packet = servo_packets.enableMotor("tilt", 1)
        self.packetBufferTilt.append(packet)

        time.sleep(1)  # Wait for motors to enable

        self.get_logger().info(f"Pan angle: {self.panAngle}, Pan steps: {self.panSteps}, Tilt angle: {self.tiltAngle}, Tilt steps: {self.tiltSteps}")
        if abs(self.panAngle) > 0.15 or abs(self.tiltAngle) > 0.15:
            self.get_logger().info("Turret angles are not zero, resetting...")
            if abs(self.panAngle) > 0.06:
                if self.panAngle > 0.06:
                    self.panIsMove = True
                    packet = servo_packets.moveToAngle("pan", "ccw", int(abs(self.panAngle) * 64.0 / 1.8), 5)
                    self.packetBufferPan.append(packet)
                else:
                    self.panIsMove = True
                    packet = servo_packets.moveToAngle("pan", "cw", int(abs(self.panAngle) * 64.0 / 1.8), 5)
                    self.packetBufferPan.append(packet)
            else:
                self.get_logger().info("Pan angle is already zero.")
            if abs(self.tiltAngle) > 0.06:
                if self.tiltAngle > 0.06:
                    self.tiltIsMove = True
                    packet = servo_packets.moveToAngle("tilt", "ccw", int(abs(self.tiltAngle) / 1.8 * 64), 5)
                    self.packetBufferTilt.append(packet)
                else:
                    self.tiltIsMove = True
                    packet = servo_packets.moveToAngle("tilt", "cw", int(abs(self.tiltAngle) / 1.8 * 64), 5)
                    self.packetBufferTilt.append(packet)
            else:
                self.get_logger().info("Tilt angle is already zero.")
        
            time.sleep(4)  # Wait for motors to move to zero position
            self.get_logger().info(f"Pan angle: {self.tiltAngle}, Pan steps: {self.panSteps}, Tilt angle: {self.tiltAngle}, Tilt steps: {self.tiltSteps}")

        else:
            self.get_logger().info("Turret angles are already zero, no need to reset.")

        self.panSafeAngle = self.panAngle / 4.0  # convert motor angle to turret pan angle
        self.tiltSafeAngle = self.tiltAngle / 5.0  # convert motor angle to turret tilt angle

        while rclpy.ok():
            #self.panIsMove = True
            #packet = servo_packets.moveToAngle("pan","ccw",int(30 * 4 * 64 / 1.8),5)
            #self.packetBuffer.append(packet)
            #self.tiltIsMove = True
            #packet = servo_packets.moveToAngle("tilt","ccw",int(30 * 5 * 64 / 1.8),5)
            #self.packetBuffer.append(packet)
            self.get_logger().info(f"Pan angle: {self.panAngle}, Pan steps: {self.panSteps}, Tilt angle: {self.tiltAngle}, Tilt steps: {self.tiltSteps}")
            time.sleep(1)

        self.running = False

    def stop(self):
        self.running = False
        self.spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = TurretControlNode() # node is now a custom class based on ROS2 Node

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