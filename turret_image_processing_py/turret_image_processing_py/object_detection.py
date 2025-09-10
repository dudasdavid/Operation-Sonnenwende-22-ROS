import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
import ultralytics
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
import turret_image_processing_py.support_lib as sup_lib 

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        self.get_logger().info(f"Ultralytics version: {ultralytics.__version__}")

        self.rgb_subscription = self.create_subscription(
            CompressedImage,
            'rgbd/color/image_raw/compressed',  # Replace with your topic name
            self.rgb_image_callback,
            1  # Queue size of 1
        )

        self.depth_subscription = self.create_subscription(
            CompressedImage,
            'rgbd/aligned_depth_to_color/image_raw/compressedDepth',  # Replace with your topic name
            self.depth_image_callback,
            1  # Queue size of 1
        )

        self.ir_subscription = self.create_subscription(
            CompressedImage,
            'rgbd/infra1/image_rect_raw/compressed',  # Replace with your topic name
            self.ir_image_callback,
            1  # Queue size of 1
        )

        pkg_image_processing_models = get_package_share_directory('turret_image_processing_models')
        model_path = os.path.join(pkg_image_processing_models, 'yolo11', 'yolo11n.pt'),

        #self.model = ultralytics.YOLO('yolo11n.pt')
        self.model = ultralytics.YOLO(model_path[0])
        self.get_logger().info(f"Model path: {self.model.ckpt_path}")

        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frames
        self.latest_rgb_frame = None
        self.latest_depth_frame = None
        self.latest_ir_frame = None
        self.rgb_frame_lock = threading.Lock()  # Lock to ensure thread safety
        self.depth_frame_lock = threading.Lock()
        self.ir_frame_lock = threading.Lock()

        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def rgb_image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.rgb_frame_lock:
            self.latest_rgb_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        with self.depth_frame_lock:
            self.latest_depth_frame = sup_lib.on_depth(msg)

    def ir_image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.ir_frame_lock:
            self.latest_ir_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def display_image(self):

        # Create a single OpenCV window
        cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame", 848,480)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_rgb_frame is not None:

                # Process the current image
                processed_frame = self.process_image(self.latest_rgb_frame)

                # Show the latest frame
                cv2.imshow("Frame", processed_frame)
                self.latest_rgb_frame = None  # Clear the frame after displaying

            if self.latest_depth_frame is not None:
                # Show the latest frame
                print(self.latest_depth_frame[int(480/2)][int(848/2)])
                cv2.imshow("Depth (m)", self.latest_depth_frame / np.nanmax(self.latest_depth_frame))
                self.latest_depth_frame = None  # Clear the frame after displaying

            if self.latest_ir_frame is not None:
                # Show the latest frame
                cv2.imshow("IR", self.latest_ir_frame)
                self.latest_ir_frame = None  # Clear the frame after displaying

            # Check for quit key
            key = cv2.waitKey(6) & 0xFF
            if key in [27, ord('q')]: # 27 = ESC
                self.running = False
                break
            elif key in [32, ord('s')]: # 32 = Space
                pass

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):

        height, width = img.shape[:2]

        if height != 480 or width != 864:
            dim = (864, 480)
            img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

        '''
        Classes:
        0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
        5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
        10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
        14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
        20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
        25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
        30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite',
        34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard',
        37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 40: 'wine glass',
        41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana',
        47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot',
        52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair',
        57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table',
        61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote',
        66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven',
        70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock',
        75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier',
        79: 'toothbrush'}
        '''

        results = self.model.track(img, persist = True, conf=0.1, imgsz = (480, 864))#, classes = [0,15])
        #print(results[0].boxes.cls) # cls, conf, id, xyxy, xywh
        annotated_img = results[0].plot()
        
        return annotated_img

    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()

def main(args=None):

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
