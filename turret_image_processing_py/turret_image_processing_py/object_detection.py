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

        self.declare_parameter('inference_width', 448) # 448, 864
        self.declare_parameter('inference_height', 256) # 256, 480
        self.declare_parameter('classes', []) # [0, 15] for cats and persons
        self.declare_parameter('confidence', 0.1)

        self.inference_width = int(self.get_parameter('inference_width').value)
        self.inference_height = int(self.get_parameter('inference_height').value)
        self.classes = list(self.get_parameter('classes').value)
        self.confidence = float(self.get_parameter('confidence').value)
        self.inference_width_ratio = None
        self.inference_height_ratio = None

        # Always generate the same golors for the first n base IDs,then we'll keep generating random colors
        self.id_colors = sup_lib.generate_random_colors(20, fix_seed=True)

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

        if height != self.inference_height or width != self.inference_width:
            dim = (self.inference_width, self.inference_height)
            self.inference_height_ratio = float(height) / self.inference_height
            self.inference_width_ratio = float(width) / self.inference_width
            img_resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

        results = self.model.track(
            img_resized,
            persist = True,
            conf=self.confidence,
            imgsz = (self.inference_height, self.inference_width),
            classes=self.classes if self.classes else None
        )

        annotated_img = self.post_process_image(img, self.latest_depth_frame, results[0].boxes, self.inference_width_ratio, self.inference_height_ratio)
        #annotated_img = results[0].plot()

        return annotated_img

    def post_process_image(self, img, depth, boxes, width_ratio, height_ratio):
        cls = boxes.cls
        conf = boxes.conf
        ids = boxes.id
        xyxy = boxes.xyxy
        xywh = boxes.xywh

        result = img.copy()

        if ids == None:
            return result

        if len(ids) == 0:
            return result

        print(ids)

        for i, id_tensor in enumerate(ids):
            id = int(id_tensor.item())
            cl = int(cls[i].item())
            cf = float(conf[i].item())
            # if id is higher than pregenerated id color list we add new random colors to the list
            if id > len(self.id_colors)-1:
                self.id_colors+=sup_lib.generate_random_colors(id-len(self.id_colors)+1, fix_seed=False)
            cv2.rectangle(result, (int(xyxy[i][0]*width_ratio), int(xyxy[i][1]*height_ratio)), (int(xyxy[i][2]*width_ratio), int(xyxy[i][3]*height_ratio)), self.id_colors[id], 3)

            label_text = f"ID: {id} - {sup_lib.decode_class(cl)} - {cf*100:.3}%"
            label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
            label_left = int(xyxy[i][0]*width_ratio)
            label_top = int(xyxy[i][1]*height_ratio) - label_size[1]
            if (label_top < 1):
                label_top = 1
            label_right = label_left + label_size[0]
            label_bottom = label_top + label_size[1] - 3
            cv2.rectangle(result, (label_left - 1, label_top - 6), (label_right + 1, label_bottom + 1),
                        self.id_colors[id], -1)
            cv2.putText(result, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.6, sup_lib.best_text_color(self.id_colors[id])[::-1], 1, cv2.LINE_AA)
        

        return result



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
