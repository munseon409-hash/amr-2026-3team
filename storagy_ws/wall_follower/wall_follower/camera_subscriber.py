import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from wall_follower_msgs.msg import Stop
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime
import time

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10
        )
        self.sign_pub = self.create_publisher(
            Stop, '/sign', 10
        )

        model_path = '/home/storagy/Desktop/storagy_ws/src/wall_follower/wall_follower/best.onnx'
        self.session = onnxruntime.InferenceSession(model_path, providers=['CPUExecutionProvider'])

        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]
        self.input_shape = self.session.get_inputs()[0].shape  # e.g., [1, 3, 416, 416]

        self.get_logger().info('Camera subscriber node started and ONNX model loaded.')

    def preprocess(self, image, size=640):
        # Resize and normalize image
        image = cv2.resize(image, (size, size))
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32)
        img = img / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)  # Add batch dimension
        return img.astype(np.float32)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            input_tensor = self.preprocess(cv_image)

            # ONNX 추론
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
            pred = outputs[0]


            # post-processing: confidence thresholding
            boxes = pred[pred[..., 4] > 0.85]  # confidence score filter
            if boxes.any():
                stop_msg = Stop()
                stop_msg.sign = True
                self.sign_pub.publish(stop_msg)
                print("stop")
                time.sleep(6)

            else:
                pass

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
