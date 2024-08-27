import rclpy
import cv2
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import Image

class VslamNode(Node):
    def __init__(self):
        super().__init__('vslam_node')
        self.sub = self.create_subscription(Image, '/ros2socket/fromsocket', self.image_callback, 10)

    def image_callback(self, msg):
        imgdata = msg.data
        print("Received image length: ", len(imgdata))
        imgtime = msg.header.stamp
        np_arr = np.frombuffer(imgdata, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            print("Decoded image shape: ", image.shape)
            cv2.imshow("Video Stream", image)
            cv2.waitKey(1)
        else:
            print("Failed to decode image")

def main(args=None):
    rclpy.init(args=args)
    try:
        vslam_node = VslamNode()
        rclpy.spin(vslam_node)
        vslam_node.destroy_node()
    except KeyboardInterrupt:
        if rclpy.ok():
            rclpy.shutdown()
        exit(0)
    except Exception as e:
        print("Error in vslam_node: ", e)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()