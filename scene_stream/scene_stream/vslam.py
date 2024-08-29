import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Image

class PicBuffer:
    def __init__(self):
        self.buffer = []
        self.max_size = 10

    def add(self, pic):
        if len(self.buffer) >= self.max_size:
            self.buffer.pop(0)
        self.buffer.append(pic)

    def pop2(self):
        # returns the first and last elements of the buffer
        if len(self.buffer) == self.max_size:
            return self.buffer[0], self.buffer[-1]
        else:
            return None, None

    def clear(self):
        self.buffer.clear()

class VslamNode(Node):
    def __init__(self, buf, pub_freq=10, visualize=False):
        super().__init__('vslam_node')
        self.sub = self.create_subscription(Image, '/ros2socket/fromsocket', self.image_callback, 10)
        self.buffer = buf
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.pub = self.create_publisher(MarkerArray, '/vslam/markers', 10)
        self.pub_freq = pub_freq
        self.timer = self.create_timer(1.0 / pub_freq, self.timer_callback)
        self.markers = MarkerArray()
        self.current_marker_id = 0
        self.R = None
        self.T = None
        self.visualize = visualize

    def timer_callback(self):
        self.pub.publish(self.markers)
        self.markers.markers.clear()

    def image_callback(self, msg):
        imgdata = msg.data
        imgtime = msg.header.stamp
        np_arr = np.frombuffer(imgdata, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            self.buffer.add(image)
            imga, imgb = self.buffer.pop2()
            if imga is None or imgb is None:
                return
            # TODO: VSLAM processing
            # Convert image to grayscale
            graya = cv2.cvtColor(imga, cv2.COLOR_BGR2GRAY)
            grayb = cv2.cvtColor(imgb, cv2.COLOR_BGR2GRAY)

            # Perform feature detection (e.g., using ORB)
            keypointsa, descriptorsa = self.orb.detectAndCompute(graya, None)
            keypointsb, descriptorsb = self.orb.detectAndCompute(grayb, None)

            # if any of the images has no keypoints, return
            if len(keypointsa) == 0 or len(keypointsb) == 0:
                return
            
            # if any of the descriptors is None, return
            if descriptorsa is None or descriptorsb is None:
                return

            # Perform feature matching (e.g., using Brute-Force Matcher)
            matches = self.bf.match(descriptorsa, descriptorsb)

            # Sort matches by distance
            matches = sorted(matches, key=lambda x: x.distance)

            # Select top matches (e.g., 50)
            last = min(50, len(matches))
            top_matches = matches[:last]

            # Extract matched keypoints
            src_pts = np.float32([keypointsa[m.queryIdx].pt for m in top_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([keypointsb[m.trainIdx].pt for m in top_matches]).reshape(-1, 1, 2)

            # intrinsics matrix: hard coded for now.
            # given that focal length in pixels is 3072, and the image size is 4096x3072
            intrinsics = np.array([[3072, 0, 2048], [0, 3072, 1536], [0, 0, 1]])

            # Compute the essential matrix using opencv
            essential_matrix, _ = cv2.findEssentialMat(src_pts, dst_pts, intrinsics, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            
            # Check if the essential matrix is valid
            if essential_matrix is None:
                return

            if essential_matrix.shape[0] != 3 or essential_matrix.shape[1] != 3:
                print("Failed to find essential matrix")
                return

            # Get relative motion between the two frames
            retval, R, T, mask = cv2.recoverPose(essential_matrix, src_pts, dst_pts, intrinsics)

            # accumulate the relative motion
            if self.R is not None and self.T is not None:
                self.R = np.dot(self.R, R)
                self.T = self.T + np.dot(self.R, T)
            else:
                self.R = R
                self.T = T

            if self.visualize:
                # Draw matches
                result = cv2.drawMatches(imga, keypointsa, imgb, keypointsb, top_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                # Display the results
                cv2.imshow("VSLAM Result", result)
                cv2.waitKey(1)

            # Publish the relative motion as a marker
            marker = Marker()
            marker.id = self.current_marker_id
            self.current_marker_id += 1
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = float(self.T[0])
            marker.pose.position.y = float(self.T[1])
            marker.pose.position.z = float(self.T[2])
            r_quat = Rotation.from_matrix(self.R).as_quat()
            marker.pose.orientation.x = r_quat[0]
            marker.pose.orientation.y = r_quat[1]
            marker.pose.orientation.z = r_quat[2]
            marker.pose.orientation.w = r_quat[3]
            marker.scale.x = 0.1
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.markers.markers.append(marker)
        else:
            print("Failed to decode image")

def main(args=None):
    rclpy.init(args=args)
    try:
        buffer = PicBuffer()
        vslam_node = VslamNode(buffer, pub_freq=10, visualize=False)
        rclpy.spin(vslam_node)
        vslam_node.destroy_node()
    except KeyboardInterrupt:
        if rclpy.ok():
            rclpy.shutdown()
        exit(0)
    except Exception as e:
        #print("Error in vslam_node: ", e)
        raise e
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()