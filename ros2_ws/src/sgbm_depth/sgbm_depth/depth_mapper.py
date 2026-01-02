import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import math

class DepthMapper(Node):
    def __init__(self):
        super().__init__("depth_mapper")
        
        self.bridge = CvBridge()
        self.focal = None
        self.baseline = None
        self.proj_const = None
        self.cam_info_msg = None

        # tf, frames
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.left_frame = "zed_left_camera_frame_optical"
        self.right_frame = "zed_right_camera_frame_optical"

        # sgbm stereo matcher
        block_size = 5
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=block_size,
            P1=8 * 1 * block_size**2,
            P2=32 * 1 * block_size**2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
            speckleWindowSize=100,
            speckleRange=2
        )

        # pubs, subs
        self.depth_pub = self.create_publisher(Image, "/depth", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/depth/camera_info", 10)

        self.info_sub = self.create_subscription(
            CameraInfo, "/zed/zedxm/left/gray/rect/image/camera_info", self.info_cb, 10
        )

        left_sub = message_filters.Subscriber(self, Image, "/zed/zedxm/left/gray/rect/image")
        right_sub = message_filters.Subscriber(self, Image, "/zed/zedxm/right/gray/rect/image")

        self.sync = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.05)
        self.sync.registerCallback(self.stereo_cb)
        
        self.get_logger().info("DepthMapper: Initialized")

    def info_cb(self, msg):
        if self.cam_info_msg is None:
            self.cam_info_msg = msg
            self.focal = msg.k[0]
            self.destroy_subscription(self.info_sub)
            self.get_logger().info(f"Focal: {self.focal:.2f} px")

    def update_baseline(self):
        if self.baseline is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                self.left_frame, self.right_frame, rclpy.time.Time()
            )
            t = tf.transform.translation
            self.baseline = math.sqrt(t.x**2 + t.y**2 + t.z**2)
            
            if self.focal:
                self.proj_const = self.focal * self.baseline
                self.get_logger().info(f"Baseline: {self.baseline:.4f} m")
                return True
        except tf2_ros.TransformException:
            pass
        return False

    def stereo_cb(self, left_msg, right_msg):
        if self.proj_const is None:
            if not self.update_baseline() or self.cam_info_msg is None:
                return

        left = self.bridge.imgmsg_to_cv2(left_msg, "mono8")
        right = self.bridge.imgmsg_to_cv2(right_msg, "mono8")

        # div by 16.0 for SGBM fixed-point scaling
        disp = self.stereo.compute(left, right).astype(np.float32) / 16.0

        # vectorized depth calc (focal * baseline / disp)
        depth = np.full_like(disp, np.nan)
        np.divide(self.proj_const, disp, out=depth, where=disp > 0)

        # publish depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth, "32FC1")
        depth_msg.header = left_msg.header
        self.depth_pub.publish(depth_msg)

        # pub cam info, update the header to match the current frame
        self.cam_info_msg.header = left_msg.header 
        self.info_pub.publish(self.cam_info_msg)

def main():
    rclpy.init()
    node = DepthMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
