import cv2
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def depth_callback(depth_msg):
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    
    # 定义要获取距离的像素点 (x, y)
    x = 320  # 水平方向像素位置
    y = 240  # 垂直方向像素位置

    # 获取该像素点的深度值
    distance = depth_image[y, x]  # 请注意，numpy 数组的索引是 [y, x]
    
    print(f"距离 ({x}, {y}) 的距离是 {distance} 毫米")

def image_callback(image_msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    # 在x=320，y=10的位置画一个红点
    cv2.circle(image, (320, 240), 5, (0, 0, 255), -1)
    cv2.imshow("image", image)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('depth_listener', anonymous=True)
    
    # 订阅深度图像话题
    depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
    
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    
    rospy.spin()
