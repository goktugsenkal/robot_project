import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os

def callback(data):
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        # Save image to the user's home directory
        home_dir = os.path.expanduser('~')
        file_path = os.path.join(home_dir, 'output_image.png')
        cv2.imwrite(file_path, cv_image)
        rospy.loginfo(f"Image saved to {file_path}")

    except Exception as e:
        rospy.logerr(f"Failed to process image: {e}")

def image_listener():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/image_topic', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    image_listener()

