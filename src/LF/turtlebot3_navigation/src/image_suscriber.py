import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    # Convierte la imagen recibida en formato ROS a formato OpenCV
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # Muestra la imagen utilizando OpenCV
    cv2.imshow("Image", cv_image)
    cv2.waitKey(1)  # Espera 1 milisegundo para refrescar la ventana

def main():
    rospy.init_node('image_viewer_node', anonymous=True)
    # Suscríbete al tópico de la imagen
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
