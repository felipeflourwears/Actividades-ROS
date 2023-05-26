import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

# Definir el diccionario de marcadores ArUco
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

def image_callback(msg):
    bridge = CvBridge()
    # Convierte la imagen recibida en formato ROS a formato OpenCV
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Convertir el fotograma a escala de grises
    gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

    # Detectar los marcadores en la imagen
    markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(gray, aruco_dict)

    # Verificar si se encontraron marcadores
    if markerIds is None:
        print("No se encontraron marcadores ArUco en la imagen.")
    else:
        if len(markerIds) > 0:
            # Obtener el código del primer marcador detectado
            codigo_marcador = markerIds[0][0]

            # Dibujar los marcadores detectados en la imagen
            image_with_markers = cv.aruco.drawDetectedMarkers(cv_image.copy(), markerCorners, markerIds)

            # Mostrar la imagen con los marcadores en una ventana separada
            cv.imshow('Image with Markers', image_with_markers)

            # Imprimir el código del marcador detectado
            print("Código del marcador: ", codigo_marcador)

    # Esperar 1 milisegundo para refrescar la ventana
    cv.waitKey(1)

def main():
    rospy.init_node('image_viewer_node', anonymous=True)
    # Suscríbete al tópico de la imagen
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
