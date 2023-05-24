#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

class JSFiltro:
	def __init__(self):
		rospy.init_node('joystick_filtro')
		#Frecuencia de muestreo del joystick en HZ
		self.sample_rate = 60
		
		#Constante de tiempo para el filtro de primer orden(alpha)
		self.time_constant = 0.02
		self.last_filtered_value=0.0
		
		#Suscriber para un dato en el eje del joystick
		rospy.Subscriber('joy', Joy, self.joy_callback)
		
		#Publisher para la señal filtrada
		self.f_pub=rospy.Publisher('filtered', Float32, queue_size=10)
	
	def joy_callback(self, data):
		#Adquirimos señal de Joystick
		joy_signal=data.axes[1]
		
		#Ecuacion de filtro de primer orden de tipo IIR
		alpha=self.time_constant/(self.time_constant+(1.0/self.sample_rate))
		
		filtered_value=alpha * joy_signal + (1. - alpha) *self.last_filtered_value
		self.last_filtered_value = filtered_value
		#Publish la señal filtrada
		self.f_pub.publish(filtered_value)

if __name__ == '__main__':
	try:
		joystick_filter = JSFiltro()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass	
