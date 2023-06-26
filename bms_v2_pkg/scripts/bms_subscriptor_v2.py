#!/usr/bin/python3

import rospy
from sensor_msgs.msg import BatteryState

def battery_state_callback(msg):
    # Procesa los datos recibidos desde el topic "battery_state"
    voltage = msg.voltage
    current = msg.current
    soc = msg.percentage
    #min_cell_voltage = msg.min_cell_voltage
    #max_cell_voltage = msg.max_cell_voltage
    #min_cell_voltage = msg.min_volt
    #max_cell_voltage = msg.max_volt
    # Realiza cualquier acción o cálculo necesario con los datos recibidos
    
    # Muestra los datos en la consola
    print('Battery State: voltage={}, current={}, soc={}, min_cell_voltage={}, max_cell_voltage={}'.format(
        voltage,
        current,
        soc,
        #min_cell_voltage,
        #max_cell_voltage
        #min_volt,
        #max_volt
    ))

def battery_state_subscriber():
    # Inicializa el nodo del suscriptor
    rospy.init_node('battery_state_subscriber')
    
    # Crea un suscriptor para el topic "battery_state" con la función de callback correspondiente
    rospy.Subscriber('battery_state', BatteryState, battery_state_callback)
    
    # Espera a que lleguen mensajes
    rospy.spin()

if __name__ == '__main__':
    battery_state_subscriber()
