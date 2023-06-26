#!/usr/bin/python3

import sys
import asyncio
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import BatteryState
from bms_v3_pkg.msg import CustomBatteryState
#import aiohttp
import requests

def battery_state_callback(msg) :

	battery_voltage = msg.voltage
	battery_current = msg.current
        battery_soc = msg.percentage
        lowest_cell_voltage = msg.min_cell_voltage
        highest_cell_voltage = msg.max_cell_voltage
        lowest_cell_temperature = msg.min_cell_temperature
        highest_cell_temperature = msg.max_cell_temperature
        
        url = 'https://api.thingspeak.com/update'
        api_key = KZWLB267YFY9TQWB
        
        params = {
        'field1': str(battery_voltage),
        'field2': str(battery_current),
        'field3': str(battery_soc),
        'field4': str(lowest_cell_voltage),
        'field5': str(highest_cell_voltage),
        'field6': str(lowest_cell_temperature),
        'field7': str(highest_cell_temperature)

    	}
    	response = requests.post(url + '?api_key=' + api_key, params=params)

    	if response.status_code == 200:
        	rospy.loginfo('Datos enviados a ThingSpeak con éxito')
    	else:
        	rospy.logwarn('Error al enviar los datos a ThingSpeak')

def main():
    rospy.init_node('thingspeak_publisher')
    rospy.Subscriber('battery_state', CustomBatteryState, battery_state_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
