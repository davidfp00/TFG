#!/usr/bin/python3

import sys
import asyncio
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState

from smartbms.smartbms import BMS

class ComInstance:
    def __init__(self, port, key, bms):
        self.port = port
        self.key = key
        self.bms = bms

async def program():
    instances = []
    loop = asyncio.get_running_loop()
    print('123\\SmartBMS to ROS\r\n')
    
    port = '/dev/ttyUSB0'  # Puerto a utilizar (puedes modificarlo según tus necesidades)
    
    bms = BMS(loop, port)
    await bms.connect()
    com = ComInstance(port, None, bms)
    instances.append(com)
    
    # sleep for 10 seconds so we have time to receive BMS data and for internet connection to start
    await asyncio.sleep(10)
    
    # Inicializea nodo ROS y publicador
    rospy.init_node('smartbms')
    pub = rospy.Publisher('battery_state', BatteryState, queue_size=10)
    
    while True:
        try:
            # Datos publicados a ROS.
            for instance in instances:
                battery_voltage = instance.bms.pack_voltage
                battery_current = instance.bms.pack_current
                soc = instance.bms.soc
                lowest_cell_voltage = instance.bms.lowest_cell_voltage
                highest_cell_voltage = instance.bms.highest_cell_voltage
                allowed_to_charge = instance.bms.allowed_to_charge
                allowed_to_discharge = instance.bms.allowed_to_discharge
                cell_communication_error = instance.bms.cell_communication_error or instance.bms.serial_communication_error

                # Creamos mensaje BatteryState 
                battery_state_msg = BatteryState()
                battery_state_msg.voltage = battery_voltage
                battery_state_msg.current = battery_current
                battery_state_msg.percentage = soc
                
                # Publicamos el topic battery state
                pub.publish(battery_state_msg)
                
                # Imprimimos los valores a través de la terminal de Linux
                print('Battery State: voltage={}, current={}, soc={}, lowest_cell_voltage={}, highest_cell_voltage={}, allowed_to_charge={}, allowed_to_discharge={}, cell_communication_error={}'.format(
                    battery_voltage,
                    battery_current,
                    soc,
                    lowest_cell_voltage,
                    highest_cell_voltage,
                    allowed_to_charge,
                    allowed_to_discharge,
                    cell_communication_error
                ))
            
            print('Published battery state to ROS topic "battery_state"')
        
        except Exception as e:
            print('Error publishing battery state to ROS:', e)

        await asyncio.sleep(60)

def main():
    asyncio.run(program())

if __name__ == "__main__":
    main()
