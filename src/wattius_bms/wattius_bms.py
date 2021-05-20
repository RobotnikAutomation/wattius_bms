import threading
import rospy
from pymodbus.client.sync import ModbusTcpClient


from rcomponent.rcomponent import RComponent
from robotnik_msgs.msg import BatteryStatus

BATTERY_STATE_UNKNOWN           = 'unknown'    
#BATTERY_STATE_START_CHARGING    = 'start_charging'    
BATTERY_STATE_CHARGING          = 'charging'    
#BATTERY_STATE_STOP_CHARGING     = 'stop_charging'    
BATTERY_STATE_DISCHARGING       = 'discharging'    

class WattiusBMS(RComponent):
    def __init__(self):
        RComponent.__init__(self)

        #self._driver = DalyBMSDriver()
        self._battery_status = BatteryStatus()
        self._last_battery_state = BATTERY_STATE_UNKNOWN
        self._time_init_charging = rospy.Time.now()
        self._last_discharge_value = 3.0
        
    
    def ros_read_params(self):
        RComponent.ros_read_params(self)
        
        self._address = rospy.get_param('~ip_address', 'localhost') 
        self._port = rospy.get_param('~port', 502)
        
        self._soc_input_register = rospy.get_param('~soc_input_register', 1000)
        self._voltage_input_register = rospy.get_param('~voltage_input_register', 1001)
        self._current_input_register = rospy.get_param('~current_input_register', 1002)
    
    def ros_setup(self):
        self._battery_status_pub = rospy.Publisher("~status", BatteryStatus, queue_size=10)
        self._reading_timer = threading.Timer(2.0, self.read)
        self._reading_timer.start()
        
        RComponent.ros_setup(self)
    
    def setup(self):
        self._client = ModbusTcpClient(self._address, port=self._port)

        RComponent.setup(self)
    
    def shutdown(self):
        self._reading_timer.cancel()

        RComponent.shutdown(self)

    def ros_shutdown(self):
        self._battery_status_pub.unregister()

        RComponent.ros_shutdown(self)
        
    def read(self):
        
        # READ SOC
        self._client.connect()

        soc = self._client.read_input_registers(self._soc_input_register)
        soc = soc.registers[0] / 100.0
        self._battery_status.level = soc

        self._client.close()


        # READ VOLTAGE
        self._client.connect()

        voltage = self._client.read_input_registers(self._voltage_input_register)
        voltage = voltage.registers[0] / 10
        self._battery_status.voltage = voltage

        self._client.close()

        # READ CURRENT
        self._client.connect()

        current = self._client.read_input_registers(self._current_input_register)
        current = current.registers[0] / 10
        self._battery_status.current = current

        self._client.close()

        # Charging
        # TODO: Apply hysteresis
        if current > 0.0:
            if self._last_battery_state == BATTERY_STATE_UNKNOWN or self._last_battery_state == BATTERY_STATE_DISCHARGING:
                self._time_init_charging = rospy.Time.now().secs

            self._last_battery_state = BATTERY_STATE_CHARGING
            self._battery_status.is_charging = True

            elapsed_time = (rospy.Time.now().secs - self._time_init_charging)/60
            elapsed_time = int(elapsed_time)

            self._battery_status.time_charging = elapsed_time

        elif current <= 0.0:
            self._last_battery_state = BATTERY_STATE_DISCHARGING


            self._battery_status.is_charging = False
            self._battery_status.time_charging = 0
            self._last_discharge_value = self._battery_status.current

        #print voltage.__dict__

        '''
        self._battery_status.voltage = data['total_voltage']
        self._battery_status.current = data['current']

        data = self._driver.get_mosfet_status()
        
        if data['mode'] == 'discharging':
            self._battery_status.is_charging = False
            self._battery_status.time_charging = 0
            self._last_discharge_value = self._battery_status.current
        
        elif data['mode'] == 'charging':
        
            if self._last_battery_state == 'Unknown' or self._last_battery_state == 'discharging':
                self._time_init_charging = rospy.Time.now().secs


            self._battery_status.is_charging = True
            elapsed_time = (rospy.Time.now().secs - self._time_init_charging)/60
            elapsed_time = int(elapsed_time)

            self._battery_status.time_charging = elapsed_time

        remaining_hours = round(data['capacity_ah']/self._last_discharge_value, 0)
        self._battery_status.time_remaining = int(remaining_hours)*60
        self._last_battery_state = data['mode']
        '''
        self._reading_timer = threading.Timer(2.0, self.read)
        self._reading_timer.start()
        
    
    def ros_publish(self):
        self._battery_status_pub.publish(self._battery_status)



