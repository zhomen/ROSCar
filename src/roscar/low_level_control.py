#!/usr/bin/python3

import rospy, time
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist

class ServoConvert():
    def __init__(self, id=1, center_value=333, range=90, direction=1):
        self.value          = 0.0
        self.center_value   = center_value
        self._center        = center_value
        self._range         = range
        self._half_range    = 0.5*range
        self._dir           = direction
        self.id             = id
        
        #--- Convert its range in [-1, 1]
        self._sf            = 1.0/self._half_range

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value          = value_in
        self.value_out      = int(self._dir*value_in*self._half_range
                + self._center)
        print(self.id, self.value_out)
        return(self.value_out)

class RCLowLevelCtrl():
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")
        rospy.init_node('roscar')

        self.actuators = {}
        self.actuators['throttle']  = ServoConvert(id=5)
        self.actuators['steering']  = ServoConvert(id=6, direction=1) 
        #--- positive left
        rospy.loginfo("> Actuators correctly initialized")

        self._servo_msg             = ServoArray()
        for i in range(2): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/sercos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher correctly initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist          = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo("> Subscriber correctly initialized")

        #--- Get the last time a command was received
        self._last_time_cmd_rcv     = time.time()
        self._timeout_s             = 5

        rospy.loginfo("Intialization complete")

    def set_actuators_from_cmdvel(self, message):
        #--- Get a message from cmd_vel, assuming max of 1

        #--- Save the time
        self._last_time_cmd_rcv = time.time()

        #--- Convert vel into servo values
        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        rospy.loginfo("Got a command v = %2.1f s = %2.1f"%(message.linear.x, message.angular.z))
        self.send_servo_msg()

    def set_actuators_idle(self):
        #--- Convert vel into serco values
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        rospy.loginfo("setting actuators to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        for x,(key,servo_obj) in enumerate(self.actuators.items()):
            print("Servo Object ID -> " + str(servo_obj.id))
            print(self._servo_msg.servos)
            self._servo_msg.servos[x].servo = servo_obj.id
            self._servo_msg.servos[x].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(key, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        print(time.time() - self._last_time_cmd_rcv)
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):
        #--- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print(self._last_time_cmd_rcv, self.is_controller_connected)
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

if __name__ == "__main__":
    roscar  = RCLowLevelCtrl()
    roscar.run()
