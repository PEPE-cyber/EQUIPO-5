#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

# Setup Variables, parameters and messages to be used (if required)


#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

class SetPointGenerator:
    def __init__(self):
        #Setup parameters
        self.freq = rospy.get_param("/freq")
        self.amp = rospy.get_param("/amp")
        self.signal = rospy.get_param("/signal_type")
        
        #Setup msg variable
        self.setpoint = set_point()
        self.prevTime = 0
        self.setpoint.value = 0

        print("Set Point Generator Initialised")

    def get_output(self):
        # Set the time of the setpoint
        self.setpoint.time =  rospy.get_time()
        # Generate the setpoint depending on the signal type
        if self.signal == "sine": 
            self.setpoint.value = self.amp * (np.sin(2 * np.pi * self.freq * rospy.get_time())) 
        elif self.signal == "square": 
            if rospy.get_time() - self.prevTime > ( 1 / self.freq):
                if self.setpoint.value > 0:
                    self.setpoint.value = - self.amp
                else:
                    self.setpoint.value =  self.amp
                self.prevTime = rospy.get_time()
        else: 
            if rospy.get_time() - self.prevTime > ( 1 / self.freq):
                self.setpoint.value = np.random.randint(self.amp * 2) - self.amp
                self.prevTime = rospy.get_time()
        return self.setpoint

    def stop(self):
        print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    print(set_point)
    rospy.init_node("Set_Point_Generator")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    pub = rospy.Publisher("/set_point", set_point, queue_size=1)
    setpoint = SetPointGenerator()

    print("The Set Point Genertor is Running")

    #Run the node
    while not rospy.is_shutdown():
        #Generate the set point here
        pub.publish(setpoint.get_output())
        #Write your code here
        rate.sleep()