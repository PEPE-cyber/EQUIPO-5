#!/usr/bin/env python
import rospy
from pid_control.msg import motor_output
from pid_control.msg import motor_input 
from pid_control.msg import set_point
from std_msgs.msg import Float32

#Setup parameters, vriables and callback functions here (if required)
class PIDController:
    def __init__(self):
        # get parameters from parameter server
        self.kp = rospy.get_param("/kp")
        self.ki = rospy.get_param("/ki")
        self.kd = rospy.get_param("/kd")
        print("kp: " + str(self.kp) + " ki: " + str(self.ki) + " kd: " + str(self.kd))
        # initialise variables
        self.setpoint = 0.0
        self.error = 0.0
        self.error_sum = 0.0
        self.error_diff = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0
        self.controllerOutput = motor_input()
        # setup publishers and subscribers
        self.motorInputPub = rospy.Publisher("/motor_input", motor_input, queue_size=10)
        self.motorOutputSub = rospy.Subscriber("/motor_output", motor_output, self.updateFromMotorOutput, queue_size=10)
        self.setpointSub = rospy.Subscriber("/set_point", set_point, self.updateSetpoint, queue_size=10)
        print("Controller Initialised")

    
    def updateFromMotorOutput(self, output):
        # Check if this is the first time we have received a message
        if self.prev_time != 0:
            # Calculate the dt, error, error_sum (integral) and error_diff (derivative)
            # Dt is the time difference between the current and previous 
            dt = output.time - self.prev_time
            # Save the current time for the next iteration
            self.prev_time = output.time
            # Calculate the error difference between the setpoint and the real output of the motor
            self.error = self.setpoint - output.output 
            # Calculate the integral of the error (error_sum) the integral is the sum  of the area of the trapeziums formed by the error and the time difference
            self.error_sum += (self.error + self.prev_error) * dt / 2
            # Calculate the derivative of the error (error_diff) the derivative is the slope of the line formed by the error and the time difference
            self.error_diff = (self.error - self.prev_error) / dt
            # Store the current error for the next iteration
            self.prev_error = self.error
        else:
            # This is the first time we have received a message so we can't calculate the error_diff or error_sum
            self.prev_time = output.time
        #print("setpoint", self.setpoint, "MotorOutput",  output.output, "Error", self.error, "MotorInput", self.controllerOutput.input)

    def get_output(self):
        # Calculate the output of the controller
        self.controllerOutput.input = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.controllerOutput.time = rospy.get_time()
        return self.controllerOutput
    
    def run(self):
        # Publish the output of the controller
        self.motorInputPub.publish(controller.get_output())
  
    def stop(self):
        # Stop the controller
        print("Stopping")

    # Callback function to update the setpoint from the setpoint topic
    def updateSetpoint(self, setpoint):
        # Update the setpoint
        self.setpoint = setpoint.value
        #print("Setpoint Updated to: " + str(self.setpoint) + " at time: " + str(setpoint.time) )

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("controller")
    # Set the rate of the node (For the publisher rate)
    rate = rospy.Rate(100)
    # Create an instance of the controller class
    controller = PIDController()
    rospy.on_shutdown(controller.stop)
    # Create a publisher for the error 
    errorPub = rospy.Publisher("/error", Float32, queue_size=10)
    #Setup Publishers and subscribers here
    
    
                          
    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():
        # Run the controller (get the output and publish it)
        controller.run()
        # Publish the error
        errorPub.publish(controller.error)
        # Sleep to maintain the rate
        rate.sleep()

