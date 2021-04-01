#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from differential_drive_arduino.msg import motors_speed

#                        left motor sub node 
global left_motor_vel

def lmotor_callback(data1):
    left_motor_vel= data1.data

    
def lm_listener():

    rospy.init_node('lm_listener', anonymous=False)

    rospy.Subscriber("lwheel_vtarget", Float32, lmotor_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
###################################################################################

#                        right motor sub node 
global right_motor_vel

def rmotor_callback(data2):
    right_motor_vel= data2.data

    
def rm_listener():

    rospy.init_node('rm_listener', anonymous=False)

    rospy.Subscriber("rwheel_vtarget", Float32, rmotor_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

###################################################################################

def motors_pub():
    pub = rospy.Publisher('motors_vel', motors_speed, queue_size=10)
    rospy.init_node('two_motors_pub', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    motors_pub=motors_speed()
    while not rospy.is_shutdown():
        
        motors_pub.Motor_A = left_motor_vel
        motors_pub.Motor_B = right_motor_vel
            
        pub.publish(motors_pub)
        rospy.loginfo("sending Two motors speed on -/motors_vel- topic")
          
    rate.sleep()
    
##########################################################################################



if __name__ == '__main__':
    try:
        lm_listener()
        rm_listener()
        motors_pub()
        
    except rospy.ROSInterruptException:
        pass
