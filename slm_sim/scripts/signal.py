#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

if __name__=='__main__':
    #Initialize and Setup node
    rospy.init_node("Sig_generator")
    print("Signal Generator is Running")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #Setup de publishers
    sig_pub = rospy.Publisher("tau", Float32 , queue_size=10)

    try:

        previous_time = rospy.get_time()
        signal = 1.0


        while not rospy.is_shutdown():

            if (rospy.get_time()-previous_time) >= 10:
                signal = 1.0
                previous_time = rospy.get_time() 
            else: 
                signal = 0

            # Publish new angle
            sig_pub.publish(signal)

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialize and Setup node