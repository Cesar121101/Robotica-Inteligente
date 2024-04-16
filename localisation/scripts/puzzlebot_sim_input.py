#!/usr/bin/env python
import rospy


if __name__ == '__main__':
    # Initialize the node 'input' at 100 Hz
    rospy.init_node("puzzlebot_sim_input")
    rate = rospy.Rate(100)

    print("-- User input --")
    vel = 0.0
    selection = False

    while not rospy.is_shutdown():
        if selection == False:
            # rospy.get_param("/linear_velocity", int())
            # rospy.get_param("/angular_velocity", int())

            lin_vel = input("\nVelocity: ")
            ang_vel = input("\nAngular Velocity: ")

            rospy.set_param("/linear_velocity", lin_vel)
            rospy.set_param("/angular_velocity", ang_vel)

            selection = True
            
            print("\nValues Established.")
            
            

        # When velocity has been defined
        # else:
        #     # Validate the value of the parameters
        #     print("User distance: %s", rospy.get_param("/user_dist", "No param found"))
        #     print("User time: %s", rospy.get_param("/user_time", "No param found"))

        #     # Ask the user if he/she wants to provide another selection
        #     print("\n\nHave you finish setting the parameters? 0 for No, 1 for Yes")
        #     another = input("Your choice: ")

        #     # If the user wants to provide another selection
        #     if another == 0:
        #         selection = False   # Go to the beginning

        #     # If the user does not want to provide another selection
        #     elif another == 1:
        #         rospy.set_param("/user_finish", 1.0) # Set the user finish
        #         break   # Finish program
        rate.sleep()