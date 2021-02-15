#!/usr/bin/env python

#STUDENT NAME: BORA ONER  
#STUDENT ID:   150170301

####################### READ ME ########################
#   
#   The turtlerobot is designed to follow waypoints that are predefined points. In this robot deisng, the implementation of the robot controller unit is as follows:
#   
#       The robot has a short term memory that stores the last most near point that it has captured, this was needed because as the robot was approaching to the target point 
#       a new data was starting to be retrieved from waypoint rosnode so the path of the robot was changing drastically. To prevent that a short term memory is used. 
#
#       Working Mechanism of the Robot:
#           
#          1- Calculate the distance between the current position and the next waypoint posisition
#          2- Calculate the angle of the line between the points by using tangent funciton 
#          3- Rotate the Robot's orientation acording to the the theta value of the tangent so that Robot has facing the Waypoint coorindates
#          4- Move towards that direction
#          5- While moving to the point based on the distance propotions(%60); rotate towards slightly to fit the waypoint's orientation
#          6- Robot checks if it is landed on the point with an error of 0.1 on x and y coordinates
#          7- Afterwards stops at the point
#          8- This Algorithm continues in a loop fashion
#
#            
#
#
#
########################################################

import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import math #atan2--> arc tangent funciton for slope calculation, #sqrt--> used inside the distance calculation


##short term memory to hold the previous waypoint components
way_x = None
way_y = None
way_t = None
way_current_ori = None #holds the previous waypoint_theta --> Used for fit the orientation of the point after reaching to it 
linear_distance = None #holds the current distance between point 


waypoint=None

def change_next_waypoint(x_cord,y_cord,theta):  #updates the global coordinate variable
    global way_x
    way_x = x_cord
    global way_y
    way_y = y_cord
    global way_current_ori
    global way_t
    
    way_current_ori = way_t
    way_t = theta

def ang_diff_calculate(robot_theta,way_current_ori): ##determines the angle value to rotate the robot
    
    ang_diff = 0 

    if robot_theta * way_current_ori >0:    ##both angles have the same 
        ang_diff = abs(robot_theta) - abs(way_current_ori)            
        
        if robot_theta >0: # + + case
        
            ang_diff = -ang_diff

        else:   # - - case
            ang_diff = ang_diff            

    else:
        if robot_theta >= 0:    ##if robot_theta has + angle value and way_current_ori has negative angle
    
            if robot_theta < 1.5 and way_current_ori > -1.5 :  ##robot_theta and target angle is on the 1st quadrant
                ang_diff = way_current_ori - robot_theta
    
            elif robot_theta < 1.5 and waypoint < -1.5:        ##robot_theta is on the 2nd and target angle is on the 3rd quadrant
                
                ang_diff = way_current_ori - robot_theta
                    
            elif robot_theta > 1.5 and way_current_ori > -1.5: ##robot_theta is on the 2nd and target angle is on the 4th quadrant
    
                ang_diff = robot_theta - way_current_ori  
                                
            else:                                              ##robot_theta is on the 2nd and target angle is on the 3rd quadrant
                
                ang_diff = 2*math.pi - (robot_theta - way_current_ori) 
                
        else: #if way_currrent_ori is positive side 
            
            if way_current_ori < 1.5 and robot_theta > -1.5:    ##robot_theta is on the 4th and target angle is on the 2nd quadrant
                
                ang_diff = way_current_ori - robot_theta
                
            elif way_current_ori < 1.5 and robot_theta < -1.5:  ##robot_theta is on the 2nd and target angle is on the 3rd quadrant
                
                ang_diff = way_current_ori - robot_theta
            
            elif way_current_ori > 1.5 and robot_theta > -1.5:  ##robot_theta is on the 4th and target angle is on the 2nd quadrant

                ang_diff = way_current_ori - robot_theta
                
            else:                                               ##robot_theta is on the 2nd and target angle is on the 3rd quadrant
                
                ang_diff = 2*math.pi - (way_current_ori - robot_theta) #- le carpmayi unutma
                
                ang_diff = -ang_diff
                
            
    return ang_diff


def waypoint_callback(msg): #  callback

    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(3); # rate is set to 3 this will make robot receive more frequent data
    danger_flag = False
    change_way = True ##if sub_vec not updated and it is in inf loop saves the robot
    flag = True
    waypoint_flag = False    
    Rotate_flag = False
    global way_x
    way_x = None
    global way_y
    way_y = None
    global way_t
    way_t = None            
    
    global linear_distance
    linear_distance = 0
    
    while not rospy.is_shutdown():
        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************
        print("\n")
        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")


        motor_command=Twist()

        #I don't know what to do because nobody has programmed me with any smartness,
        #so I'll do what everybody does, and drive very fast straight forwards.
        
        if change_way == True:
            global way_x
            way_x = waypoint.translation.x
            global way_y
            way_y = waypoint.translation.y
            global way_t
            way_t = waypoint_theta            
            global way_current_ori
            way_current_ori = waypoint_theta
            change_way = False
            
            linear_distance = math.sqrt((translation[1] - way_y)**2 + (translation[0] - way_x)**2) ##inital distance
            
        err_lin_x = 0.08 # x axis error
        err_lin_y = 0.08 # y axis error
        ang_err = 0.1    #general agnle error is set

        if flag:
            #if the point is reached the loop goes into the if statement
            if (way_x - err_lin_x <= translation[0] and translation[0] <= way_x+ err_lin_x) and (way_y - err_lin_y <= translation[1] and translation[1] <= way_y + err_lin_y):
                print("I have arrived to the waypoint")
                                    
                if danger_flag == False:
                    change_next_waypoint(waypoint.translation.x,waypoint.translation.y,waypoint_theta) #change the new waypoints
                    linear_distance = math.sqrt((translation[1] - way_y)**2 + (translation[0] - way_x)**2) ##the whole distance between two waypoint is calculated

                    flag = False #toggle the flag
                    Rotate_flag = False
                    continue

            linear_distance_cal = math.sqrt((translation[1] - way_y)**2 + (translation[0] - way_x)**2) ##distance between robot and the waypoint is calculted
            
            #vector substruction for obtaining the line between points
            subvec_x = way_x - translation[0]
            subvec_y = way_y - translation[1]   
            sub_vec_theta = math.atan2(subvec_y,subvec_x)           
            
            w = ang_diff_calculate(robot_theta, sub_vec_theta) ##the angle amount to rotate with its direction is returned
            
            if  abs(w) < 0.1: #the angular error is near to 0.1 go linear
                print("I am moving towards the current waypoint")
                motor_command.linear.x = 0.2
                
                if linear_distance_cal <= linear_distance*0.5 or linear_distance <0.1: #if robot getting closer to point rotate towards the waypoint orientation
                
                    ang_diff = ang_diff_calculate(robot_theta,way_t)
                    motor_command.angular.z = ang_diff*0.9 
                    print("I am getting closer to the arrival point I am setting my orientation ")
                    print("The distance between me and the point = " + str(linear_distance_cal) + "I am rotating with "+str(ang_diff) + "angle value")
            else:
                motor_command.angular.z = w
                print("I am rotating to face the destination point")
                print("Rotating with = " + str(w) + "angle value")

        else:
            
            if danger_flag==True:
                change_next_waypoint(waypoint.translation.x,waypoint.translation.y,waypoint_theta)
                danger_flag = False
            
            if way_current_ori - ang_err <= robot_theta and way_current_ori + ang_err >= robot_theta:
                
                print("I have set my orientation, I am good to move towards the waypoint")
                flag = True ##Complete the rotation
            else:
                print("Rotating to satisfy the waypoing orientataion")
                w = ang_diff_calculate(robot_theta,way_current_ori)
                motor_command.angular.z= w
                print("Rotating with = " + str(w) + "angle value")
    
    
    
        motor_command_publisher.publish(motor_command)

        #######################################################################
        #FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME
        #FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME
        #FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME
        #######################################################################

        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
