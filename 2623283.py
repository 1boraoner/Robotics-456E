#!/usr/bin/env python

#STUDENT NAME: BORA ONER  
#STUDENT ID:   150170301

############################ READ ME#################################
#
#   For this assignment the robot's movement module is designed based on the laser scan data that laser sensor on tutrlebot.
#   In my design the robot has states which are called "MOVE" "STOP" and "TURN". In the MOVE state the robot's movement commands are 
#   decided based on the current distances coming from the laser scan. STOP state is a bridge for TURN and MOVE states interchange. In the STOP 
#   vaious flags are change so that the other state's actions change. Lastly, TURN state is a positional module where if the robot is stuck a certain point it help
#   the robot to turn with certain degree values. For instance; with TURN state ROBOT can TURN 90, 180 and 360 degrees around itself.
#
#   For this assignment, 4 extra functions are implamented.
#   data_cleaner => clenas the signal data from NaN values.
#   arr_dir => divides the laser data into 5 different sub arrays and stores their averages as an array
#   change_state => change states based on the current_state and the next_state
#   change_angle => returns the best optimal direction based on the given array of sub arrays.
#
#   For the controls;
#
#    ROBOT starts with the 360 turn and finds the longest path it sees from that point and changes its direction to there
#    
#    After this operation ROBOT is entirely free to move and controlled with flags, and if else conditions based on the current distance of the
#    front face of the turtlebot.
#    
#    If ROBOT is too close with the edge values arr[0] andarr[4] based on their differnce ROBOT turns RIGHT or LEFT in 90 degrees
#    IF ROBOT is in a position where its edge values are too close to itself but the front is in medium distance it moves 180 degrees to be safe 
#    IF ROBOT is TURNed 90 degrees infor a certain number the ROBOT might be in a closed loop(a room) and cant find the exit, SO, it turns to find the 
#    longest path to be free from the loop
#    
#   MOVE operetaion is an autonomous mode so robot is deciding to when to rotate, when to change modes mainly based on the distance between its front side
#   values. In the code many intervals of the distance are defined and I believe, all the possible intervals are covered so that, robot can fully function.
#
#
#    Also, ROBOTS aim is to explore the area where it is in so, with the rotations robot's mapping system is also constructing the map of te area.
#
#############################################################################



import rospy
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


def data_cleaner(arr): ##returns a cleaned data 
    
#    sum1 = map(lambda x: 0.001 if x!=x else x, arr)
#    av = sum(sum1) / len(sum1)
#       
    narr = []
    flag_1 = False
    for i in range(len(arr)):
        
        if arr[i] != arr[i]:
            if i == 0:
                flag_1 = True
            
            if flag_1:
                narr.append(0)
            else:
                narr.append(narr[i-2]) #To keep the average value in a certain point if NaN
            
        else:
            narr.append(arr[i])
    return narr

def arr_dir(arr):   ##Divides array 5 sub array and thakes averages of each sub array to store
    
    sub_arr = [0,0,0,0,0]
    for i in range(5):
        for j in range(len(arr)/5):
            sub_arr[i] = sub_arr[i] + arr[(len(arr)/5)*i:(len(arr)/5)*(i+1)][j]
        sub_arr[i] /= len(arr)/5
    return sub_arr
            
    
def change_state(current,n_state): #changes states based on the conditions
    
    if current == "STOP":
        if n_state == "TURN":        
            return "TURN"
        elif n_state == "MOVE":
            return "MOVE"
        else:
            return "TURN"
            
    elif current == "TURN":
        if n_state =="MOVE":
            return "MOVE"
        else:
            return "STOP"
    elif current == "MOVE":
        if n_state=="TURN":
            return "TURN"
        else:            
            return "STOP"
    
    
    
def change_angle(arr): #verilen sub array tutan arrayleri max olanini returnler
    
    maxes = [max(sub_arr) for sub_arr in arr]        
    way_to_go = maxes.index(max(maxes))  ##maxes.index(max(maxes)) ---> ince ayarli olanda kullanicam
    return way_to_go

def laser_callback(data, args):
    
    motor_command = Twist() #Message object 
    
    max_arr = args #array that holds the max sub arrays of the each partition of the 360 rotation
    
    global lnum  #laser number which used as counter for the rotation operation
    global state #holds the current state of the robot
    global prev_state #holds the previous state of the robot
    global flag #flag idctating the ROTATION operation type 0->360 1->180 2->90 
    global lstm #holds the number of 90 degree rotations


    if state == "STOP": #body of the STOP state
        
        motor_command.angular.z = 0
        motor_command.linear.x = 0 #DONT MOVE
        
        if prev_state == "TURN":
            prev_state = state
            state = change_state(state,"TURN")  
            lnum = 0              
        if prev_state == "MOVE":
            
            if flag == 0 : #360 degree rotation
                prev_state ="START"
            if flag == 1:   #180 degree rotation
                prev_state ="MOVE"
            if flag == 2:   #90 degree rotation
                prev_state ="EDGE"
                
            state = change_state(state,"TURN")
            del max_arr[:] #clean the array so that for the next rotation fresh array is appended with max sub-arrays
            lnum = -1 #decrease the lnum to -1 becasue in the rotation lnum = 0 is important
                        
    elif state == "MOVE":
        
        list_arr = data_cleaner(data.ranges)    ##cleaned data      
        arr = arr_dir(list_arr)                 #averages of the sub_arrays
        if arr[2] !=0: #if the strong front is not zero

            if 3<arr[2] <6: #if the distance between 3 and 6 meters
                
                if 2 < arr[1] < 3.5 or 2 < arr[3] < 3.5: ##checking if there is a difference between the sides so move a little bit to there
                    difference = arr[1] - arr[3]
                    angl = 0
                    if abs(difference) < 0.5:
                        angl = 0.1 
                    elif abs(difference) < 1:
                        angl = 0.3 
                    elif abs(difference) < 1.5:
                        angl = 0.44 
                    else:
                        angl = 0.6 
                        
                    if difference < 0: #check the sign of the varible
                        motor_command.angular.z = angl #rotate to right side
                    else:
                        motor_command.angular.z = -angl #rotate to left side
                    motor_command.linear.x = 0.8 

                if arr[4] < 0.8 or arr[0] < 0.8: #if a sharpe edge is observed rotate acordingly
                    
                    if arr[4] == min(arr):
                        motor_command.angular.z = -1.5
                    elif arr[0] == min(arr):
                        motor_command.angular.z = 1.5
                    motor_command.linear.x = 0.6
                
                #first part of the if statement is the COSINE THEOREM for finding the length of the DOOR observed
                if (arr[4]**2+arr[0]**2 -(2*arr[0]*arr[4]*0.86602))**0.5 < 1.3 and arr[1] > 1 and arr[2] > 1.5 and arr[3] > 1 and arr[4]!=0 and arr[0]!=0: 

                    if arr[4] > arr[0]:
                        motor_command.angular.z = 0.7 #oppsing abgle is given to robot to get the edge smootly
                        motor_command.linear.x = 0.6

                    else:
                        motor_command.linear.x = 0.6
                        motor_command.angular.z = -0.7 #ters aci verildi
                                           
                else:
                    
                    motor_command.linear.x= 0.8
                        
            elif 2 <arr[2] < 3: #if front distance is between 2meters and 3 meters
                
                if 0.75 < arr[1] < 1.5 or 0.75< arr[3] < 1.5: #if the sides between front and edges are between 0.75 and 1.5 check difference
                    difference = arr[1] - arr[3]
                    angl = 0
                    if abs(difference) < 0.5:
                        angl = 0.2
                    elif abs(difference) < 1:
                        angl = 0.3
                    elif abs(difference) < 1.5:
                        angl = 0.45
                    else:
                        angl = 0.55
                        
                    if difference < 0:
                        motor_command.angular.z = angl
                    else:
                        motor_command.angular.z = -angl
                    
                    motor_command.linear.x = 0.6
                else:     
                    motor_command.linear.x = 0.6

           
            elif 1.25 < arr[2] < 2: # checking front distance between 1.25 and 2 meters
                
                if 0.43 <arr[4] < 0.6 and 0.43 <arr[0]< 0.6: #turn 180
                    prev_state = state
                    state = change_state(state,"STOP")
                    lnum = 0
                else:
                    if arr[0] + arr[1] < arr[3] + arr[4]: #checking which side is more dominant so that move there
                        if (arr[1] - arr[3]) < 0 and 0.7 < arr[1] < 1.5:
                            motor_command.angular.z = 2.2
 
                        elif (arr[3] - arr[1]) < 0  and 0.7 < arr[3] < 1.5:
                             motor_command.angular.z = -2.2
                        motor_command.linear.x = 0.6                
                    else:
                        motor_command.linear.x = 0.7 #0.7
           
            elif 0.8< arr[2] < 1.25: #distance between 0.8 and 1.25 meters
                if arr[4] == 0 and arr[0]<2:
                    motor_command.angular.z = -1.8
                elif arr[0] == 0 and arr[4] < 2:
                    motor_command.angular.z = 1.8
                motor_command.linear.x = 0.6
                
            elif 0.5 < arr[2] < 0.8: #checking for very small values this means very close to something
                if 0.4 <arr[4] < 0.5 and 0.4 <arr[0]< 0.5 and abs(arr[0] - arr[4]) < 0.2: #turn 180
                    prev_state = state
                    state = change_state(state,"STOP")
                    lnum = 0
                    flag = 1
                elif abs(arr[4] - arr[0]) < 0.5: #90 turn
                    if lstm == 4:
                        prev_state = state
                        state = change_state(state,"STOP")
                        lnum =0
                        flag = 0
                        lstm = 0 # CLEAN THE COUNTER 90 DEGREE                        
                    else:                        
                        prev_state = state
                        state = change_state(state,"STOP")
                        lnum =0
                        flag = 2
                        lstm += 1
                    
                else:# general else body, check which is max value so change direction to theat part
                     if arr[2]!= max(arr):
                        if arr[1] == max(arr):
                            motor_command.angular.z = -1.7
                            motor_command.linear.x = 0.2
                        elif arr[0] == max(arr):
                            motor_command.angular.z = -2.5
                            motor_command.linear.x = 0.35
                        elif arr[3] == max(arr):
                            motor_command.linear.x = 0.2
                            motor_command.angular.z = 1.7
                        elif arr[4] == max(arr):
                            motor_command.angular.z = 2.4
                            motor_command.linear.x = 0.3                   
            elif arr[2]<0.5: #checking for the smallest distance and avoiding possible collusions
                
                if arr[3] < 0.5 and arr[4] < 0.6 or arr[1] < 0.5 and arr[0] < 0.6: #rotate 90 degrees
                    prev_state = state
                    state = change_state(state,"STOP")
                    lnum =0
                    flag = 2
                elif arr[3] < 0.5 or arr[1] < 0.5: #rotate 360 degrees
                    prev_state = state
                    state = change_state(state,"STOP")
                    lnum =0
                    flag = 0
                else: # general else body, check which is max value so change direction to theat part
                    if arr[2]!= max(arr):
                        if arr[1] == max(arr):
                            motor_command.angular.z = -1.7
                            motor_command.linear.x = 0.2
                        elif arr[0] == max(arr):
                            motor_command.angular.z = -2.5
                            motor_command.linear.x = 0.35
                        elif arr[3] == max(arr):
                            motor_command.linear.x = 0.2
                            motor_command.angular.z = 1.7
                        elif arr[4] == max(arr):
                            motor_command.angular.z = 2.4
                            motor_command.linear.x = 0.3 
                        
                
                
        else:  
            if arr[1] == 0 or arr[3] == 0:
                    
                if arr[1] == 0:        
                    motor_command.angular.z = -0.3                     
        
                elif arr[3] == 0:
                    motor_command.angular.z = 0.3
                motor_command.linear.x = 1
                    
            elif (arr[0] == 0 and arr[1] != 0) or (arr[4] == 0 and arr[3] != 0):
                    
                if arr[3] > arr[1]:
                    motor_command.angular.z  = 0.15
                else:
                    motor_command.angular.z = -0.15
                    motor_command.linear.x = 0.5                    
                    
            else:
                if arr[1] == max(arr):
                    motor_command.angular.z = -0.5
                    motor_command.linear.x = 0.4
                elif arr[0] == max(arr):
                    motor_command.angular.z = -0.8
                    motor_command.linear.x = 0.6
                if arr[3] == max(arr):
                    motor_command.linear.x = 0.5
                    motor_command.angular.z = 0.4
                elif arr[4] == max(arr):
                    motor_command.angular.z = 0.8
                    motor_command.linear.x = 0.6
        
        if all(direction is 0 for direction in arr):
                motor_command.linear.x = 0.5
                motor_command.angular.z = -0.05
                    
                    
    elif state == "TURN": #rotation state

        if prev_state == "START":  #START prev_state is a special sub routine different than others because ROBOT decides the longest path to go 36
            
            
            if lnum%10 == 0 and lnum< 80: #8 points so 8*10
                
                list_arr = data_cleaner(data.ranges)        
                arr = arr_dir(list_arr)
                max_arr.append(arr)
                best_way = change_angle(max_arr)
                
                
            if lnum !=0 and (lnum%80 ==0 or lnum >80): #lnum 80 satisfied so a one turn is complete
                
                prev_state= state
                state = change_state(state,"STOP") 
                lnum = 0 #reset laser_number
                motor_command.angular.z = 0
            else:
                motor_command.angular.z = 0.52 
                
        elif prev_state== "STOP": #back rotation after finding the best way(longest path)
           
            best_way = change_angle(max_arr)
            
            if best_way < 5:
                if  lnum> (best_way+1)*10:
                    motor_command.angular.z = 0
                    
                    prev_state= state
                    state = change_state(state,"MOVE") 
                    lnum = 0 
                else:    
                    motor_command.angular.z = 0.52
            elif best_way >=5:
                if  lnum> (len(max_arr)-best_way)*10:
                    motor_command.angular.z = 0
                    prev_state= state
                    state = change_state(state,"MOVE")
                    lnum = 0
                    
                else:        
                    motor_command.angular.z = -0.50
            
        elif prev_state == "MOVE": ##180 DEGREES ROTATION
            
            if lnum != 0 and (lnum%50 == 0 or lnum > 50):
                
                prev_state = state
                state = change_state(state,"MOVE")
                
            else:
                motor_command.angular.z = 0.50
        elif prev_state=="EDGE":
            list_arr = data_cleaner(data.ranges)        
            arr = arr_dir(list_arr)
            
            if lnum == 0:    
                if arr[0] < arr[4]:
                    flag = 4 #LEFT
                
            if lnum != 0 and (lnum%20 == 0 or lnum > 20):
                
                prev_state = state
                state = change_state(state,"MOVE")
                
            else:
                 
                if flag==4: #LEFT ROTATE
                    motor_command.angular.z = 0.50
                else:#RIGHT ROTATE
                    motor_command.angular.z = -0.50
    
    lnum += 1
        
    global motor_command_publisher
    motor_command_publisher.publish(motor_command)    
    
            

def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        ## Here x and y has been incremented with five to make it fit in the terminal
        ## Note that we have lost some map information by shrinking the data
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    ## This square is occupied
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    ## This square is unoccupied
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"
    
def explorer_node():
    rospy.init_node('amble')
    
    max_arr = list() # array of arrays holds the all sub_arrays of each partition during rotation
    global lnum #laser number
    lnum = 0 #initialized as 0
   
    global state, prev_state
    state = "TURN" #start with turning and finding the longest path
    prev_state ="START"
    
    global flag #flag for control of the robot
    flag = 0 #0:360 1:180 2:90    
    
    global lstm # 90 degree rotation amount counter
    lstm = 0
    
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    
    rospy.Subscriber("/scan", LaserScan, laser_callback,(max_arr), queue_size = 1000)
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)
    rospy.spin()
    
    
    
if __name__ == '__main__':
    explorer_node()
