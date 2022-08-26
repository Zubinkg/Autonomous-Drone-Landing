import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M'

if len(sys.argv) > 1:
    URI = sys.argv[1]


found1 = False
found2 = False

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver = False)
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf =cf) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multi_ranger:
                motion_commander.up(0.1) 
                def approach():
                #this is the advanced approach function with non tuned PID values to correct for drift in a more practical manner 
                #The PID controller requires tuning and some modification but was not accomplished during the timeframe accurately
                    velocity = 0.2
                    Kp = 0.01
                    Kd = 0.05
                    error = multi_ranger.front - 0.3
                    current_time = time.time() 
                    last_error = 0
                    last_time = 0
                    delta_time = current_time - last_time 
                    delta_error = error - last_error
                    while error > 0:
                        delta_time = time.time() - last_time
                        delta_error = error - last_error
                        velocity = (Kp * error) + (Kd * delta_error) + velocity 
                        motion_commander.start_forward(velocity)
                        last_error = error

                def pivot():
                    # This function rotates and checks the front ranging sensor to detect landing markers  
                    # close to the drone's starting position. The first part uses a while loop to continue rotating
                    # until the drone detects an obstacle or if the time to complete the rotation is over
                    # if the drone detects an obstacle within its range then it moves forward until 0.3m away 
                    # and rotates to prepare to the drone to circle around the marker/obstacle 
                    # if the drone starts out closer than or roughly equal to it moves back until it is 0.3m away
                    end_time1 = time.time()+ 5.0
                    motion_commander.start_turn_left()
                    while multi_ranger.front > 0.7 and time.time() < end_time1:
                        continue
                    if time.time() > end_time1:
                        pass
                    if multi_ranger.front <= 0.7 and multi_ranger.front> 0.3:
                        dist2 = multi_ranger.front
                        motion_commander.forward( dist2 - 0.3) 
                        motion_commander.turn_right(6)
                        motion_commander.turn_left(60)
                        found1 = True
                        print('detected obstacle during pivot')
                        return found1
                    elif multi_ranger.front <= 0.3: 
                        motion_commander.back(0.3 - multi_ranger.front)
                        time.sleep( 0.1)
                        motion_commander.turn_right(10)
                        motion_commander.turn_left(60)
                        found1 = True 
                        return found1
                    else: 
                        found1 = False
                        return found1
                
                def approach_right():
                    motion_commander.right((multi_ranger.right - 0.3))
                    motion_commander.turn_right(80)
                
                def circle_search():
                    # this is the function that directs the drone to search in concentric circles  
                    # of increasing radius. The radius increase is defined as 0.3m and the drone completes 3 attempts 
                    # After these three attempts are completed without detection, the drones moves to the outer edge 
                    # to detect the rare cares where the landing site is distant from the drone 
                    # The function uses a while loop to increment the circle radius and move within the circles until detection 
                    # The two detection cases are treated differently with the front sensor moving to the marker 
                    # and rotating at a fixed angle to initiate the next function properly 
                    # Dectection from the right ranging sensor will cause the drone to move again 0.3m away from the obstacle 
                    radius = 0.3
                    range1 = 0.3
                    attempt = 0
                    flight_time2 = 31.415 * radius * 1.14
                    end_time2 = time.time() + flight_time2
                    while attempt < 3:
                        motion_commander.right(0.3)
                        motion_commander.start_circle_left(radius)
                        while (multi_ranger.front > 0.5 and multi_ranger.right > 0.5) and time.time() < end_time2:
                            continue
                        if time.time() > end_time2:
                            radius += range1
                            attempt += 1
                            flight_time2 = 31.415 * radius
                            end_time2 = time.time() + (1.14 * flight_time2)
                            print('starting new circle')
                        if multi_ranger.front <= 0.5:
                            approach()
                            found2 = True
                            return found2
                            print('detected obstacle during repeat_check')
                        if multi_ranger.right <= 0.5:
                            dist3 = multi_ranger.right
                            motion_commander.right((dist3 - 0.3))
                            found2 = True
                            return found2
                            print('detected obstacle during repeat check') 
                    if attempt >=3:
                        motion_commander.forward(1.83)
                        motion_commander.start_circle_left(1.83)
                        radius = 1.83
                        while multi_ranger.left > 0.5 and time.time() < end_time2:
                            continue
                        if multi_ranger.left <= 0.5:
                            dist4 = multi_ranger.left
                            motion_commander.left(dist4)
                            found2 = True
                            return found2
                def centering(): 
                    #This is the core function of the algorithm which directs the drone to circle
                    # around the detected marker to detect the second one and center itself on the midpoint 
                    # between the two of them and then move forward to search for the second set of markers for the square arrangement 
                    # After detecting the second set the drone moves back to roughly half the side length which is represented by the dist variable 
                    # The deviations from the perfect numbers 1.5 and 0.5 for the distances moved is drift compensation
                    dist = 0
                    move_dist = 0 
                    motion_commander.start_circle_right(0.3)
                    while multi_ranger.left > 0.3: 
                        continue                     
                    if multi_ranger.left <= 0.3: 
                        print('detected obstacle while centering')
                        move_dist = ((0.3 - multi_ranger.left)/2)
                        dist = ((multi_ranger.left + 0.3))
                        #advanced case of more than four markers, then the dist variable would be set to: 
                        #dist = ((multi_ranger.left + 0.3)/(2 * tan(180/n))) where n is the number of sides
                        motion_commander.right(move_dist)
                        print('moving forward')
                        time.sleep(0.5)
                        motion_commander.turn_left(6)
                        motion_commander.forward((dist/2))
                        end_time3 = time.time() + 2.7
                        motion_commander.start_forward()
                        time.sleep(0.75)
                        while (multi_ranger.right > 0.35 and multi_ranger.left > 0.08 and multi_ranger.front > 0.35) and time.time() < end_time3: 
                            continue
                        if multi_ranger.right <=0.35: 
                            time.sleep(0.1)
                            motion_commander.turn_right(10)
                            motion_commander.back(0.57*dist)
                            pass
                        elif multi_ranger.front <= 0.08: 
                            motion_commander.back(0.57*dist)
                            pass
                        elif multi_ranger.left <= 0.35: 
                            motion_commander.turn_left(10)
                            motion_commander.back(0.57*dist)
                            pass
                        elif multi_ranger.right and multi_ranger.left <= 0.2: 
                            motion_commander.back(0.57*dist)    
                        elif time.time() > end_time3: 
                            motion_commander.back((2 * dist))
                    #advanced method to check all the markers and verify the center position
                    
                    #motion_commander.start_turn_left()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
                    # while multi_ranger.front > 0.2: 7
                            #continue
                    #if multi_ranger.front <= 0.2:
                        # motion_commander.start_turn_right() 
                        # while multi_ranger.front > 0.2: 
                    #           continue 
                    #       if multi_ranger.front < 0.2: 
                    #           motion_commander.back(dist) 
                    # if time > flight_time: 
                    #    motion_commander.back((velocity* flight_time))
                    #    while (multi_ranger.right > 0.2 and multi_ranger.left > 0.2) and time < flight_time2: 
                    #         motion_commander.start_forward()
                    #         if multi_ranger.right <=0.2:
                    #             dist = multi_ranger.right
                    #             motion_commander.start_turn_left()
                    #             while multi_ranger.front > 0.2: 
                    #                  continue
                    #             if multi_ranger.front <= 0.2:
                    #                motion_commander.start_turn_right() 
                    #                while multi_ranger.front > 0.2: 
                    #                   continue 
                    #                if multi_ranger.front < 0.2: 
                    #                   motion_commander.back(dist) 
                time.sleep(2)
                found1 = pivot()
                if found1 is True: 
                   centering()
                   print('working on centering after finding obstacle during pivot')
                else:
                   found2 = circle_search()
                   print('working on repeat check')
                   if found2 is True: 
                       centering()
                       print('working on centering after repeat check')
                   else: 
                      print('oops')
                