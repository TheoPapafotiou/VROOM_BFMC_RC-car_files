import math

class IntersectionNavigation:

    def intersection_navigation(self, starting_yaw, current_yaw, direction):

        offset = 8
        left_steering = -13.5
        right_steering = 18
        navigation = True

        starting_yaw = 360 - starting_yaw  if (starting_yaw >= 180) else starting_yaw
        current_yaw = 360 - current_yaw  if (current_yaw >= 180) else current_yaw

        steering = right_steering if direction == "right" else left_steering
        
        if int(abs(current_yaw - starting_yaw)) >= 55:
            speed = 0.2
        else:
            speed = 0.3
            
        if int(abs(current_yaw - starting_yaw)) in range(90 - offset, 90 + offset):
            navigation = False
            
        else:
            navigation = True 
        
        print(starting_yaw, current_yaw)
        return steering, speed, navigation
