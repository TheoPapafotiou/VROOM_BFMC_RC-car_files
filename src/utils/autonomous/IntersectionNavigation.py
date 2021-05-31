import math

class IntersectionNavigation:

    def intersection_navigation(self, starting_yaw, current_yaw, direction):

        print("Hello!")
        offset = 5
        steering = 14.0
        navigation = False

        starting_yaw = 360 - starting_yaw  if (starting_yaw >= 180) else starting_yaw
        current_yaw = 360 - current_yaw  if (current_yaw >= 180) else current_yaw

        if abs(current_yaw - starting_yaw) not in range(90 - offset, 90 + offset):
            steering = steering if direction == "right" else -steering
            speed = 0.3
            navigation = True 
        print(steering)
        print(starting_yaw, current_yaw)
        return steering, speed, navigation
