
import math
import time


def autodrive(self):
    self._steering_servo.run(370)


    target_x = -30
    target_y = 100

    throttle_pulse = 380
    steering_pulse = 360

    NEUTRAL_STEERING_PULSE = 360 
    MAX_LEFT_STEERING_PULSE = 700
    MAX_RIGHT_STEERING_PULSE = 200


    current_x = 0
    current_y = 0
    current_rotation = 90

    angle = math.atan2((target_y - current_y), (target_x - current_x))
    
    steering_angle = math.degrees(angle)

    required_rotation = current_rotation - steering_angle

    steering_ratio = required_rotation / 160.0

    if required_rotation >0 :
        steering_pulse = int(NEUTRAL_STEERING_PULSE + steering_ratio * (MAX_RIGHT_STEERING_PULSE - NEUTRAL_STEERING_PULSE))

    elif required_rotation < 0 :
        steering_pulse = int(NEUTRAL_STEERING_PULSE - steering_ratio * (MAX_LEFT_STEERING_PULSE - NEUTRAL_STEERING_PULSE))

    else:
        steering_pulse = 360

    distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

    move_time = 0.15 * distance/10

    self._steering_servo.run(360)


    while(move_time >0):

        self._steering_servo.run(steering_pulse)

        self._throttle.run(throttle_pulse)
        time.sleep(0.2)
        move_time -= 0.2
        print(steering_pulse)
        print(move_time)
    
    self._throttle.run(0)
    self._steering_servo.run(370)