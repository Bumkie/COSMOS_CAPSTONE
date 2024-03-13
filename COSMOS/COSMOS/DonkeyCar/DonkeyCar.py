import time
import Adafruit_PCA9685
from Adafruit_GPIO import I2C



class DonkeyCar():
    
    def __init__(self, main, name="donkey_ros"):
        
        print(">>> [DonkeyCar] 시작")
        self.main = main
        self.name = name
        self.stop_event = main.stop_event
        
        #제어를 위한 객체 생성
        self.throttle = PCA9685(channel=0, busnum=1) #전진/후진을 위한 객체
        self.steering_servo = PCA9685(channel=1, busnum=1) #방향 설정을 위한 객체

        self.autodrive_one()


    def autodrive_one(self):
        speed_pulse = 385
        steering_pulse = 370

        print("donkey test__")
        self.throttle.run(speed_pulse)
        time.sleep(0.2)
        self.steering_servo.run(steering_pulse)
        
        self.throttle.run(0)

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
            
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

        self.pulse = 340
        self.prev_pulse = 340
        self.running = True


    def set_pwm(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))


    def run(self, pulse):
        pulse_diff = pulse - self.prev_pulse

        if abs(pulse_diff) > 40:
            if pulse_diff > 0:
                pulse += 0.7 * pulse_diff
            else:
                pulse -= 0.7 * pulse_diff

        self.set_pwm(pulse)
        self.prev_pulse = pulse


    def set_pulse(self, pulse):
        self.pulse = pulse


    def update(self):
        while self.running:
            self.set_pulse(self.pulse)
