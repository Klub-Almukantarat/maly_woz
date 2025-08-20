import RPi.GPIO as GPIO
from time import sleep

enable_pin_l = 16
pwm_pin_l = 12  # PWM pin
enable_pin_r = 18
pwm_pin_r = 35  # PWM pin
GPIO.setwarnings(False)  # disable warnings
GPIO.setmode(GPIO.BOARD)  # set pin numbering system
GPIO.setup(pwm_pin_l, GPIO.OUT)
GPIO.setup(pwm_pin_r, GPIO.OUT)
GPIO.setup(enable_pin_l, GPIO.OUT)
GPIO.setup(enable_pin_r, GPIO.OUT)

pi_pwm_l = GPIO.PWM(pwm_pin_l, 2000)  # create PWM instance with frequency
pi_pwm_r = GPIO.PWM(pwm_pin_r, 2000)  # create PWM instance with frequency
pi_pwm_l.start(20)  # start PWM of required Duty Cycle
pi_pwm_r.start(20)  # start PWM of required Duty Cycle

GPIO.output(enable_pin_l, 1)
GPIO.output(enable_pin_r, 1)

while True:
    sleep(0.5)
