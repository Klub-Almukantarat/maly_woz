import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class FeedForwardControl(Node):
    def __init__(self):
        super().__init__("ff_control")

        self.wheel_separation = 1.0
        self.wheel_radius = 0.14
        self.omega_to_pwm = 10.0

        pwm_freq = 2_000
        self.motor_l_dir_pin = 16
        motor_l_pwm_pin = 12
        self.motor_r_dir_pin = 18
        motor_r_pwm_pin = 35

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        # PWM
        GPIO.setup(motor_l_pwm_pin, GPIO.OUT)
        self.motor_l_pwm = GPIO.PWM(motor_l_pwm_pin, pwm_freq)
        self.motor_l_pwm.start(0)
        GPIO.setup(motor_r_pwm_pin, GPIO.OUT)
        self.motor_r_pwm = GPIO.PWM(motor_r_pwm_pin, pwm_freq)
        self.motor_r_pwm.start(0)

        # Directions
        GPIO.setup(self.motor_l_dir_pin, GPIO.OUT)
        GPIO.setup(self.motor_r_dir_pin, GPIO.OUT)
        GPIO.output(self.motor_l_dir_pin, 1)
        GPIO.output(self.motor_r_dir_pin, 1)

        self.vel_sub = self.create_subscription(Twist, "cmd_vel", self.vel_cb, 10)

    def vel_cb(self, msg: Twist) -> None:
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
        print(lin_vel, ang_vel)

        tmp = ang_vel * self.wheel_separation / 2
        omega_l = (lin_vel - tmp) / self.wheel_radius
        omega_r = (lin_vel + tmp) / self.wheel_radius

        self._set_l_speed(omega_l)
        self._set_r_speed(omega_r)

    def _set_l_speed(self, omega: float) -> None:
        # TODO check directions
        if omega > 0:
            GPIO.output(self.motor_l_dir_pin, 1)
        else:
            omega *= -1
            GPIO.output(self.motor_l_dir_pin, 0)

        duty_cycle = omega * self.omega_to_pwm
        if duty_cycle > 100:
            duty_cycle = 100.0
        self.motor_l_pwm.ChangeDutyCycle(duty_cycle)

    def _set_r_speed(self, omega: float) -> None:
        # TODO check directions
        if omega > 0:
            GPIO.output(self.motor_r_dir_pin, 0)
        else:
            omega *= -1
            GPIO.output(self.motor_r_dir_pin, 1)

        duty_cycle = omega * self.omega_to_pwm
        if duty_cycle > 100:
            duty_cycle = 100.0
        self.motor_r_pwm.ChangeDutyCycle(duty_cycle)


def main(args=None):
    rclpy.init(args=args)

    node = FeedForwardControl()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
