import smbus
import time
import RPi.GPIO as GPIO
import curses
import serial
import math
import deck_generation1
import card_detector

# MPU9250 Registers
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
GYRO_ZOUT_H = 0x47
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D

# Motor GPIO Pins
PWMA = 18   # PWM control for Motor A
AIN1 = 23   # Direction control for Motor A
AIN2 = 24   # Direction control for Motor A
PWMB = 19   # PWM control for Motor B
BIN1 = 17   # Direction control for Motor B
BIN2 = 22   # Direction control for Motor B used to be 22
STBY = 25   # Standby pin

# Pins for communication with Arduino
shoot_pin = 8
spin_pin = 6

# Initialize the I2C bus
bus = smbus.SMBus(1)
device_address = 0x68  # MPU9250 device address

# Time step for integration
dt = 0.01  # Loop time in seconds

# PID Parameters
Kp = 0.1
Ki = 0.0
Kd = 0.0

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY, shoot_pin, spin_pin], GPIO.OUT)

# Create PWM instances
pwmA = GPIO.PWM(PWMA, 100)  # 100 Hz frequency for Motor A
pwmB = GPIO.PWM(PWMB, 100)  # 100 Hz frequency for Motor B

# MPU9250 Initialization
def MPU9250_Init():
    # Write to power management register
    bus.write_byte_data(device_address, PWR_MGMT_1, 0x00)
    # Write to sample rate register
    bus.write_byte_data(device_address, SMPLRT_DIV, 0x07)
    # Write to configuration register
    bus.write_byte_data(device_address, CONFIG, 0x00)
    # Write to gyroscope configuration register
    bus.write_byte_data(device_address, GYRO_CONFIG, 0x18)  # ±2000°/s
    # Write to accelerometer configuration register
    bus.write_byte_data(device_address, ACCEL_CONFIG, 0x00)
    # Write to interrupt enable register
    bus.write_byte_data(device_address, INT_ENABLE, 0x01)

# Read raw data from a specified address with error handling
def read_raw_data(addr):
    try:
        high = bus.read_byte_data(device_address, addr)
        low = bus.read_byte_data(device_address, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value = value - 65536
        return value
    except OSError as e:
        print(f"I2C Error: {e}")
        return 0

# Get gyroscope Z-axis data
def get_gyroscope_z_rate():
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    # Convert to degrees per second (assuming ±2000°/s range and sensitivity of 16.4 LSB/(°/s))
    gyro_z_rate = gyro_z / 16.4
    return gyro_z_rate

# Get accelerometer data for X and Y axes
def get_accelerometer_data():
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    # Convert to g's (assuming ±2g range and sensitivity of 16384 LSB/g)
    acc_x = acc_x / 16384.0
    acc_y = acc_y / 16384.0
    return acc_x, acc_y

# Serial control
def send_serial_command(command):
    try:
        # Initialize serial port
        ser = serial.Serial('/dev/ttyACM0', 9600)  # Update with correct serial port and baud rate
        # Send command over serial
        ser.write(command.encode())
        ser.close()
    except Exception as e:
        print(f"Error sending command: {e}")

# PID Controller
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# Function to control Motor A
def motor_a_control(speed, direction):
    GPIO.output(STBY, GPIO.HIGH)
    if direction == "forward":
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)

# Function to control Motor B
def motor_b_control(speed, direction):
    GPIO.output(STBY, GPIO.HIGH)
    if direction == "forward":
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)
    pwmB.ChangeDutyCycle(speed)

# Function to stop both motors
def motors_stop():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output(STBY, GPIO.LOW)

# Gradually change motor speed
def smooth_speed_change(current_speed, target_speed, step=5):
    if current_speed < target_speed:
        return min(current_speed + step, target_speed)
    elif current_speed > target_speed:
        return max(current_speed - step, target_speed)
    else:
        return current_speed

# Dealing the card to n players
def dealing(deal_pile, current_angle, number_of_players):
    time_factor = 1.1
    angle = math.floor(360 / number_of_players * (deal_pile - 1))
    turn_angle = angle - current_angle
    if turn_angle > 180:
        turn_angle -= 360
    if turn_angle < -180:
        turn_angle += 360
    yaw_angle = 0
    direction_left = "backward"
    direction_right = "backward"
    if turn_angle < 0:
        direction_left = "forward"
    if turn_angle > 0:
        direction_right = "forward"
      
        
    while abs(yaw_angle - turn_angle) >= 3:
        motor_b_control(25 + abs(yaw_angle - turn_angle) / 5, direction_left)
        motor_a_control(25 + abs(yaw_angle - turn_angle) / 5, direction_right)
        time.sleep(dt)
        gyro_z_rate = get_gyroscope_z_rate()
        yaw_angle += gyro_z_rate * dt * time_factor
    motor_b_control(0, direction_left)
    motor_a_control(0, direction_right)
    
    return angle

def shoot(stdscr):
    MPU9250_Init()
    pwmA.start(0)
    pwmB.start(0)
    yaw_angle = 0
    pid = PID(Kp, Ki, Kd, setpoint=yaw_angle)
    base_speed = 50  # Base speed for motors
    direction = "forward"
    motor_a_active = False
    motor_b_active = False
    target_motor_a_speed = 0
    target_motor_b_speed = 0
    current_motor_a_speed = 0
    current_motor_b_speed = 0
    GPIO.output(shoot_pin, GPIO.LOW)
    GPIO.output(spin_pin, GPIO.LOW)
    
    try:
        stdscr.nodelay(1)
        stdscr.timeout(100)

        while True:
            key = stdscr.getch()

            if key == ord('w'):
                direction = "forward"
                motor_a_active = True
                motor_b_active = True
            elif key == ord('s'):
                direction = "backward"
                motor_a_active = True
                motor_b_active = True
            elif key == ord('a'):
                motor_b_control(base_speed, "backward")
                motor_a_control(base_speed, "forward")
                motor_a_active = False
                motor_b_active = False
                continue
            elif key == ord('d'):
                motor_b_control(base_speed, "forward")
                motor_a_control(base_speed, "backward")
                motor_a_active = False
                motor_b_active = False
                continue
            elif key == ord('u'):
                base_speed = min(base_speed + 15, 100)
            elif key == ord('l'):
                base_speed = max(base_speed - 15, 0)
            elif key == ord(' '):
                # Shoot command
                GPIO.output(shoot_pin, GPIO.HIGH)
                time.sleep(0.1)  # Adjust delay to match Arduino code
                GPIO.output(shoot_pin, GPIO.LOW)
                
            elif key == ord('o'):
                # start spinning command
                GPIO.output(spin_pin, GPIO.HIGH)
                time.sleep(0.1)  # Adjust delay to match Arduino code
                GPIO.output(spin_pin, GPIO.LOW)
            else:
                motor_a_active = False
                motor_b_active = False

            try:
                gyro_z_rate = get_gyroscope_z_rate()
                # acc_x, acc_y = get_accelerometer_data()
                yaw_angle += gyro_z_rate * dt - 0.0125  # correction
            except OSError:
                yaw_angle = 0  # Reset yaw angle if there is an I2C read error

            correction = pid.compute(yaw_angle, dt)
            if direction == "backward":
                correction = -2.5 * correction
                    
            target_motor_a_speed = base_speed + 2 * correction
            target_motor_b_speed = base_speed - 2 * correction
            target_motor_a_speed = max(min(target_motor_a_speed, 100), 0)
            target_motor_b_speed = max(min(target_motor_b_speed, 100), 0)

            current_motor_a_speed = smooth_speed_change(current_motor_a_speed, target_motor_a_speed)
            current_motor_b_speed = smooth_speed_change(current_motor_b_speed, target_motor_b_speed)

            if motor_a_active:
                motor_a_control(current_motor_a_speed, direction)
            else:
                pwmA.ChangeDutyCycle(0)

            if motor_b_active:
                motor_b_control(current_motor_b_speed, direction)
            else:
                pwmB.ChangeDutyCycle(0)

            stdscr.addstr(0, 0, f"Yaw Angle: {yaw_angle:.2f}, Correction: {correction:.2f}, Motor A Speed: {current_motor_a_speed:.2f}, Motor B Speed: {current_motor_b_speed:.2f}")
            stdscr.refresh()
            time.sleep(dt)
    
    except KeyboardInterrupt:
        # Send stop command before exiting
        send_serial_command('e')
        pass

    finally:
        motors_stop()
        GPIO.cleanup()

def deal():
    print("checkpoint 3")
    GPIO.output(spin_pin, GPIO.HIGH)
    time.sleep(0.2)
    GPIO.output(spin_pin, GPIO.LOW)
    number_of_players = 4
    current_angle = 0
    print("checkpoint 2")
    dealt_num = [0] * number_of_players
    # Generate card pile using number_of_players
    print("checkpoint 1")
    generation = deck_generation1.generation()
    for i in range(52):
        #card = card_detector.card_identify()
        print(" the card is: ")
        #print(card)
        #pile = generation[card]-1
        pile = generation[i]-1
        # If the pile has 13 cards, choose a pile that does not have 13 cards
        while dealt_num[pile] == 13:
            pile = (pile + 1) % number_of_players
        dealt_num[pile] += 1
        # Turn to right angle
        print (" the pile is: ")
        print(pile)
        current_angle = dealing(pile, current_angle, number_of_players)
        GPIO.output(shoot_pin, GPIO.HIGH)
        time.sleep(0.1)  # Adjust delay to match Arduino code
        GPIO.output(shoot_pin, GPIO.LOW)
        time.sleep(2)
        print ("dealt cards = ")
        print(i)
'''
def main():
    MPU9250_Init()
    pwmA.start(0)
    pwmB.start(0)
    GPIO.output(shoot_pin, GPIO.LOW)
    GPIO.output(spin_pin, GPIO.LOW)
    
    key = input("Press 1 to deal, Press 2 to shoot cards: ")
    if key == '1':
        deal()
    if key == '2':
        shoot()
'''


if __name__ == "__main__":
    MPU9250_Init()
    pwmA.start(0)
    pwmB.start(0)
    GPIO.output(shoot_pin, GPIO.LOW)
    GPIO.output(spin_pin, GPIO.LOW)
    
    key = input("Press 1 to deal, Press 2 to shoot cards: ")
    if key == '1':
        print("checkpoint 0")
        deal()
        
    if key == '2':
        curses.wrapper(shoot)

