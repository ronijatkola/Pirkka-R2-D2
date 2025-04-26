from machine import Pin, PWM
import utime

# =============== PORT CONFIGURATION ===============
# IR sensors (lateral detection)
IR_LEFT_PIN = 16   # GPIO 16 - Left IR sensor
IR_RIGHT_PIN = 17  # GPIO 17 - Right IR sensor

# Indicator LEDs
RED_LED_PIN = 14   # GPIO 14 - Red LED: no object detected
GREEN_LED_PIN = 15 # GPIO 15 - Green LED: object detected

# Ultrasonic sensor (distance)
TRIGGER_PIN = 18   # GPIO 18 - Ultrasonic sensor trigger
ECHO_PIN = 19      # GPIO 19 - Ultrasonic sensor echo

# Motors (with 2 pins per motor to control direction)
LEFT_MOTOR_FORWARD = 12     # GPIO 12 - Left motor forward
LEFT_MOTOR_BACKWARD = 13    # GPIO 13 - Left motor backward
RIGHT_MOTOR_FORWARD = 10    # GPIO 10 - Right motor forward
RIGHT_MOTOR_BACKWARD = 11   # GPIO 11 - Right motor backward

# =============== PARAMETERS ===============
TARGET_DISTANCE = 15  # Target distance in cm
MARGIN = 5            # Tolerance margin to avoid oscillations
MAX_DETECTION_DISTANCE = 30  # Maximum distance (in cm) to detect objects

# =============== PIN INITIALIZATION ===============
# IR sensors
ir_left = Pin(IR_LEFT_PIN, Pin.IN)
ir_right = Pin(IR_RIGHT_PIN, Pin.IN)

# Indicator LEDs
red_led = Pin(RED_LED_PIN, Pin.OUT)
green_led = Pin(GREEN_LED_PIN, Pin.OUT)

# Ultrasonic sensor
trigger = Pin(TRIGGER_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

# Motors
left_motor_forward = Pin(LEFT_MOTOR_FORWARD, Pin.OUT)
left_motor_backward = Pin(LEFT_MOTOR_BACKWARD, Pin.OUT)
right_motor_forward = Pin(RIGHT_MOTOR_FORWARD, Pin.OUT)
right_motor_backward = Pin(RIGHT_MOTOR_BACKWARD, Pin.OUT)

# =============== FUNCTIONS ===============
def measure_distance():
    """
    Measures distance with the ultrasonic sensor
    Returns the distance in cm
    """
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(10)
    trigger.low()
    
    # Wait for echo to start (timeout of 30000µs = 30ms)
    start_time = utime.ticks_us()
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start_time) > 30000:
            return -1  # Timeout, no object detected
        pass
    
    signal_off_time = utime.ticks_us()
    
    # Wait for echo to end (timeout of 30000µs = 30ms)
    start_time = utime.ticks_us()
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start_time) > 30000:
            return -1  # Timeout, signal too long
        pass
    
    signal_on_time = utime.ticks_us()
    
    # Distance calculation: (time × speed of sound) ÷ 2
    duration = signal_on_time - signal_off_time
    distance = (duration * 0.0343) / 2
    
    return distance

def control_motors(left_direction, right_direction):
    """
    Controls motors with the specified directions.
    - 1: forward
    - 0: stop
    - -1: backward
    """
    # Left motor
    if left_direction == 1:
        # Forward
        left_motor_forward.value(1)
        left_motor_backward.value(0)
    elif left_direction == -1:
        # Backward
        left_motor_forward.value(0)
        left_motor_backward.value(1)
    else:
        # Stop
        left_motor_forward.value(0)
        left_motor_backward.value(0)
    
    # Right motor
    if right_direction == 1:
        # Forward
        right_motor_forward.value(1)
        right_motor_backward.value(0)
    elif right_direction == -1:
        # Backward
        right_motor_forward.value(0)
        right_motor_backward.value(1)
    else:
        # Stop
        right_motor_forward.value(0)
        right_motor_backward.value(0)

def stop_motors():
    """Stops all motors"""
    left_motor_forward.value(0)
    left_motor_backward.value(0)
    right_motor_forward.value(0)
    right_motor_backward.value(0)

# =============== MAIN LOOP ===============
def main():
    print("Starting hand-following robot")
    
    try:
        while True:
            # Reading sensors
            distance = measure_distance()
            left_detection = ir_left.value()
            right_detection = ir_right.value()
            
            # Determine if an object is present (within the maximum detection distance)
            object_present = (distance > 0 and distance <= MAX_DETECTION_DISTANCE) or left_detection or right_detection
            
            # Control indicator LEDs
            green_led.value(1 if object_present else 0)
            red_led.value(0 if object_present else 1)
            
            # Display information (for debugging)
            print(f"Distance: {distance:.1f} cm | Left IR: {left_detection} | Right IR: {right_detection}")
            
            # If no object is detected, stop motors
            if not object_present:
                stop_motors()
                print("No object detected - Motors stopped")
                utime.sleep(0.1)
                continue
                
            # Direction control based on IR sensors
            if left_detection and not right_detection:
                # Turn left (left motor backward, right motor forward)
                left_direction = -1
                right_direction = 1
                print("Turning left")
            elif right_detection and not left_detection:
                # Turn right (left motor forward, right motor backward)
                left_direction = 1
                right_direction = -1
                print("Turning right")
            else:
                # Distance control (forward/backward)
                if distance > 0 and distance <= MAX_DETECTION_DISTANCE:  # Check if distance is valid and within range
                    error = distance - TARGET_DISTANCE
                    
                    if abs(error) <= MARGIN:
                        # Within acceptable margin, maintain position
                        left_direction = 0
                        right_direction = 0
                        print("Maintaining position")
                    elif error > 0:
                        # Too far, move forward
                        left_direction = 1
                        right_direction = 1
                        print("Moving forward")
                    else:
                        # Too close, move backward
                        left_direction = -1
                        right_direction = -1
                        print("Moving backward")
                else:
                    # Invalid distance, stop for safety
                    left_direction = 0
                    right_direction = 0
                    print("Invalid distance measurement")
            
            # Apply calculated directions
            control_motors(left_direction, right_direction)
            
            # Pause to stabilize the system
            utime.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        # Cleanup at the end of the program
        stop_motors()
        red_led.value(0)
        green_led.value(0)
        print("Robot stopped, cleanup done")

# Program start
if __name__ == "__main__":
    main() 