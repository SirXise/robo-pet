from controller import Robot, Keyboard
import cv2
import numpy as np
import math
import random

# Constants
TIMESTEP = 32
MAX_SPEED = 6.28
SCALE = 500  # Adjust scaling factor for GPS data
ROBOT_RADIUS = 7  # Radius foAr robot's position marker
WINDOW_SIZE = 800  # Window size to display the map
RETURN_THRESHOLD = 0.1  # Distance threshold to detect return to start
MIN_DISTANCE_TRAVELED = 5.0  # Minimum distance before checking return

# Initialize the robot
robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# Read Keyboard
keyboard = Keyboard()
keyboard.enable(timeStep)

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Sensors
proximity_sensors = [robot.getDevice(f'ps{i}') for i in range(8)]
for ps in proximity_sensors:
    ps.enable(TIMESTEP)

light_sensors = [robot.getDevice(f'ls{i}') for i in range(8)]
for ls in light_sensors:
    ls.enable(TIMESTEP)

# Get the devices for the red, green, and yellow LEDs
red_led = robot.getDevice('led0')    # Red LED
green_led = robot.getDevice('led8')  # Green LED
yellow_led = robot.getDevice('led9') # Yellow LED

# Initialize LEDs to ensure they are all off
red_led.set(0)
green_led.set(0)
yellow_led.set(0)

# GPS and Compass
gps = robot.getDevice('gps')
gps.enable(TIMESTEP)
compass = robot.getDevice('compass')
compass.enable(TIMESTEP)

# Create an empty black image for the map
map_image = np.zeros((WINDOW_SIZE, WINDOW_SIZE, 3), dtype=np.uint8)

# Variables to track the robot's position and state
start_position = None  # The robot's starting position
total_distance_traveled = 0  # Total distance traveled by the robot
completed_perimeter = False  # Whether the robot has completed the perimeter
last_position = None  # To calculate travel distance
free_movement_mode = False

def delay(robot, milliseconds):
    """Delay for the specified number of milliseconds."""
    steps = int(milliseconds / TIMESTEP)
    for _ in range(steps):
        robot.step(TIMESTEP)

def gps_to_map_coords(gps_x, gps_y):
    """Convert GPS coordinates to map coordinates based on scaling."""
    map_x = int(gps_x * SCALE) + (WINDOW_SIZE // 2)
    map_y = int(-gps_y * SCALE) + (WINDOW_SIZE // 2)  # Invert Y-axis for correct visual representation
    return map_x, map_y

def distance_between_points(x1, y1, x2, y2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def update_map(gps_x, gps_y, sensor_values, compass_values):
    """Update the map with the robot's position, leaving a white trail, and detected obstacles."""
    global map_image
    map_x, map_y = gps_to_map_coords(gps_x, gps_y)

    # Leave a white trail for the previous robot path
    cv2.rectangle(map_image,
                  (map_x - ROBOT_RADIUS, map_y - ROBOT_RADIUS),
                  (map_x + ROBOT_RADIUS, map_y + ROBOT_RADIUS),
                  (255, 255, 255), -1)  # White square for the trail

    # Resize the map to fit the window size (800x800)
    resized_map = cv2.resize(map_image, (WINDOW_SIZE, WINDOW_SIZE))
    cv2.imshow("SLAM Map", resized_map)
    cv2.waitKey(1)


def is_light_on(light_values):
    """check light on
    """
    threshold = 3000
    for value in light_values:
        if value < threshold:
            # print("ON", value)
            return True
    # print("OFF", value)
    return False


# Main loop
while robot.step(TIMESTEP) != -1:
    key = keyboard.getKey()

    # Read sensor values
    sensor_values = [ps.getValue() for ps in proximity_sensors]
    light_values = [ls.getValue() for ls in light_sensors]

    # print("Light sensor values:", light_values)

    # Check if the light is on and 'T' is not pressed
    if is_light_on(light_values) and key != ord('T'):
        # Get GPS values
        gps_position = gps.getValues()
        compass_values = compass.getValues()
        robot_x, robot_y, robot_z = gps_position

        # Initialize starting position
        if start_position is None:
            start_position = (robot_x, robot_y)
            last_position = start_position

        # Calculate traveled distance
        distance = distance_between_points(robot_x, robot_y, last_position[0], last_position[1])
        total_distance_traveled += distance
        last_position = (robot_x, robot_y)

        # Check if the robot has returned to the starting position
        if not free_movement_mode and total_distance_traveled > MIN_DISTANCE_TRAVELED and \
                distance_between_points(robot_x, robot_y, start_position[0], start_position[1]) < RETURN_THRESHOLD:
            completed_perimeter = True  # Mark perimeter as completed
            free_movement_mode = True  # Switch to free movement mode
            print("Perimeter traversal completed. Switching to free movement mode.")

        # Update the map with the robot's position
        update_map(robot_x, robot_y, sensor_values, compass_values)

        # Output the current position and total distance traveled
        print(f"Robot position: X={robot_x}, Y={robot_y}, Total distance traveled: {total_distance_traveled:.2f} meters")

        # Movement logic
        if not free_movement_mode:
            # Wall-following logic
            left_wall = sensor_values[5] > 80
            front_wall = sensor_values[7] > 80

            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

            if front_wall:
                print("Front wall detected. Turning.")
                red_led.set(0)      # Turn off the red LED
                green_led.set(0)    # Turn off the green LED
                yellow_led.set(1)   # Turn on the yellow LED
                left_speed = MAX_SPEED
                right_speed = -MAX_SPEED
            elif left_wall:
                print("Left wall detected. Following the wall.")
                red_led.set(0)      # Turn off the red LED
                green_led.set(0)    # Turn off the green LED
                yellow_led.set(1)   # Turn on the yellow LED
                left_speed = MAX_SPEED
                right_speed = MAX_SPEED
            else:
                print("No wall detected. Moving forward.")
                left_speed = -MAX_SPEED / 8
                right_speed = MAX_SPEED
                red_led.set(0)      # Turn off the red LED
                green_led.set(1)    # Turn on the green LED
                yellow_led.set(0)   # Turn off the yellow LED
        else:
            # Free movement logic
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

            if sensor_values[7] > 80:  # Front obstacle
                # Turn randomly to avoid obstacle
                if random.choice([True, False]):
                    print("Front obstacle detected. Turning randomly.")
                    left_speed = MAX_SPEED
                    right_speed = -MAX_SPEED
                    red_led.set(0)      # Turn off the red LED
                    green_led.set(0)    # Turn off the green LED
                    yellow_led.set(1)   # Turn on the yellow LED
                    delay(robot, 2000)
                else:
                    print("Front obstacle detected. Turning in the other direction.")
                    left_speed = -MAX_SPEED
                    right_speed = MAX_SPEED
                    red_led.set(0)      # Turn off the red LED
                    green_led.set(0)    # Turn off the green LED
                    yellow_led.set(1)   # Turn on the yellow LED
                    delay(robot, 2000)
            elif sensor_values[6] > 80:  # Slight right obstacle
                print("Slight right obstacle detected. Turning in the other direction.")
                left_speed = MAX_SPEED
                right_speed = MAX_SPEED / 2
                red_led.set(0)      # Turn off the red LED
                green_led.set(0)    # Turn off the green LED
                yellow_led.set(1)   # Turn on the yellow LED
            elif sensor_values[0] > 80:  # Slight left obstacle
                print("Slight left obstacle detected. Turning in the other direction.")
                left_speed = MAX_SPEED / 2
                right_speed = MAX_SPEED
                red_led.set(0)      # Turn off the red LED
                green_led.set(0)    # Turn off the green LED
                yellow_led.set(1)   # Turn on the yellow LED
            # Free movement logic
            red_led.set(0)      # Turn off the red LED
            green_led.set(1)    # Turn on the green LED
            yellow_led.set(0)   # Turn off the yellow LED
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


    else:
        # Light is off or 'T' is pressed
        # Stop condition: Red LED
        red_led.set(1)      # Turn on the red LED
        green_led.set(0)    # Turn off the green LED
        yellow_led.set(0)   # Turn off the yellow LED
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print("Stopping the robot.")
        continue  # Skip the rest of the loop

# Stop the robot
left_motor.setVelocity(0)
right_motor.setVelocity(0)
cv2.destroyAllWindows()
