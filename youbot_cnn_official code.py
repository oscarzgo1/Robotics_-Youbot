"""move_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor, Camera , DistanceSensor
import math 
import numpy as np
import cv2 


# ======= MOTORS SPEEDS ======= # 

# Variables for the wheels operations
MAX_SPEED = 5
TIME_STEP = 64
FORWARD_SPEED = 1 * MAX_SPEED
ARMS_SPEED =  0.2 * 1.50
GRIPPER_OPEN = 0.025
GRIPPER_CLOSE = 0.01
TURN_SPEED  = 1 * MAX_SPEED
STOP = 0.0
TURN_RATIO = 0.9


# ====== DISTANCE SENSORS - SAFE_DISTACNE  ====== # 

# Variables for the safe distance (if object is close , so the robot won't collie with an object)
SAFE_DISTANCE_LEFT = 1100
SAFE_DISTANCE =  900
SAFE_DISTANCE_BLACK = 880



# ====== ARM POSITION ====== # 

ARM_PICKUP_POS_BLACK = [0.62,-0.44 ,-0.95 , -0.52 , 0.0] # ARM_POSITION for the pickup position for the black colour
ARM_PICKUP_POS_RED = [-0.22,-0.14 ,-1.0772 , -0.92 , 0.0] # ARM_POSITION for the pickup position for the red colour
ARM_DRIVE_POS = [0.0,0.0,0.0,0.0,0.0] # Drive position for the arm
ARM_PLACE_POS_RED = [1.10,-0.10 ,-0.52 , -0.75 , 0.0] # Drive place position for the red colour
ARM_DRIVE_POS_RED = [1.10,0.0,0.0,0.0,0.0] # Arm drive position for the red colour to avoid hitting the shelf
ARM_PLACE_POS_BLACK = [0.60, 1.25, -2.00, -1.30, 0.0] # Arm place position for the black colour
ARM_PLACE_POS_BLACK_2 = [0.40, 0.05, -1.45, -0.30, 0.0] # Arm place positon for the black colour to avoid hitting the shelf

# ========= STATES ========= # 

STATE_SEARCH_RED       = 0
STATE_APPROACH_RED     = 1
STATE_PICKUP_RED       = 2
STATE_SEARCH_QR        = 3
STATE_NAVIGATE_TO_DESK = 4
STATE_PLACE_RED        = 5


STATE_SEARCH_BLACK = 6
STATE_APPROACH_BLACK = 7
STATE_SEARCH_QR_GREEN  = 8
STATE_NAVIGATE_TO_DESK_BLACK = 9
STATE_PICKUP_BLACK = 10
STATE_PLACE_BLACK = 11


robot_state = STATE_SEARCH_RED


# Create the Robot instance.
robot = Robot()
robot.step(500)
timestep = int(robot.getBasicTimeStep())

#Intilazing the actuators for the wheels:

WHEEL_NAMES = ['wheel1', 'wheel2', 'wheel3' , 'wheel4'] 
wheel_motors = [robot.getDevice(wheels) for wheels in WHEEL_NAMES]

#Setting up the wheel position to 'inf' and keeping it's position to being the same.

for motors in wheel_motors:
    motors.setPosition(float('inf'))
    motors.setVelocity(0.0)
    
    
# 1.Intilizaing the arm joints:

ARM_JOINTS = ['arm1', 'arm2', 'arm3' , 'arm4', 'arm5']
arm_motors   = [robot.getDevice(arms) for arms in ARM_JOINTS]

#Setting up for the start the constant speed for the robot's arms;

for arms in arm_motors:
    arms.setVelocity(1.0)
        
#.Intiliazing the gripper actuators:

grippers_name  = ['finger1' , 'finger2']
grippers = [robot.getDevice(name) for name in grippers_name]

#Setting up the max force for the grippers to collect the object from the desk.

for motor in grippers:
    motor.setPosition(0.025)
    motor.setVelocity(0.01)
  


#DistanceSensor and Grip Sensors Intiliazing:    
  
distance_sensor = robot.getDevice('distance_sensor')
distance_sensor.enable(TIME_STEP)

distance_sensor_right  = robot.getDevice('distance_sensor_right')
distance_sensor_right.enable(TIME_STEP)

distance_sensor_left = robot.getDevice('distance_sensor_left')
distance_sensor_left.enable(TIME_STEP)

wheel_sensor_left = robot.getDevice('wheel_sensor')
wheel_sensor_left.enable(TIME_STEP)
         
#3.Reading the camera:

camera = robot.getDevice('camera')
camera_grip  = robot.getDevice('camera_grip')
width = camera.getWidth()
height = camera.getHeight()
frame_center  = width//2
camera.enable(TIME_STEP)



#camera function:

def open_camera(robots_camera):
    raw = robots_camera.getImage() # Getting the image of the camera
    img  = np.frombuffer(raw  , np.uint8).reshape(height , width , 4)
    img = img[:,:,:3] # Height , width , all channels : RGB
    img = cv2.cvtColor(img ,  cv2.COLOR_BGRA2BGR) # converting the colour of the image
    
    return img

grip_camera  = robot.getDevice('camera_grip')
grip_camera.enable(TIME_STEP)
frame_center_grip = width//2

def open_camera_grip(camera_grip):
    raw = camera_grip.getImage()
    img  = np.frombuffer(raw , np.uint8).reshape(width , height , 4)
    img = img[:,:,:3]
    img  = cv2.cvtColor(img , cv2.COLOR_BGRA2BGR)

    return img


def visualize_mask(mask):
    cv2.namedWindow('Vision' , cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Vision' , 600, 300)
    cv2.imshow('Vision', mask)
    cv2.waitKey(TIME_STEP)
    
    
    
def visualize_mask_grip(mask):
    cv2.namedWindow('Vision' , cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Vision' , 600, 300)
    cv2.imshow('Vision', mask)
    cv2.waitKey(TIME_STEP)


######--------------FUNCTIONS------------#######

# Arm function to achieve each desired position for the arm.
def arm_position(target_pos , arm_motors): 

    if len(target_pos) != len(arm_motors): # Checking if the angles for the arms are equal to arm motors.
        print("The number of the angles is not equal to the ARM MOTORS")  
    for i , motor in enumerate(arm_motors):
        target_angle = target_pos[i] # Lopping through ourt actuators.
        motor.setVelocity(0.65) # Setting up the velocity.
        motor.setPosition(target_angle) # Setting up the target position.
    robot.step(1000)      


# Gripper Function (Open)       
def open_gripper(open , grippers):
   for motor in grippers:
       motor.setPosition(open) 
   robot.step(500)

   
# Gripper Function(close)      
def close_grippers(close , grippers):
    for motor in grippers:
        motor.setPosition(close)    
    robot.step(500)
    

#Function for driving_forward   
def drive_forward(wheels):

    wheels[0].setVelocity(FORWARD_SPEED)
    wheels[1].setVelocity(FORWARD_SPEED)
    wheels[2].setVelocity(FORWARD_SPEED)
    wheels[3].setVelocity(FORWARD_SPEED)

# Function for rotating the wheels    
def rotation_stop(wheels , speed = 0.5 * MAX_SPEED):
   
     wheels[0].setVelocity(speed)
     wheels[1].setVelocity(-speed)
     wheels[2].setVelocity(speed)
     wheels[3].setVelocity(-speed)

# Function to drive_right      
def drive_right(wheels):

    turn_speed = 0.3 * MAX_SPEED
    forward_speed = 2 * MAX_SPEED

    wheels[0].setVelocity(-turn_speed)
    wheels[1].setVelocity(forward_speed)
    wheels[2].setVelocity(-turn_speed)
    wheels[3].setVelocity(forward_speed)
    robot.step(50 * 8)
    
# Function to stop (e.g.if obstacles is close)
def stop(wheels,stop):

    wheels[0].setVelocity(stop)
    wheels[1].setVelocity(stop)
    wheels[2].setVelocity(stop)
    wheels[3].setVelocity(stop)
    robot.step(50 * 5)
   
# Function to drive_back
def drive_back(wheels):

    wheels[0].setVelocity(-FORWARD_SPEED)
    wheels[1].setVelocity(-FORWARD_SPEED)
    wheels[2].setVelocity(-FORWARD_SPEED)
    wheels[3].setVelocity(-FORWARD_SPEED)
    robot.step(50 * 1)
    
# Function to drive_left    
def drive_left(wheels):
    
    forward_speed = 0.3 * MAX_SPEED
    turn_speed = 2 * MAX_SPEED

    wheels[0].setVelocity(forward_speed)
    wheels[1].setVelocity(-turn_speed)
    wheels[2].setVelocity(forward_speed)
    wheels[3].setVelocity(-turn_speed)
    robot.step(300)
    
    
# Created a function to scan for the colour in each state    
def find_color(camera, wheel_motors, color_low, color_high, frame_center, timeout=200):

    steps = 0 # Number of steps

    while steps < timeout:

        frame = open_camera(camera) # Camera function
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # HSV Image conversion

        mask = cv2.inRange(hsv, color_low, color_high) # Masking the image so the robot understand where the object is
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Creating contours

        # -------- No objeect --------
        if len(contours) == 0:
            
            rotation_stop(wheel_motors) # Rotate if the object is not found
            robot.step(TIME_STEP)
            steps += 1
            continue  

        goal = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(goal)
        cx = x + w // 2

        
        if abs(cx - frame_center) < 10:
            if cx < frame_center: # Object is left
                drive_left(wheel_motors)
                print('Object left... Following')
                robot.step(TIME_STEP)
            else:
                drive_right(wheel_motors) # Object is right
                print('Object right... Following')
                
        else:
            stop(wheel_motors, STOP)
            return True, goal  

        steps += 1

    
    stop(wheel_motors, STOP)
    return False, None

   
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    ds_mid = distance_sensor.getValue() # Middle sensor values
    ds_left = distance_sensor_left.getValue() # Left sensors values
    ws = wheel_sensor_left.getValue() # Left wheel_sensors values
    frame = open_camera(camera) # Camera
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # HSV Image conversion

    print("Distance sensor =", ds_mid)
    

# ==================ROBOT_STATES================== #

    # ============================================
    # 1) SEARCH_RED
    # ============================================
    if robot_state == STATE_SEARCH_RED: # Using states to navigate robot what to do next in steps

        print("Searching for RED object...")

        RED_LOW  = np.array([0, 0, 255]) # Lower bound for red
        RED_HIGH = np.array([10, 255, 255]) # Upper bound for red

        found, contour = find_color(
            camera,
            wheel_motors,
            RED_LOW,
            RED_HIGH,
            frame_center
        ) 

        if found:
            print("Red object found! Approaching...")
            robot_state = STATE_APPROACH_RED
        else:
            rotation_stop(wheel_motors)
            


    # ============================================
    # 2) APPROACH_RED
    # ============================================
    elif robot_state == STATE_APPROACH_RED:

        print("Approaching RED object...")

        
        if ds_mid < SAFE_DISTANCE:
            print("Close enough — switching to PICKUP_RED")
            stop(wheel_motors, STOP)
            robot_state = STATE_PICKUP_RED # Next State  === STATE_PICKUP_RED ===
            continue

        # If it is still far , continues to drive_forward
        drive_forward(wheel_motors)


    # ============================================
    # 3) PICKUP_RED
    # ============================================
    elif robot_state == STATE_PICKUP_RED:

        print("Picking up RED object...")

        arm_position(ARM_PICKUP_POS_RED, arm_motors)
        close_grippers(GRIPPER_CLOSE, grippers)
        robot.step(500)
        arm_position(ARM_DRIVE_POS, arm_motors)

        print("Red object picked. Searching for QR...")
        robot_state = STATE_SEARCH_QR # Next State === STATE_SEARCH_QR_CODE ===


    # ============================================
    # 4) SEARCH_QR (Purple)
    # ============================================
    elif robot_state == STATE_SEARCH_QR:

        print("Searching for Purple QR code...") # Searching for the purple qr_code

        QR_LOW  = np.array([140, 60, 60]) # Lower bound purple
        QR_HIGH = np.array([175, 255, 255]) # Upper bound purple

        foundqr, contourqr = find_color(
            camera,
            wheel_motors,
            QR_LOW,
            QR_HIGH,
            frame_center
        )

        if foundqr:
            print("QR found! Navigating to desk...")
            robot_state = STATE_NAVIGATE_TO_DESK # Next state === STATE_NAVIGATE_TO_DESK ===
        else:
            rotation_stop(wheel_motors)


    # ============================================
    # 5) NAVIGATE_TO_DESK
    # ============================================
    elif robot_state == STATE_NAVIGATE_TO_DESK:

        print("Navigating to desk...") # Goes to desk

        drive_back(wheel_motors) # Drives_back
        robot.step(50 * 11) # Amount of steps it needs to make

        drive_right(wheel_motors) # Drives_right
        robot.step(50 * 25)

        drive_forward(wheel_motors) # Drive_forward
        robot.step(50 * 75)

        stop(wheel_motors, STOP) # Stops

        robot_state = STATE_PLACE_RED # Next state === STATE_PLACE_RED ===


    # ============================================
    # 6) PLACE_RED
    # ============================================
    elif robot_state == STATE_PLACE_RED:

        print("Placing RED object on desk...")

        arm_position(ARM_PLACE_POS_RED, arm_motors)
        open_gripper(GRIPPER_OPEN, grippers)
        robot.step(300)
        arm_position(ARM_DRIVE_POS_RED, arm_motors)
        arm_position(ARM_DRIVE_POS, arm_motors)

        print("RED object placed successfully.")
        drive_back(wheel_motors)
        robot.step(50 * 25)

        robot_state = STATE_SEARCH_BLACK # Next state === STATE_SEARCH_BLACK === 
        
        
    # ========================================
    # 7) SEARCH_BLUE
    # ========================================   
        
    elif robot_state == STATE_SEARCH_BLACK:
        
        print("Scanning for BLACK object...")
        drive_left(wheel_motors)
        robot.step(50 * 10)

        blue_lower = np.array([0,0,0])
        blue_upper = np.array([0,0,0])

        found_b, contour_b = find_color(
            camera,
            wheel_motors,
            blue_lower,
            blue_upper,
            frame_center
        )

        if found_b:
            print("BLACK object found! Approaching...")
            robot_state = STATE_APPROACH_BLACK
          
        else:
            rotation_stop(wheel_motors , speed = 0.1 * MAX_SPEED)
            


    elif robot_state == STATE_APPROACH_BLACK:

        print("Approaching BLACK object...")

        if ds_mid < SAFE_DISTANCE or ds_left < SAFE_DISTANCE:
            stop(wheel_motors, STOP)
            robot.step(200)
     
            robot_state = STATE_PICKUP_BLACK
            continue

        drive_forward(wheel_motors)


    elif robot_state == STATE_PICKUP_BLACK:

        print("Picking BLACK object...")
        
        if ds_left < SAFE_DISTANCE:
            stop(wheel_motors,STOP)
            arm_position(ARM_PICKUP_POS_BLACK, arm_motors)
            close_grippers(GRIPPER_CLOSE, grippers)
            robot.step(300)
            arm_position(ARM_DRIVE_POS, arm_motors)
        
        print('Picked BLACK. Moving back slighty.')
        drive_back(wheel_motors)
        robot.step(200)
        
        stop(wheel_motors , STOP)
        

        robot_state = STATE_SEARCH_QR_GREEN
        print('Searching GREEN QR...')
        continue


    elif robot_state == STATE_SEARCH_QR_GREEN:
        
        green_lower = np.array([40, 50, 0])
        green_upper = np.array([85, 255, 255])

        found_qr_g, contour_qr_g = find_color(
            camera,
            wheel_motors,
            green_lower,
            green_upper,
            frame_center,
        )

        if found_qr_g:
            print("QR FOUND. Navigating...")
            robot_state = STATE_NAVIGATE_TO_DESK_BLACK
        else:
               drive_forward(wheel_motors)
               robot.step(50 * 5)


    elif robot_state == STATE_NAVIGATE_TO_DESK_BLACK:

        
        wheel_motors[0].setVelocity(0.5 * FORWARD_SPEED * TURN_RATIO)
        wheel_motors[1].setVelocity(0.5 * FORWARD_SPEED * TURN_RATIO)
        wheel_motors[2].setVelocity(-FORWARD_SPEED)
        wheel_motors[3].setVelocity(-FORWARD_SPEED)
        
        if ds_mid < SAFE_DISTANCE_BLACK or ds_left < SAFE_DISTANCE_BLACK or ws < SAFE_DISTANCE_BLACK:
            print("Navigating to desk... Distance =", ds_mid)
            print("Desk close. Stopping.")
            stop(wheel_motors, STOP)
            robot.step(200)
            robot_state = STATE_PLACE_BLACK
        
     


    elif robot_state == STATE_PLACE_BLACK:

        print("Placing BLACK object...")

        arm_position(ARM_PLACE_POS_BLACK, arm_motors)
        robot.step(50 * 30)
        arm_position(ARM_PLACE_POS_BLACK_2, arm_motors)
        robot.step(50 * 30)
        open_gripper(GRIPPER_OPEN, grippers)
        robot.step(300)
        drive_back(wheel_motors)
        robot.step(50 * 20)
        stop(wheel_motors , STOP)
        arm_position(ARM_DRIVE_POS, arm_motors)

        print("BLACK object placed.")
        break

                       
    pass
 
