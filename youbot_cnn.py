"""move_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, PositionSensor, Camera , DistanceSensor
import math 
import numpy as np
import cv2 


MAX_SPEED = 5
TIME_STEP = 64
FORWARD_SPEED = 1 * MAX_SPEED
ARMS_SPEED =  0.2 * 1.50
GRIPPER_OPEN = 0.03
GRIPPER_CLOSE = 0.01
TURN_SPEED  = 1 * MAX_SPEED
SAFE_DISTANCE_LEFT = 1100
SAFE_DISTANCE =  900


WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.39 
MAX_SPEED_RAD = 6.28




STATE_DETECT = 0
STATE_NAVIGATE = 1

robot_state = STATE_DETECT

TARGET_X = 0.0
TARGET_Z = 1.17
DESTINATION_THRESHOLD = 0.05
 

DEGREES = 45
ANGLE_THRESHOLD  = 1
DISTANCE = 0.02

# create the Robot instance.
robot = Robot()

robot.step(500)

# get the time step of the current world.
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
    
    
#2.Intiliazing the gripper actuators:

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

grip_sensor = robot.getDevice('grip_sensor')
grip_sensor.enable(TIME_STEP)    
   
    
#3.Reading the camera:

camera = robot.getDevice('camera')
camera_grip  = robot.getDevice('camera_grip')
width = camera.getWidth()
height = camera.getHeight()
frame_center  = width//2
camera.enable(TIME_STEP)


# GPS AND INERTAIL UNIT READING

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

iu = robot.getDevice('inertial unit')
iu.enable(TIME_STEP)

def get_yaw():
    v = iu.getRollPitchYaw()
    yaw = round(math.degrees(v[2]))
    
    if yaw < 0 :
        yaw+=360
    
    return yaw
    
print('Turning : {}'.format(DEGREES))
starting_yaw = get_yaw()
print('Start {}' + str(starting_yaw))
target = (starting_yaw + DEGREES) % 360
print('Target : {}'.format(target))


distance_coordinates  = [

    DISTANCE * math.cos(math.radians(target)),
    DISTANCE * math.sin(math.radians(target))

]
    
             

#4.Reading the sensors for the wheels/arm_sensors/fingers_sensors

#Wheels_sensors

wheels = ['wheel1sensor', 'wheel2sensor' , 'wheel3sensor', 'wheel4sensor']
wheels_sensors  = [robot.getDevice(names) for names in wheels]

for sensors in wheels_sensors:
    sensors.enable(TIME_STEP)
    

#Arms_sensors:

arms = ['arm1sensor', 'arm2sensor', 'arm3sensor', 'arm4sensor', 'arm5sensor']
arms_sensors = [robot.getDevice(names) for names in arms]

for sensors in arms_sensors:
    sensors.enable(TIME_STEP)
    
#Fingers_sensors

finger_l_s  = robot.getDevice('finger1sensor')
finger_l_s.enable(TIME_STEP)

finger_r_s  = robot.getDevice('finger2sensor')
finger_r_s.enable(TIME_STEP) 


#camera function:

def open_camera(robots_camera):
    raw = robots_camera.getImage()
    img  = np.frombuffer(raw  , np.uint8).reshape(height , width , 4)
    img = img[:,:,2]
    img = cv2.cvtColor(img ,  cv2.COLOR_BGRA2BGR)
    
    return img

grip_camera  = robot.getDevice('camera_grip')
grip_camera.enable(TIME_STEP)
frame_center_grip = width//2

def open_camera_grip(camera_grip):
    raw = camera_grip.getImage()
    img  = np.frombuffer(raw , np.uint8).reshape(width , height , 4)
    img = img[:,:,3]
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
    
  
ARM_PICKUP_POS_BLUE = [0.0,-0.261795 ,-1.0472 , -0.523603 , 0.0]
ARM_PICKUP_POS_RED = [0.13,-0.14 ,-1.0772 , -0.92 , 0.0]
ARM_DRIVE_POS = [0.0,0.0,0.0,0.0,0.0]
ARM_PLACE_POS_RED = [0.0,-0.0 ,-0.65 , -0.75 , 0.0]
ARM_PLACE_POS_BLUE = [0.0, -0.0, -1.20, -0.523603, 0.0]


def arm_position(target_pos , arm_motors):

    if len(target_pos) != len(arm_motors):
        print("The number of the angles is not equal to the ARM MOTORS")  
    for i , motor in enumerate(arm_motors):
        target_angle = target_pos[i]
        motor.setVelocity(0.90)
        motor.setPosition(target_angle)
    robot.step(1000)      

        
def open_gripper(open , grippers):
   for motor in grippers:
       motor.setPosition(open) 
   robot.step(500)

       
def close_grippers(close , grippers):
    for motor in grippers:
        motor.setPosition(close)    
    robot.step(500)

        
# -----------------------------------------------------------------

def set_omni_speed(wheel_motors, vx, vy, omega):
    
    omega1 = (vy - vx - WHEEL_BASE * omega) / WHEEL_RADIUS
    
    omega2 = (vy + vx + WHEEL_BASE * omega) / WHEEL_RADIUS
    
    omega3 = (vy + vx - WHEEL_BASE * omega) / WHEEL_RADIUS
    
    omega4 = (vy - vx + WHEEL_BASE * omega) / WHEEL_RADIUS

    
    max_calc_omega = max(abs(omega1), abs(omega2), abs(omega3), abs(omega4))
    
    if max_calc_omega > MAX_SPEED_RAD:
        scale = MAX_SPEED_RAD / max_calc_omega
        omega1 *= scale
        omega2 *= scale
        omega3 *= scale
        omega4 *= scale

    
    wheel_motors[0].setVelocity(omega1) 
    wheel_motors[1].setVelocity(omega2) 
    wheel_motors[2].setVelocity(omega3)  
    wheel_motors[3].setVelocity(omega4)  
    
    
    
def drive_forward():
    set_omni_speed(wheel_motors , 0.0 , FORWARD_SPEED , 0.0)
    

def rotation_stop(wheels):
   
        wheels[0].setVelocity(FORWARD_SPEED)
        wheels[1].setVelocity(-FORWARD_SPEED)
        wheels[2].setVelocity(FORWARD_SPEED)
        wheels[3].setVelocity(-FORWARD_SPEED)
    
    
def drive_right(wheels):

    wheels[0].setVelocity(-TURN_SPEED)
    wheels[1].setVelocity(-FORWARD_SPEED)
    wheels[2].setVelocity(TURN_SPEED)
    wheels[3].setVelocity(FORWARD_SPEED)
    
    #robot.step(50 * 6)
    
    

        
    
def stop():
    set_omni_speed(wheel_motors , 0.0 , 0.0 , 0.0)
    robot.step(200)
    

def drive_back(wheels):
    wheels[0].setVelocity(-FORWARD_SPEED)
    wheels[1].setVelocity(-FORWARD_SPEED)
    wheels[2].setVelocity(-FORWARD_SPEED)
    wheels[3].setVelocity(-FORWARD_SPEED)
    
    robot.step(50 * 3)
    


def drive_right_side():
    set_omni_speed(wheel_motors , FORWARD_SPEED , FORWARD_SPEED , 0.0)
    

def drive_left_side():
    set_omni_speed(wheel_motors , FORWARD_SPEED , 0.0 , 0.0) 
    


def angle_to_target(current_pos , target_pos):
    dx = target_pos[0] - current_pos[0] #X
    dx = target_pos[1] - current_pos[1] #Z
    
    angle = math.degrees(math.attan2(dz,dx))
    if angle < 0:
        angle += 360
    return angle           
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:


    gps_val = gps.getValues()
    
    ds_mid = distance_sensor.getValue()
    ds_right = distance_sensor_right.getValue()
    ds_left = distance_sensor_left.getValue()
    
    
    grip_left_val = finger_l_s.getValue()
    grip_right_val = finger_r_s.getValue()
     
    frame = open_camera(camera)
    frame_grip = open_camera_grip(camera_grip)
    
    hsv = cv2.cvtColor(frame , cv2.COLOR_BGR2HSV)
    hsv_frame_grip = cv2.cvtColor(frame_grip , cv2.COLOR_BGR2HSV) 
    
    cx , cy = width//2 , height//2
    h,s,v = hsv[cy,cx]
    #print(f'HSV  = H:{h} , S={s}, V={v}')
  
      
    L_R1 = np.array([0,0,255])    
    U_R1 = np.array([10,255,255])
    mask_red = cv2.inRange(hsv,L_R1,U_R1)
    
    
    #L_R1_grip = np.array([0,0,255])    
    #U_R1_grip = np.array([10,255,255])
    #mask_red_grip = cv2.inRange(hsv_frame_grip,L_R1_grip,U_R1_grip)
    
    
    #visualize_mask(mask_red)
    #visualize_mask_grip(mask_red_grip)
    
    #contours_red_grip,_ = cv2.findContours(mask_red_grip , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
      
    contours_red,_ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE)
    if robot_state ==  STATE_DETECT:
    
        if len(contours_red) > 0:  #and len(contours_red_grip) > 0:
            goal_object = contours_red[0]
            #goal_object_grip = max(contours_red_grip , key=cv2.contourArea) 
        
            x,y,w,h = cv2.boundingRect(goal_object)
            #x_grip,y_grip,w_grip,h_grip = cv2.boundingRect(goal_object_grip)
            
            center_x  = x + w // 2
            #center_x_grip = x_grip + w_grip // 2
        
            if abs(center_x - frame_center) < 20: #and  abs(center_x_grip - frame_center_grip) < 5:
                stop()
         
                if ds_mid > SAFE_DISTANCE:
                  
                    print('Object is close , Stopping')
                    print('Distance : {}'.format(ds_mid))   
                    stop()
                    arm_position(ARM_PICKUP_POS_RED , arm_motors)
                    close_grippers(GRIPPER_CLOSE , grippers)
                    robot.step(200)
                    arm_position(ARM_DRIVE_POS , arm_motors)
                    close_grippers(GRIPPER_CLOSE , grippers)
                    
 
                    print('Object has been secured... Navigating to the desk')
                    drive_right(wheel_motors)
                    

                else:
                 
                    print('Object has been found , following..')
                    set_omni_speed(wheel_motors , 0.0 , FORWARD_SPEED , 0.0)
                    print('GPS :  {}'.format(gps_val))
                    #stop()
                
            elif center_x < frame_center :
                print('Object found - left side ,  turning left')
                #set_omni_speed(wheel_motors, 0.0, FORWARD_SPEED * 0.1, 0.5)
                rotation_stop(wheel_motors)
            else:
                print('Object found - right side  , turning right')
                set_omni_speed(wheel_motors, 0.0, FORWARD_SPEED * 0.1, -0.5)
                #stop() 
            
        else:
            #print('Object not found.... Scanning the area')
            #rotation_stop(wheel_motors)
            drive_right(wheel_motors)                
                
        
          
    #elif robot_state == STATE_NAVIGATE:
    
    #current_yaw  = get_yaw()
    #print(f'Current yaw {current_yaw} and Target Yaw {target}')
    
    #angle_error  = target - current_yaw
    
    #if angle_error > 180:
        #angle_error -=360
    #elif angle_error <-180:
        #angle_error +=360
        
    
    #if abs(angle_error) > 5: #Still need to turn
        
        #if angle_error > 0:
            #for motor in wheel_motors:
                #motor.setVelocity(TURN_SPEED)
                
                
         #else:
             #for motor in wheel_motors:
                 #motor.setVelocity(-TURN_SPEED)
         #print(f'Turning error : {angle_error}')
         
         
    #else:
        #print('Heading correct , moving forward')
        #set_omni_speed(wheel_motors , 0.0 , FORWARD_SPEED , 0.0)
              
          
    
    

# Enter here exit cleanup code.
 