# Description 
The servocontroller node to move the 6 servos for wheels and the 3 servos for the robotarm

ros2 name: `n10_servo_controller`

wheel-servo-channels: 0-5, usual wheel enumeration

# Setup
In order to access GPIO for I2C, the user running the node needs I2C permissions:
```
sudo usermod -aG i2c $USER
```
Open (file may not exist yet):
```
sudo nano /etc/udev/rules.d/99-i2c.rules
```
add these lines:
```  
  SUBSYSTEM=="i2c-dev", MODE="0666"
  KERNEL=="i2c-[0-1]*", GROUP="i2c"
```
Reload the rules:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Finnally, you may need to relog.

# Interface

## subscribed
- `/n10/servo_cmd_wheels` `std_msgs::msg::Float32MultiArray (6)` : control wheel servos
  - from `-π/2 ≐ pointing right` to `π/2 ≐ pointing left`

- `/n10/servo_cmd_arm` `std_msgs::msg::Float32MultiArray (3)` : control robotarm servos
  - index 0: base_joint `0 ≐ pointing up` `counterclockwise`
  - index 1: arm_joint `0 ≐ same as base_joint` `counterclockwise`
  - index 2: gripper state `π/2 ≐ open` `-π/2 ≐ closes`
