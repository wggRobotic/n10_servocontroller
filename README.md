# Setup
Your user needs acces to i2c:
```
sudo usermod -aG i2c $USER
sudo nano /etc/udev/rules.d/99-i2c.rules
```
add these lines (file may not exist yet):
```  
  SUBSYSTEM=="i2c-dev", MODE="0666"
  KERNEL=="i2c-[0-1]*", GROUP="i2c"
```
reload rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
relog

# Functionality
 
nodename `n10_servo_controller`

subscribed:
- `/n10/servo_cmd_wheels` `Float32MultiArray` `length 6`
  -  control wheelservos -pi/2 to pi/2
- `/n10/servo_cmd_arm` `Float32MultiArray` `length 6`
    - control robotarmservos -pi/2 to pi/2

