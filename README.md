Your user needs acces to i2c:

  sudo usermod -aG i2c $USER
  sudo nano /etc/udev/rules.d/99-i2c.rules
add these lines (file may not exist yet)
  SUBSYSTEM=="i2c-dev", MODE="0666"
  KERNEL=="i2c-[0-1]*", GROUP="i2c"

  sudo udevadm control --reload-rules
  sudo udevadm trigger

relog
