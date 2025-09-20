# PiClock
Clock with LED's and stepper motor control run on a Pi Zero 2 W. The OS it was developed for was Bookworm 32-bit. Tested python version is 3.11.2.

## Install
Enable UART and SPI interfaces in raspi-config
```base
sudo raspi-config
```
Update the packages that come with the OS and install all the pre-requisits, clone the repository, and install the python modules in a python vertual enviroment
```bash
sudo apt update
sudo apt full-upgrade
sudp apt install python3-dev git
git clone https://github.com/abestat2/PiClock
cd PiClock
source PiClock_env/bin/activate
pip install -r requirements.txt
```

