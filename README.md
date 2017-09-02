# Main software repository for Nvision SegwayRMP220

This is the main software repository for the Nvision segway project. It contains all software, as well as hardware projects.

- For an introduction to the project, see the site [nvision](http:).
- For extensive technical documentation, please refer to [ nvision][ habonbon doc].

Notice
=======
You should git pull everytime before you push a new commit!
Master now is been protected 

Building
========
pass: ubuntu 14.04.5
ROS version: indigo 

git clone https://github.com/segway-Nvision/rmp220-segway-nvision.git 

To compile the code, 

```
cd ~/rmp220-segway-nvision/catkin_ws

catkin_make

cd ..

source environment.sh
```

Usage
========

1.**teleop with joystick**

open 2 terminal

**terminal 1**

```bash=
cd ~/rmp220-segway-nvision/

source environment.sh

roslaunch roslaunch rmp_teleop joystick.launch
```

**terminal 2**

```bash=
cd ~/rmp220-segway-nvision/

source environment.sh

roslaunch rmp_base rmp_base.launch
```



