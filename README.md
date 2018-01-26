# LEIA-ME #

Quick guidelines to set up the system onboard

### DRONI: Dirigível robótico de concepção inovadora ###

* Quick summary
* Version 1.0
* Manual v1.0 available on [drive link](https://drive.google.com/file/d/1HWO_bzj1xkyPWd0h4xp2d0hq8KFojqW6/view?usp=sharing)

### Setting up the onboard computer ###
* Download the custom mtig driver from miguel's github account [master branch](https://github.com/makquel/mtig_driver)
* Dowload the custom flea3 driver from kumar's github accouunt [master branch](https://github.com/KumarRobotics/flea3)
* Check for dependencies
* Setup droni.rules at /etc/udev/rules.d/

### Setting mavlink communication ###
* Launch mavlink_system script under ros2mav folter to establish communication with QGC station


