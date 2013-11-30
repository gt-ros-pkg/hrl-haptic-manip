#!/bin/bash -x

roslaunch sandbox_marc_darpa_m3 teleop.launch &

sleep 10

roslaunch sandbox_marc_darpa_m3 teleop_controller.launch &