#!/bin/bash

rosservice call /auv/service/calibrate/EKF "{}"
sleep 2
rosservice call /auv/service/calibrate/vectornav "{}"
