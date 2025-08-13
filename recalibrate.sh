#!/bin/bash

rosservice call /auv/services/calibrate/EKF "{}"
sleep 2
rosservice call /auv/services/calibrate/vectornav "{}"
