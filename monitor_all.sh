#!/bin/bash
# 同时监控所有关键主题
rostopic echo /mavros/state &
rostopic echo /mavros/rc/out &
rostopic echo /mavros/global_position/rel_alt &
rostopic echo /mavros/altitude &
rostopic echo /mavros/imu/data &
rostopic echo /mavros/setpoint_raw/attitude &
wait
