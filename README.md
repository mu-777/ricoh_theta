# ricoh_theta
ROS node for ricoh theta v0.1

**CAUTION**  
This ROS package is not official  
I have nothing to do with Ricoh Company, Ltd.

## Overview
![theta viewer](http://uploda.cc/img/img5508c9546bd7f.png)

## Quick start
1. Connect your PC to your theta via wifi

2. ```rosrun theta_service theta_capture_server.py```

3. ```rqt --standalone rqt_theta_viewer``` @ another terminal  
or ```rqt``` and Plugins -> Visualization -> Theta Viewer

4. Click shutter and wait for seconds  
(your rqt window may be black out but it is temporal, just wait)

5. Now you can see your theta image :)



