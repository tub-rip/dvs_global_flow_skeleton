# dvs_global_flow

Estimation of a global optical flow vector with an event camera.
Using the [Contrast Maximization method](http://rpg.ifi.uzh.ch/docs/CVPR18_Gallego.pdf), the implemented algorithm estimates an optical flow vector (u,v) shared by all the events on the image during a short time window.

## Dependencies

- dvs ROS messages
- catkin simple
- Google Logs
- Google Flags

## Compiling

	catkin build dvs_global_flow
	source devel/setup.bash

## Running the code
In one terminal:

	roscore
	
In another terminal:

	rqt --perspective-file gflow.perspective


In another terminal:

	roslaunch dvs_global_flow ijrr_translation.launch
	
For this last part, please make sure that the launch file has the correct path to the data (e.g., ROS bag `slider_far.bag` from the [Event Camera Dataset IJRR 2017](http://rpg.ifi.uzh.ch/davis_data.html))