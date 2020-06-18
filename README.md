# dvs_global_flow

Estimation of a global optical flow vector with an event camera.
Using the [Contrast Maximization method](http://rpg.ifi.uzh.ch/docs/CVPR18_Gallego.pdf), the implemented algorithm estimates an optical flow vector (u,v) shared by all the events on the image during a short time window.

## Dependencies

The following catkin package dependencies are specified in the `dependencies.yaml` file. They can be installed with the following commands:

	sudo apt-get install python3-vcstool
	vcs-import < dvs_global_flow/dependencies.yaml

The dependencies are:
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))
- [catkin simple](https://github.com/catkin/catkin_simple)
- Google Logging Library (glog)
- Gflags (formerly Google Commandline Flags)

The [GSL library](http://www.gnu.org/software/gsl/) is a scientific library that can be installed with the command:

	sudo apt-get install libgsl-dev
	
## Compiling

	catkin build dvs_global_flow
	source devel/setup.bash

## Running the code
In a terminal:

	roslaunch dvs_global_flow ijrr_translation.launch
	
For this last part, please make sure that the launch file has the correct path to the data (e.g., ROS bag).

ROS bags with linear motion from the [Event Camera Dataset IJRR 2017](http://rpg.ifi.uzh.ch/davis_data.html):
- [slider_close.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_close.bag)
- [slider_far.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_far.bag)
- [slider_hdr_close.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_hdr_close.bag)
- [slider_hdr_far.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_hdr_far.bag)

Also, test on the first seconds of sequences with dominantly linear motion. 