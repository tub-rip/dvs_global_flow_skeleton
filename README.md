# dvs_global_flow

Estimation of a global optical flow vector with an event camera.
Using the [Contrast Maximization method](http://rpg.ifi.uzh.ch/docs/CVPR18_Gallego.pdf), the implemented algorithm estimates an optical flow vector (u,v) shared by all the events on the image during a short time window.

## Documentation and video
- [Problem statement: global optical flow estimation using Contrast Maximization](https://drive.google.com/file/d/1WQG3vHLGF299q8imsEq-ZVJUTJk7BSk9/view?usp=sharing)
- [Video example](https://youtu.be/bPsMyLcc_0M)

## Input / Output
**Input**:
- Events (topic)

**Output**:
- Image velocity (a 2D vector) obtained by maximizing the contrast of the image of warped events (optimization algorithm).
- Image of warped event IWE (i.e., event image), without and with motion compensation using the estimated flow vector.

**Parameters** (in the launch file):
- Number of events to process together in the "sliding window".
- Number of events to shift in the "sliding window" for the next packet of events.
- Type of objective function to be optimized (variance, mean square).
- Whether to use polarity or not.
- Amount of Gaussian smoothing for voting in the IWE (typically 1 pixel).
- Verbosity level (>=0), for printing and debugging purposes.


## Dependencies

### Create a catkin workspace

Create a catkin workspace (if there is none yet). For example, from your home folder:

	cd
	mkdir -p catkin_ws/src
	cd catkin_ws
	catkin config --init --mkdirs --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

Depending on the ROS distribution you installed, you might have to use `kinetic` instead of `melodic` in the previous command.

### Add packages to the catkin workspace

Clone this repository into the `src` folder of your catkin workspace.

The catkin package dependencies are:
- [catkin simple](https://github.com/catkin/catkin_simple)
- ROS messages for DVS ([rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros))
- [Google Logging Library (glog)](https://github.com/catkin/catkin_simple.git)
- [Gflags (formerly Google Commandline Flags)](https://github.com/ethz-asl/gflags_catkin)

The above dependencies are specified in the [dependencies.yaml](dependencies.yaml) file. They can be installed with the following commands from the `src` folder of your catkin workspace:

	cd catkin_ws/src
	sudo apt-get install python3-vcstool
	vcs-import < dvs_global_flow/dependencies.yaml

The previous command should clone the repositories into folders *catkin_simple*, *rpg_dvs_ros*, etc. inside the src/ folder of your catkin workspace, at the same level as this repository *dvs_global_flow*. They should NOT be inside the *dvs_global_flow* folder.

The [GSL library](http://www.gnu.org/software/gsl/) is a scientific library that can be installed with the command:

	sudo apt-get install libgsl-dev

## Compiling

**Preliminaries**:
First, build `catkin simple` and the `davis_ros_driver`. The most important commands are:

	catkin build catkin_simple
	catkin build davis_ros_driver

Then, incorporate the packages to the path, so that ROS finds them:

	source ~/catkin_ws/devel/setup.bash

**Compile this package**:

	catkin build dvs_global_flow --force-cmake

The flag `--force-cmake` is optional.
After building, at least the first time, remember to run:

	source ~/catkin_ws/devel/setup.bash

Sometimes (in case of strange errors) it is useful to delete the build folder before compilation:

	rm -rf build/dvs_global_flow/

An alternative command to start from scratch (cleaning all catkin packages) is (to be used with *caution*): `catkin clean`

## Running the code
Download a ROS bag, for example [slider_close](http://rpg.ifi.uzh.ch/datasets/davis/slider_close.bag), to use as source of data (as if an event camera was connected to the computer).

ROS bags with linear motion from the [Event Camera Dataset IJRR 2017](http://rpg.ifi.uzh.ch/davis_data.html):
- [slider_close.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_close.bag)
- [slider_far.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_far.bag)
- [slider_hdr_close.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_hdr_close.bag)
- [slider_hdr_far.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_hdr_far.bag)

You may also test on the first seconds of sequences with dominantly linear motion and approximately constant depth, for example, the first 7 seconds of [boxes_translation.bag](http://rpg.ifi.uzh.ch/datasets/davis/boxes_translation.bag).

In a terminal:

	roslaunch dvs_global_flow ijrr_translation.launch

Please make sure that the launch file has the correct path to the data (e.g., ROS bag).

End the program execution with `Ctrl + C` keyboard shortcut.


## References
- Gallego et al., *[A Unifying Contrast Maximization Framework for Event Cameras...](https://arxiv.org/pdf/1804.01306)*, CVPR 2018. [PDF](http://rpg.ifi.uzh.ch/docs/CVPR18_Gallego.pdf),  [Poster](http://rpg.ifi.uzh.ch/docs/CVPR18_Gallego_poster.pdf),  [YouTube](https://youtu.be/KFMZFhi-9Aw),  [Spotlight presentation](https://youtu.be/IOntXI5iRzA).
- Gallego et al. *[Focus Is All You Need: Loss Functions For Event-based Vision](http://rpg.ifi.uzh.ch/docs/CVPR19_Gallego.pdf)*, CVPR 2019. [PDF arXiv](https://arxiv.org/pdf/1904.07235), [Poster](http://rpg.ifi.uzh.ch/docs/CVPR19_Gallego_poster.pdf), [YouTube](https://youtu.be/SU_Fp4xS_g4?t=1059)
