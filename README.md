# Mars Search Rover
A rover that navigates autonoumously, identifies and retrieves samples of interest on a simulated Martian terrain.


------------

## Simulator Download

The simulator build can be downloaded from these links: [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](  https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip). You can use training mode and navigate the rover manually without the need of any code.

<img src="https://github.com/Salman-H/mars-search-robot/raw/master/figures/sim_screenshot.png" alt="" width="98%">

------------

## Setting up the Project Environment

To setup all dependencies, run the following

``` 
cd <PATH TO PROJECT DIRECTORY>/ env 
```
```
conda env create -f vision-project.yml
```
``` 
source activate vision-project
```


------------


## Running autonomous mode

You can then control the rover autonomously by running the following:

``` 
cd <PATH TO PROJECT DIRECTORY>/code 
```
```
python drive_rover.py
```
After that you can launch the simulator and choose Autonomous mode

