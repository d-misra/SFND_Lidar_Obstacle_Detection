# Sensor Fusion for Self-Driving Cars

**Warning when cloning:** This repository is pretty heavy because the original creators chose to include
Point Cloud Data files at [`src/sensors/data/pcd`](src/sensors/data/pcd); these files sum up to about 342 MB.

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Welcome to the Sensor Fusion for Self-Driving Cars repo.

This repo is about sensor fusion, which is the process of
taking data from multiple sensors and combining it to yield a better
understanding of the world around us. It focuses mostly on two
sensors: [LiDAR](https://en.wikipedia.org/wiki/Lidar), and 
[Radar](https://en.wikipedia.org/wiki/Radar).
Measurements will be fused from these two sensors to track multiple
cars on the road, estimating their positions and speed.

**LiDAR** sensing gives us high resolution data by sending out thousands of
laser signals. These lasers bounce off objects, returning to the sensor where
we can then determine how far away objects are by timing how long it takes
for the signal to return. Also we can tell a little bit about the object
that was hit by measuring the intensity of the returned signal. Each laser
ray is in the infrared spectrum, and is sent out at many different angles,
usually in a 360 degree range. While LiDAR sensors gives us very high
accurate models for the world around us in 3D, they are currently very
expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it
can directly tell us how fast an object is moving in a certain direction.
This ability makes radars a very practical sensor for doing things like
cruise control where its important to know how fast the car in ront of you
is traveling. Radar sensors are also very affordable and common now of days
in newer cars.

**Sensor Fusion** by combing LiDAR's high resolution imaging with radar's
ability to measure velocity of objects we can get a better understanding
of the surrounding environment than we could using one of the sensors alone.


## Building the project

This project requires [PCL](https://pointcloudlibrary.github.io/). On a
Ubuntu-like system, you should be able to install the PCL development
sources like so:

```bash
sudo apt install libpcl-dev
```

The project itself uses CMake. To build, follow the typical CMake pattern: 

```bash
mkdir build && cd build
cmake ..
make
```

You can then run the simulator from the `environment` binary:

```bash
./environment
```
