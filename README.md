# DroneNLedu
Author: Jadin Tredup  
Detailed in "Enhancing Graduate Level Nonlinear Control Class with Multirotor Drone Programming" by Dr. Pushkin Kachroo 
and Jadin Tredup (unpublished).

## Introduction
The purpose of this project is to serve as an open-source platform for testing Nonlinear Control Systems for Unmanned
Aerial Vehicles (UAV's) to enhance engineering education. There is currently a large variety of UAV hardware and 
simulation systems available, however, many have restrictions that make them difficult to use for classroom environments. 
Many simulation systems require prerequisite software dependencies to run making it difficult to repeat setup across systems. 
Working with hardware in a classroom environment presents its own set of difficulties. UAV's commonly used in research 
are notoriously dangerous in very close quarters. Flying multiple, or even one UAV inside a classroom presents a huge liability 
for student safety. Furthermore, flying most UAV's indoors is expressly prohibited by many buildings without a proper 
flight-cage.    

With all of this in mind, the project was built with the following constraints in mind:
- The project should have both hardware and simulation elements based off Python 3.
- None of the elements should be prohibitively expensive.
- Hardware should be small with as little potential for danger as possible.
- Simulation is built on an open-source physics engine.
- All software is OS agnostic.

Additionally, because the software is to be accessible by all skill levels, the installation instructions are intentionally 
written for those without much background in coding.

## Hardware Info
The hardware currently supported is the Cheerson CX-10WD-TX, a nano quadcopter that can be purchased either on Amazon or 
a number of other online retailers for about $30.00. Weighing at only 17 grams, the UAV has a 0.3 megapixel FPV camera and 
a 6-axis gryo for flight stablization. It has quite a bit of functionality for such a small and cheap UAV.  Additionally, 
thanks to efforts by the open source community to derive theCX-10WD-TX’s communication protocol, its functionality hasbeen 
extended by a variety of programming languages, making it one of the most readily available programmable multirotordrones.

![Micro Drone](https://github.com/JadinTredup/DroneNLedu/blob/master/Images/cx10wd.jpg "Cheerson Micro Drone")

## Simulation Info
The simulation side of things is handled by PyBullet, a package which provides a Python API for interacting with the open-source 
Bullet physics environment. As of right now, the simulation environment only has one quadcopter model, but more can be added easily 
with the inclusion of new URDF files.

## Installation  
This installtion guide assumes one of the following:
- There is a fresh installation of Python 3 on the host PC.
- There is a fresh Python 3 virtual environment (recommended).
- The user can troubleshoot conflicting dependencies if not using a fresh environment.  

If none of these things are true, guides for installing Python 3 and using the Virtual Environment 
manager can be found in the [Guides Section](https://github.com/JadinTredup/droneNLedu/Guides).

### Download the Repository

First, download the repository as a zip-file from the github page by clicking the buttons outlined in red in the image below.

![Download Zip](https://github.com/JadinTredup/DroneNLedu/blob/master/Images/DownloadZip.png)

### Dependencies and Requirements

## Testing the Installation

### Hardware Tests

#### Connecting to Drone
The first step to running the code for the hardware modules is to connect your host PC to the wifi network established by the drone. To do this, power on your drone and wait a few seconds for the signal to be emitted. Then, on your host PC, navigate to where you configure your wifi connection and conenct to the drones network. It's name will be some variation of "XXXX" as can be seen in the images below.

### Simulation Tests
