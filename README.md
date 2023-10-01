# Fiducial marker testbench

The program uses MQTT to communicate with 2 nodes, which each control a stepper motor. The steppermotors are used to change the distance from the camera to the marker and to change the viewing angle of the marker with respect to the camera.
The program uses 5 different models to detect the marker. The models are:
- arUco
- piTag
- aprilTag
- STag
- CCTag
The program run each model with different distances and angles, and write the results to a csv file.

The project is made as an attempt to compare different types of fiducial markers, by subjecting them to the same test.
The markers were tested by changing the distance between the markers and the camera, the viewing angle of the camera and the size of the marker.
Each marker were tested in the following sizes: 1x1cm, 1.25x1.25cm, 1.5x1.5cm, 1.75x1.75cm and 2x2cm
The camera used was a Logitech C920 1080hp webcam.


## Getting Started

To test the markers, you will need a setup cable of moving the markers around. I had an ESP32 running 2 steppermotor, one controlling the distance from the camera to the marker
the other controlling the viewing angle by rotating the markers horizontaly.

The config.json can be used to enable or disable the models to run, if pictures should be taken with the different markers outlined(pitag is not fully finished so the drawing results are not precise), amongst other things. The config file is loaded during startup, where the path is provided by the user to the program.

Once a model has run, the data will be avalible in a csv file format, along with pictures if this option was set.


### Prerequisites

This project uses the following libraries:

- Boost 1.66.0
- OpenCV
- TinyXML
- cJSON


It also uses 5 different fiducial marker detection libraries, which repositories are the following:

- https://github.com/bbenligiray/stag commit version: 020f1b8
- https://github.com/alicevision/CCTag commit version: 7e2ef3c
- https://github.com/AprilRobotics/apriltag commit version: b7bd75c
- https://github.com/mpetroff/pi-tag-detector commit version: ab745a3
- https://github.com/AprilRobotics/apriltag/tree/master commit version: b7bd75c


### Installing

Besides installing the different libraries, not models, needed only ccTAG, apriltag and pitag models will have to be downloaded and installed.

To install CCTag following the instructions from their README.

To install AprilTag download the repo and run the CMakeLists. 

To install PiTag do the same as for AprilTag.

If your are having problems finding the libraries after installing, check if they are added in local/include. You may have do a little work there.


## Know Errors
For my test i set the camera to autofocus, as it had this option. This proved to introduce an error in the detection, as the first frames capture in each position would be blurry, because of the time it took the camera to autofocus in.

Also the code will run, but it is far from perfect. This is because of the programmers lack of skill and knowledge...

## Authors

  - **Mikkel Husted** - *Provided README Template* -
    [dkhusted](https://github.com/dkhusted)

## Acknowledgments
This project leans heavly on the work done by the different people, whose models have been used for the project.
Without these models, this project would not have been possible.





