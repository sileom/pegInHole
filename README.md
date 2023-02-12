# Peg-In-Hole
Code to reconstruct a surface and insert a peg into the holes.

# Requirements
The code works with a Franka Emika Panda Robot and requires the libfranka library to be installed in the system.

## Download CAD model and the Neural Network code
Download the CAD model file from [here](https://drive.google.com/drive/folders/18N9rAZPhX-Q80BK6PPuGz7znlpXTTkfz?usp=share_link) and put it in resources/input folder.

# Installation
To build, open a terminal and run
```shell
cd libfranka/build
make
```

To run the algorithm without neural network
```shell
cd libfranka/pegInHole
run_deeplab.sh
./big_head_insertion <robot-ip> <path to>/resources/output/fori_w.txt <path to save> <number of hole>
```

To run the algorithm with neural network
```shell
cd libfranka/pegInHole
run_deeplab.sh
./big_head_insertion_yolo <robot-ip> <path to>/resources/output/fori_w.txt <path to save> <number of hole>
```
