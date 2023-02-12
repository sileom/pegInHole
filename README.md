# Peg-In-Hole
Code to reconstruct a surface and insert a peg into the holes.

This repository contains the implementation of the algorithm described in the paper:
### Assembly task execution using visual 3D surface reconstruction: An integrated approach to parts mating
Michelangelo Nigro, Monica Sileo, Francesco Pierri, Domenico D Bloisi, Fabrizio Caccavale

2023 Robotics and Computer-Integrated Manufacturing

[Paper](https://www.sciencedirect.com/science/article/abs/pii/S0736584522002010) | [Video](https://youtu.be/nc-Ku3eiZqc)

If you use this work, please cite:
```shell
@article{nigro2023assembly,
  title={Assembly task execution using visual 3D surface reconstruction: An integrated approach to parts mating},
  author={Nigro, M and Sileo, M and Pierri, F and Bloisi, DD and Caccavale, F},
  journal={Robotics and Computer-Integrated Manufacturing},
  volume={81},
  pages={102519},
  year={2023},
  publisher={Elsevier}
}
```

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
