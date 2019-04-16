# linfslam


L-inf SLAM

Created by Alvaro Parra on 19/3/19.
Copyright © 2019 Alvaro Parra. All rights reserved.

This is an l1-irls c++ implementation

## Dependencies

 - l1-irls
 - opencv3
 - Boost (Filesystem)

 In Mac: 
 - brew install boost
 - brew install opencv3

## Compilation

First download and copy the l1-irls project into the linfslam project's directory.
Then make sure you are able to compile and execute l1_irls. Follow the l1_irls's readme for instructions.

- mkdir build
- cd build
- cmake ..
- make

(binary is compiled inside src)

## Config
You can test linfslam with the maptek dataset. The dataset is here:
(external hhd for now...)


There is a yaml configuration file here
maptek_dataset/LeftCam/config.yaml

Download and uncompress the orb vocabulary file from here
https://github.com/raulmur/ORB_SLAM2/tree/master/Vocabulary

Edit the path for the orb vocabulary in the configuration file.


## Execution

./linfslam path/to/config.yaml path/to/sequence_path

