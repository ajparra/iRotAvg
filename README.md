# linfslam


L-inf SLAM

Created by Alvaro Parra on 19/3/19.
Copyright © 2019 Alvaro Parra. All rights reserved.

This is an l1-irls c++ implementation

## Dependencies

 - l1-irls
 - opencv
 - Boost (Filesystem)

 In Mac: 
 - brew install boost
 - brew install opencv3

## Compilation

First download and copy the l1-irls project into the linfslam project's directory.
Then make sure you are able to compile and execute l1_irls. Follow the l1_irls readme for instructions.

- mkdir build
- cd build
- cmake ..
- make

(binary is compiled inside src)

## Execution
To see usage simply excecute

./linfslam config.yaml sequence_path

An example is provided in the folder data. To test the method you can
run

./linfslam ../data/ravg_input.txt

