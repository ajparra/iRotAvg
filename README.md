# IRA

Author: [Alvaro Parra](http://alvaroparra.com)

Incremental Rotation Averaging (IRA) incrementally solves rotation averaging. IRA is the optimisation core of L-ininity SLAM presented in [[Á. Parra, T.-J. Chin, A. Eriksson, I. Reid: Visual SLAM: Why bundle adjust?, ICRA 2019](https://cs.adelaide.edu.au/~aparra/publication/parra19_icra/)]. 



This work was supported by [Maptek](http://maptek.com) and the ARC Grant DP160103490.

## Related Publication:

[Á. Parra, T.-J. Chin, A. Eriksson, I. Reid: Visual SLAM: Visual SLAM: Why bundle adjust?, ICRA 2019](https://cs.adelaide.edu.au/~aparra/publication/parra19_icra/)


## License

IRA is released under a GPLv3 license. 

For a closed-source version of RAL (e.g., for commercial purposes), please [contact the author](https://cs.adelaide.edu.au/~aparra/#contact): alvaro.parrabustos at adelaide dot edu dot au.

For an academic use of IRA, please cite:
[Á. Parra, T.-J. Chin, A. Eriksson, I. Reid: Visual SLAM: Visual SLAM: Why bundle adjust?, ICRA 2019](https://cs.adelaide.edu.au/~aparra/publication/parra19_icra/)


## Dependencies

- Eigen
- SuiteSparse
- opencv
- Boost (Filesystem)


In Mac: 

- `brew install eigen`
- `brew install suite-sparse`
- `brew install opencv`
- `brew install boost`

In Ubuntu:
-  `sudo apt install libeigen3-dev`
- `sudo apt-get install libsuitesparse-dev`
- `sudo apt-get install libboost-all-dev`
- `For OpenCV wise, https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/ provides a good guide.`



## Compilation

- `mkdir build`
- `cd build`
- `cmake ..`
- `make`

(binary is compiled inside src)


## Config
You can test linfslam with the kitti dataset:
https://universityofadelaide.box.com/s/b9vwelbfu64f3j0m48p6aetfaywm6u3l


There is a yaml configuration file here
maptek_dataset/LeftCam/config.yaml

Download and uncompress the orb vocabulary file from here
https://github.com/raulmur/ORB_SLAM2/tree/master/Vocabulary

Edit the path for the orb vocabulary in the configuration file.


## Execution
To see usage options simply execute
```
./ira
```

IRA can be executed as ORB-SLAM: 

./ira orb_vocab.txt config.yaml sequence_path

where

- orb_vocab.txt    -- orb bocabulary as in ORB-SLAM 
- config.yaml        -- configuration file as in ORB-SLAM 
- sequence_path  -- directory containing the sequence frames
