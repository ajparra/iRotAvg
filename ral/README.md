
# RAL

Author: [Alvaro Parra](http://alvaroparra.com)

The Rotation Averaging Library (RAL) implements rotation averaging algorithms.

The current RAL version adapted L1RA and IRLS algorithms [Chatterjee, A. and Govindu, V.: "Efficient and robust large-scale rotation averaging." ICCV 2013] to fix an arbitrary number of rotations. Instead of conventionally fixing one rotation only (usually the first rotation as the identity), fixing multiple rotations allows to:
- Incorporate known absolute rotations to rotation averaging.
- Efficiently combine multiple rotations averaging problems, e.g., addressing a (large) problem by incrementally optimising over the relative rotations or by chunks.


This work was supported by [Maptek](http://maptek.com) and the ARC Grant DP160103490.

## Related Publications:

[Álvaro Parra, Tat-Jun Chin, Anders Eriksson, Ian Reid: Visual SLAM: Why bundle adjust?, ICRA 2019](https://cs.adelaide.edu.au/~aparra/publication/parra19_icra/)


## License

RAL is released under a GPLv3 license. 

For a closed-source version of RAL (e.g., for commercial purposes), please [contact the author](https://cs.adelaide.edu.au/~aparra/#contact): alvaro.parrabustos at adelaide dot edu dot au.

For an academic use of RAL, please cite:
[Álvaro Parra, Tat-Jun Chin, Anders Eriksson, Ian Reid: Visual SLAM: Why bundle adjust?, ICRA 2019](https://cs.adelaide.edu.au/~aparra/publication/parra19_icra/)


## Dependencies

 - SuiteSparse
 - Eigen
 
 In Mac: 
 
 - `brew install eigen`
 - `brew install suite-sparse`

 In Ubuntu:
 -  `sudo apt install libeigen3-dev`
 - `sudo apt-get install libsuitesparse-dev`
 
 
## Compilation

- `mkdir build`
- `cd build`
- `cmake ..`
- `make`

## Execution
To see usage simply execute
```
./l1_irls
```

An example is provided in the folder data. To test the method run:
```
./l1_irls ../data/ravg_input.txt
```
