# Modern Robotics:  Mechanics, Planning, and Control
# C++ Library

This repository contains the code library accompanying [_Modern Robotics: 
Mechanics, Planning, and Control_](http://modernrobotics.org) (Kevin Lynch 
and Frank Park, Cambridge University Press 2017). The 
[user manual](https://github.com/NxRLab/ModernRobotics/doc/MRlib.pdf) is in the doc directory of [main repository](https://github.com/NxRLab/ModernRobotics/).

The functions are available in:

* C++
* [Python](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python)
* [MATLAB](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Matlab)
* [Mathematica](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Mathematica)

Each function has a commented section above it explaining the inputs required for its use as well as an example of how it can be used and what the output will be. This repository also contains a pdf document that provides an overview of the available functions using MATLAB syntax. Functions are organized according to the chapter in which they are introduced in the book. Basic functions, such as functions to calculate the magnitude of a vector, normalize a vector, test if the value is near zero, and perform matrix operations such as multiplication and inverses, are not documented here.

The primary purpose of the provided software is to be easy to read and educational, reinforcing the concepts in the book. The code is optimized neither for efficiency nor robustness.

## Installation

### 1. Install Eigen library.
* On Mac
```console
foo@bar:~$ brew install eigen
# Don't forget to ensure that you have correct library version in your path
foo@bar:~$ export EIGEN3_INCLUDE_DIR=/usr/local/Cellar/eigen/3.3.5/include/eigen3/
```
* On Linux
```console
foo@bar:~$ sudo apt-get install eigen
```
 
### 2. Prepare build
```console
foo@bar:~$ mkdir build && cd build
```

By default cmake will install our build into the system directories.
To define a custom install directory we simply pass it to cmake:
```console 
foo@bar:~$ cmake .. -DCMAKE_INSTALL_PREFIX=../_install
```
Or just configure with defaults
```console 
foo@bar:~$ cmake .. 
```
Building and installing library
```console 
foo@bar:~$ make && make install
```

## Using library
```console
foo@bar:~$ g++ test.cpp -I../include -L../build -lModernRoboticsCpp
```
If you get something like: 
```console
foo@bar:~$ ./a.out: error while loading shared libraries: libModernRoboticsCpp.so: cannot open shared object file: No such file or directory
```
Just tell your operating system where the library is:
```console
foo@bar:~$ export LD_LIBRARY_PATH=/path/to/the/library/ModernRoboticsCpp/_install/lib
 ```
