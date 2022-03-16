# Modern Robotics:  Mechanics, Planning, and Control
# C++ Library

This repository contains the code library accompanying [_Modern Robotics:
Mechanics, Planning, and Control_](http://modernrobotics.org) (Kevin Lynch
and Frank Park, Cambridge University Press 2017). The
[user manual](https://github.com/NxRLab/ModernRobotics/blob/master/doc/MRlib.pdf) is in the doc directory of [main repository](https://github.com/NxRLab/ModernRobotics/).

The functions are available in:

* C++
* [Python](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python)
* [MATLAB](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Matlab)
* [Mathematica](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Mathematica)

Each function has a commented section above it explaining the inputs required for its use as well as an example of how it can be used and what the output will be. This repository also contains a pdf document that provides an overview of the available functions using MATLAB syntax. Functions are organized according to the chapter in which they are introduced in the book. Basic functions, such as functions to calculate the magnitude of a vector, normalize a vector, test if the value is near zero, and perform matrix operations such as multiplication and inverses, are not documented here.

The primary purpose of the provided software is to be easy to read and educational, reinforcing the concepts in the book. The code is optimized neither for efficiency nor robustness.

## Installation 

### 1. Install Software
* Eigen Library: On Linux and Mac
```console
foo@bar:~$ sudo apt-get install libeigen3-dev
```
* Eigen Libray: Mac
```console
foo@bar:~$ brew install eigen
```

* cmake: On Linux
```console
foo@bar:~$ sudo apt-get install cmake
```
```console
foo@bar:~$ mkdir build && cd build
```

### 2. Download Repo
* Download Repo:
```console
foo@bar:~$ 
```
* Extract Repo:
```console
foo@bar:~$ sudo unzip 
```

### 3. Prepare build
```console
foo@bar:~$ mkdir build && cd build
```

Or just configure with defaults
```console
foo@bar:build $ cmake ..
```
Building library
```console
foo@bar:build $ sudo make
```
Installing library
```console
foo@bar:build $ sudo make install
```


## Testing the library
```console
foo@bar:build $ ./lib_test
```
