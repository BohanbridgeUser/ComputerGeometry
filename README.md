*** 
# Lotus Computer Geometry
![Static Badge](https://img.shields.io/badge/CG-ComputerGeometry-blue)
![Static Badge](https://img.shields.io/badge/Language-C%2B%2B17-red) 
![Static Badge](https://img.shields.io/badge/CGAL-v5.4.4-green)
![Static Badge](https://img.shields.io/badge/Qt-v5.12.8-yellow)
![Static Badge](https://img.shields.io/badge/Boost-v1.71.0-orange)  

This is the repository for source of Lotus Computer Geometry   
   
A repository that contains implementations of Computer Geometry Algorithms.
The data structure of basic geometry primitives such as point, line and planes uses implementations of CGAL.
This repository implements convex hull, intersection, triangulation, Voronoi diagram.  
   
This repo is available for Windows and Linux.   

--- 

# Build Instructions   
### Windows   
To build Lotus Computer Geometry on Windows, `vcpkg` should be installed to set enviroment:
```
vcpkg install qt5 boost cgal glfw3 glad eigen
```
   
Enter the root of project:
```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="<vcpkg-root>/scripts/buildsystems/vcpkg.cmake" ..
make
```    

### Linux   
Qt5, boost, CGAL, OpenGL and Eigen should be installed. The following uses Ubuntu as an example:
```
sudo apt update
sudo apt-get install -y qt5-default libboost-all-dev libeigen3-dev libxmu-dev libxi-dev libgl-dev 
```
Enter the root of project:
```
mkdir build
cd build
cmake ..
make
```
***    

# Overview


