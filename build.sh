#!/bin/bash
# from: https://github.com/dbecad/CppND-Route-Planning-Project

#Remove final bin
rm -rf ./bin

#To rebuild the whole project (slower)
#rm -rf ./build

mkdir build
cd build

cmake ..
make

#Launch result
../bin/CppND-Route-Planning-Project -f ../map.osm
