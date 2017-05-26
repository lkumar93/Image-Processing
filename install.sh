#!/usr/bin/env bash

apt-get install libopencv*
cd corner_detection && mkdir build && cd build && cmake .. && make -j7 && make install
cd ../../edge_detection && mkdir build && cd build && cmake .. && make -j7 && make install
cd ../../image_filters && mkdir build && cd build && cmake .. && make -j7 && make install
cd ../../image_enhancement && mkdir build && cd build && cmake .. && make -j7 && make install
cd ../../image_transformation && mkdir build && cd build && cmake .. && make -j7 && make install
cd ../../motion_estimation && mkdir build && cd build && cmake .. && make -j7 && make install
