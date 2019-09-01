#!/bin/bash

export IDF_PATH=`pwd`/esp-idf
source ${IDF_PATH}/add_path.sh
export PATH=${HOME}/esp/xtensa-esp32-elf/bin:${PATH}

idf.py $*