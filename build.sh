#!/bin/bash

export XTENSA_TOOLCHAIN_ROOT=$HOME/xtensa-esp32-elf/1.22.0-80/xtensa-esp32-elf
export IDF_PATH=`pwd`/esp-idf
source ${IDF_PATH}/add_path.sh
export PATH=${XTENSA_TOOLCHAIN_ROOT}/bin:${PATH}

idf.py $*
