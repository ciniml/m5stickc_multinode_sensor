#!/bin/sh
esptool.py -p COM6 -b 1500000 --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/m5stickc-multinode-sensor.bin &
esptool.py -p COM7 -b 1500000 --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/m5stickc-multinode-sensor.bin &
esptool.py -p COM34 -b 1500000 --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/m5stickc-multinode-sensor.bin &
