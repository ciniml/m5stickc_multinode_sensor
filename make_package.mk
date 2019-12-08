.PHONY: all clean

PACKAGE_VERSION := $(shell git describe --long --dirty)
PACKAGE_NAME := m5stickc_multinode_sensor
PACKAGE_FULLNAME := $(PACKAGE_NAME)-$(PACKAGE_VERSION)

BOOTLOADER_BIN := build/bootloader/bootloader.bin
PARTITION_TABLE_BIN := build/partition_table/partition-table.bin
APP_BIN := build/m5stickc-multinode-sensor.bin
FLASH_PROJECT_ARGS := build/flash_project_args
FLASH_SH := script/flash.sh

all: $(PACKAGE_FULLNAME).zip

$(BOOTLOADER_BIN) $(PARTITION_TABLE_BIN) $(APP_BIN) $(FLASH_PROJECT_ARGS):
	./build.sh

$(PACKAGE_FULLNAME).zip: $(BOOTLOADER_BIN) $(PARTITION_TABLE_BIN) $(APP_BIN) $(FLASH_PROJECT_ARGS) $(FLASH_SH)
	-$(RM) -r $(PACKAGE_FULLNAME)
	-$(RM) $(PACKAGE_FULLNAME).zip
	mkdir -p $(PACKAGE_FULLNAME)
	cp $(BOOTLOADER_BIN) $(PACKAGE_FULLNAME)/
	cp $(PARTITION_TABLE_BIN) $(PACKAGE_FULLNAME)/
	cp $(APP_BIN) $(PACKAGE_FULLNAME)/
	cp $(FLASH_PROJECT_ARGS) $(PACKAGE_FULLNAME)/
	cp $(FLASH_SH) $(PACKAGE_FULLNAME)/
	zip -ru $(PACKAGE_FULLNAME).zip $(PACKAGE_FULLNAME)

clean:
	-$(RM) -r $(PACKAGE_FULLNAME)
	-$(RM) $(PACKAGE_FULLNAME).zip