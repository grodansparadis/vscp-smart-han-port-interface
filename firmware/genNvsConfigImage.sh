#!/bin/sh

# esptool.py -p /dev/ttyACM3 write_flash 0x9000 main/build/nvs-config.bin

python3  ${IDF_PATH}/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py generate config.csv --outdir main/build nvs-config.bin 0x4000