@echo on
esptool --chip ESP32 merge_bin -o LoRa_APRS_Meteo+iGate_DDS_CGW_0.00_20230101.bin --flash_mode dio --flash_size 4MB 0xe000 boot_app0.bin 0x1000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin 
