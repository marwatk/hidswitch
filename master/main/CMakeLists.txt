idf_component_register(SRCS "hid_host.c" "main.c" "master.c"
                       INCLUDE_DIRS "." /opt/esp/idf/components/esp_rom/esp32/include/
                       PRIV_REQUIRES usb driver protobuf-c esp_timer esp_rom tinyusb)
