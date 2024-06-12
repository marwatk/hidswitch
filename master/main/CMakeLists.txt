idf_component_register(SRCS "hid_host.c" "main.c" "slave.c" "master.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES usb driver protobuf-c esp_timer tinyusb)
