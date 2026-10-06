idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES esp_driver_usb_serial_jtag esp_timer nvs_flash esp_wifi esp_event esp_netif)
get_filename_component(RUST_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../firmware" ABSOLUTE)
add_prebuilt_library(flix_proxy_firmware "${RUST_ROOT}/target/xtensa-esp32s3-espidf/release/libflix_proxy_firmware.a")
target_link_libraries(${COMPONENT_LIB} PRIVATE flix_proxy_firmware)
