# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS protocol_examples_common
                            "components/esp32-camera")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware)

# These binary targets are referenced in the source file using 
# extern const uint8_t client_cert_pem_start[] asm("_binary_{filename}_start");
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "keys/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "keys/client.key" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "keys/AmazonRootCA1.pem" TEXT)
# target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "keys/AmazonPresignedS3CA.pem" TEXT)