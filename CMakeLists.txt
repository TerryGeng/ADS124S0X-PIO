cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_build C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pio_adc)

pico_generate_pio_header(pio_adc ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

# by default the header is generated into the build dir
#pico_generate_pio_header(adc ${CMAKE_CURRENT_LIST_DIR}/adc.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_blink ${CMAKE_CURRENT_LIST_DIR}/blink.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

target_sources(pio_adc PRIVATE pio_adc.c spi_odm.c ads124s0x_pio.c)

target_link_libraries(pio_adc PRIVATE pico_stdlib hardware_pio hardware_dma hardware_spi)
pico_add_extra_outputs(pio_adc)
