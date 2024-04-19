set(CAN2040_SOURCE can2040/src)
set(CBUSDEFS cbusdefs)
set(CANBLOCK CANBlock)

add_executable(CBUSBlock
   # Source for CAN2040 PIO based CAN controller
   ${CAN2040_SOURCE}/can2040.c
   # Sources for CBUSPico libary 
   CBUSLED.cpp
   CBUSSwitch.cpp
   CBUSConfig.cpp
   CBUSParams.cpp
   SystemTick.cpp
   CBUSCircularBuffer.cpp
   CBUSLongMessage.cpp
   CBUS.cpp
   ACAN2040.cpp
   CBUSACAN2040.cpp
   # CANBlock module using library
   ${CANBLOCK}/CANBlock.cpp
)

target_include_directories(CBUSBlock PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${CAN2040_SOURCE}
   ${CBUSDEFS}
)

target_compile_options(CBUSBlock PRIVATE
   # can2040 has an unused param, -Wpedantic is a step too far !
   -Wall -Wextra -Werror -Wno-unused-parameter 
)

# Custom linker scipt to put CAN2040 code into RAM
pico_set_linker_script(CBUSBlock ${CMAKE_CURRENT_SOURCE_DIR}/memmap_block.ld)

target_link_libraries(
   CBUSBlock
   pico_stdlib
   #pico_stdio_semihosting
   pico_multicore
   cmsis_core
   hardware_i2c
   hardware_flash
)

pico_add_extra_outputs(CBUSBlock)
