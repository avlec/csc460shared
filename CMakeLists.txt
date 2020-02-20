cmake_minimum_required(VERSION 3.16)

# This file is heavily based off of
# https://github.com/acristoffers/CMake-AVR/blob/master/AssemblyBlink/CMakeLists.txt
# with a little less spice.

set(PROG_TYPE avrisp)
# Configure later \/
set(USBPORT /dev/null)

# AVR Variables
set(MCU atmega2560)
set(F_CPU 16000000)
set(BAUD 9600)
add_definitions(-DF_CPU=${F_CPU})

# Program Names
set(AVRCPP avr-g++)
set(AVRC   avr-gcc)
set(AVRSTRIP avr-strip)
set(OBJCOPY avr-objcopy)
set(OBJDUMP avr-objdump)
set(AVRSIZE avr-size)
set(AVRDUDE avrdude)

# Sets compiler
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CXX_COMPILER ${AVRCPP})
set(CMAKE_C_COMPILER ${AVRC})
set(CMAKE_ASM_COMPILER ${AVRC})

project(cswitchexample C ASM)

set(BASE_PATH "${${PROJECT_NAME}_SOURCE_DIR}")

# Compiler flags
set(CSTANDARD "-std=gnu99")
set(CDEBUG    "-gstabs -g -ggdb")
set(CWARN     "-Wall -Wstrict-prototypes -Wl,--gc-sections -Wl,--relax")
set(CTUNING   "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffunction-sections -fdata-sections")
set(COPT      "-Os -lm -lprintf_flt")
set(CMCU      "-mmcu=${MCU}")
set(CDEFS     "-DF_CPU=${F_CPU} -DBAUD=${BAUD}")

set(CFLAGS   "${CMCU} ${CDEBUG} ${CDEFS} ${COPT} ${CWARN} ${CSTANDARD} ${CTUNING}")
set(CXXFLAGS "${CMCU} ${CDEBUG} ${CDEFS} ${COPT} ${CTUNING}")

set(CMAKE_C_FLAGS   "${CFLAGS}")
set(CMAKE_CXX_FLAGS "${CXXFLAGS}")
set(CMAKE_ASM_FLAGS   "${CFLAGS}")

#add_definitions(-DVOLUNTARY)

add_library(cswitch_lib STATIC cswitch.S)

# Project setup
add_executable(${PROJECT_NAME} cswitch.S shared.c LED_Test.h LED_Test.c)
target_link_libraries(${PROJECT_NAME} cswitch_lib)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}.elf")