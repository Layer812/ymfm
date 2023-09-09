cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_C_FLAGS_RELEASE "-Wall -O3")
#set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(picoaudio C CXX ASM)
pico_sdk_init()

add_executable(picoaudio
    picoaudio.cpp
    tar.cpp
    ../../src/ymfm_adpcm.cpp
    ../../src/ymfm_misc.cpp
    ../../src/ymfm_opl.cpp
    ../../src/ymfm_opm.cpp
    ../../src/ymfm_opn.cpp
    ../../src/ymfm_opq.cpp
    ../../src/ymfm_opz.cpp
    ../../src/ymfm_pcm.cpp
    ../../src/ymfm_ssg.cpp
)
pico_set_program_name(picoaudio "ymfm example for pico audio")
pico_set_program_version(picoaudio "v0.1")
pico_enable_stdio_uart(picoaudio 0)
pico_enable_stdio_usb(picoaudio 0)

target_link_libraries(picoaudio PRIVATE
    pico_stdlib
    pico_divider
    pico_audio_i2s 
    pico_multicore
    hardware_dma
    hardware_pio
    hardware_i2c
    hardware_interp
    hardware_timer
    hardware_clocks
    hardware_pwm
    hardware_flash
    hardware_flash
    hardware_sync)
pico_wrap_function(picoaudio __aeabi_idiv)
pico_wrap_function(picoaudio __aeabi_idivmod)
pico_wrap_function(picoaudio __aeabi_ldivmod)
pico_wrap_function(picoaudio __aeabi_uidiv)
pico_wrap_function(picoaudio __aeabi_uidivmod)
pico_wrap_function(picoaudio __aeabi_uldivmod)

target_include_directories(picoaudio PRIVATE ../../src/ ../vgmrender .)

target_compile_definitions(picoaudio PRIVATE
       # compile time configuration of I2S
       #       PICO_COPY_TO_RAM=1
       PICO_FLASH_SPI_CLKDIV=3
       PICO_AUDIO_I2S_MONO_INPUT=1
       PICO_AUDIO_I2S_MONO_OUTPUT=1
       USE_AUDIO_I2S=1
       PICO_AUDIO_I2S_DATA_PIN=9
       PICO_AUDIO_I2S_CLOCK_PIN_BASE=10
       #Overclocking to 304Mhz
       CORE_CLOCK=304000
       CORE_VOLTAGE=VREG_VOLTAGE_1_30

	)

pico_add_extra_outputs(picoaudio)
