# Minimal Pico SDK import shim.
# Set PICO_SDK_PATH in your shell, or pass -DPICO_SDK_PATH=/path/to/pico-sdk.

if (NOT PICO_SDK_PATH)
    if (DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    endif()
endif()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR
        "PICO_SDK_PATH is not set. Set it to your pico-sdk directory, e.g.\n"
        "  export PICO_SDK_PATH=$HOME/pico-sdk")
endif()

get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
