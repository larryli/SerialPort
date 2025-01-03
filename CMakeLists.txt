cmake_minimum_required(VERSION 3.5.0)
project(SerialPort VERSION 0.1.0 LANGUAGES C)

if(MSVC)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /MANIFEST:NO")
    add_definitions(/D_CRT_SECURE_NO_WARNINGS)
endif()

add_library(SerialPort SerialPort.c)
target_link_libraries(SerialPort comctl32 comdlg32)
