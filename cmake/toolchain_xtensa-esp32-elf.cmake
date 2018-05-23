# MazeLib/cmake/toolchain_xtensa-esp32-elf.cmake
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_C_COMPILER xtensa-esp32-elf-gcc)
set(CMAKE_CXX_COMPILER xtensa-esp32-elf-g++)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs" CACHE INTERNAL "")
# set(CMAKE_FIND_ROOT_PATH /home/kerikun11/bin/xtensa-esp32-elf/xtensa-esp32-elf)
# set(CMAKE_FIND_ROOT_PATH C:/msys64/opt/xtensa-esp32-elf/xtensa-esp32-elf)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
