# MazeLib/cmake/toolchain_arm-none-eabi_cm4.cmake
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs" CACHE INTERNAL "")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

unset(CMAKE_CXX_FLAGS CACHE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -fno-use-cxa-atexit -fno-exceptions -fno-rtti -fno-threadsafe-statics -ffunction-sections -fdata-sections -mslow-flash-data" CACHE STRING "" FORCE)
