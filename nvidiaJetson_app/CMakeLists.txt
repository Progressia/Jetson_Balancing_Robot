# require CMake 3.5 or greater
cmake_minimum_required(VERSION 3.5)

project(app VERSION 0.2 LANGUAGES CXX)

include_directories( ${PROJECT_SOURCE_DIR}
                    )

# OpenCV, jetson-utils, jetson-inference, CUDA
find_package( OpenCV REQUIRED )

# add directory for libnvbuf-utils to program
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

set(LIBRARIES "${OpenCV_LIBS}" )

# compile the app program
add_executable(app 
                uart.cpp
                main.cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

target_compile_options(app PRIVATE -Werror)
target_compile_features(app PRIVATE cxx_std_17)

# link app to jetson-inference library
target_link_libraries(app ${LIBRARIES})
