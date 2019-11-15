cmake_minimum_required(VERSION 2.8)
project(onboard)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
set (PLATFORM_LIBS pthread ${CMAKE_DL_LIBS})
#add_definitions("-std=c++11 -Wall -Wextra -Werror -Wno-psabi")
add_definitions("-std=c++11 -Wall -Wextra -Wno-psabi")
find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(include)
include_directories(KCF)

aux_source_directory(./src MAIN_SRCS)
aux_source_directory(./KCF KCF_SRCS)

add_executable( onboard ${MAIN_SRCS} ${KCF_SRCS})
target_link_libraries( onboard ${OpenCV_LIBS} ${PLATFORM_LIBS} 
                        MAVSDK::mavsdk_action
                        MAVSDK::mavsdk_offboard
                        MAVSDK::mavsdk_telemetry
                        MAVSDK::mavsdk
                        )
