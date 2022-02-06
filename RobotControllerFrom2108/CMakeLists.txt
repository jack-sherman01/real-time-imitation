PROJECT(RobotManager)
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

find_package(Boost REQUIRED COMPONENTS system)

# add library
set(BOOST_LIBRARYDIR /usr/lib/i383-linux-gnu/)

set(LIB_INCLUDE lib/include)
set(LIB_CONFIG lib/config)
set(Util_DIR Util/)

# Specify compiled language
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -D_GLIBCXX_USE_NANOSLEEP")

SET(GCC_COVERAGE_LINK_FLAGS "-lrt")

# Sets the included header file, dir
include_directories(
    ${LIB_INCLUDE}
    ${LIB_CONFIG}
    ${Util_DIR}
    /usr/local/include/modbus
)

# Set up the link library search directory
link_directories(/usr/local/lib/)

#add_subdirectory(lib)
add_subdirectory(lib)

# Search all sources(cpp) files
aux_source_directory(. DIR_SRCS)
aux_source_directory(lib/src DIR_SRCS)
aux_source_directory(lib/src/Simulation DIR_SRCS)
aux_source_directory(lib/src/Robots DIR_SRCS)
aux_source_directory(lib/src/Interface DIR_SRCS)
aux_source_directory(lib/src/PathGenerator DIR_SRCS)

#aux_source_directory(lib/include/CmpRIK.cpp DIR_SRCS)

# Generate the executable file
ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})

TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
    ${CMAKE_DL_LIBS}
    ${Boost_LIBRARIES}
    pthread
    glog
    rt
    modbus
)
