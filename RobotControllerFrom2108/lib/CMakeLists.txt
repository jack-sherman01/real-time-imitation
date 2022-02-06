PROJECT(CmpRIK)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(CMAKE_BUILD_TYPE Development)

set(INCLUDE include)
set(CONFIG config)
set(UTIL ../Util)

include_directories(
    ${INCLUDE}
    ${config}
    ${UTIL}
)

aux_source_directory(src DIR_SRCS)
aux_source_directory(src/Simulation DIR_SRCS)
aux_source_directory(src/Robots DIR_SRCS)
aux_source_directory(src/Interface DIR_SRCS)
aux_source_directory(src/PathGenerator DIR_SRCS)

add_library(${PROJECT_NAME} SHARED ${DIR_SRCS})

TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    pthread
    glog
    rt
)
