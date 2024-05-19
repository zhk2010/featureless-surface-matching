project(pcle_common)

#pcl
find_package(PCL ${PCL_VERSION} REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

file(GLOB COMMON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/)

include_directories(${COMMON_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(pcle_common SHARED ${COMMON_SRC})

target_link_libraries(pcle_common
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)