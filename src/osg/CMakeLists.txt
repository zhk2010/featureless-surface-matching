project(pcle_osg)

set(OSG osg osgText osgShadow osgDB osgViewer osgGA OpenThreads osgUtil osgManipulator)
find_package(OpenSceneGraph 3 REQUIRED COMPONENTS ${OSG} REQUIRED)

file(GLOB OSGPLUS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

set(OSGPLUS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_library(pcle_osg SHARED ${OSGPLUS_SRC})

target_include_directories(pcle_osg
    PRIVATE ${OSGPLUS_INCLUDE_DIR}
)

target_link_libraries(pcle_osg
    glog
    pcle_common
    ${OSG}
)
