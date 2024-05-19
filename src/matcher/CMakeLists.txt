project(pcle_matcher)

file(GLOB MATCHER_SRC 
                      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
                      ${CMAKE_CURRENT_SOURCE_DIR}/src/impl/*.cpp
                      )

add_library(pcle_matcher SHARED ${MATCHER_SRC})

target_link_libraries(pcle_matcher
    pcle_osg
    pcle_common
    glog
)
