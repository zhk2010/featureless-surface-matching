include_directories(${OUTPUT_DIR_INCLUDE})

SUBDIRLIST(SUBDIRS ${SRC_DIR})
foreach(subdir ${SUBDIRS})
    set(HEADER_INCLUDE_DIR ${SRC_DIR}/${subdir}/include/)
    include_directories(${HEADER_INCLUDE_DIR})
    set(HEADER_INCLUDE_DIR ${SRC_DIR}/${subdir}/include/impl/)
    include_directories(${HEADER_INCLUDE_DIR})
endforeach()

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

add_executable(NFMatch NFMatch.cc)
target_link_libraries(NFMatch
    glog
    ${OpenCV_LIBS}
    pcle_matcher
    pcle_common
    pcle_osg
)