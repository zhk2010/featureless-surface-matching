cmake_minimum_required(VERSION 3.22.1)

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
foreach(subdir ${SUBDIRS})
    set(HEADER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/include/)
    include_directories(${HEADER_INCLUDE_DIR})
    LIST(APPEND includedirlist ${HEADER_INCLUDE_DIR})
    set(HEADER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/include/impl/)
    include_directories(${HEADER_INCLUDE_DIR})
    LIST(APPEND includedirlist ${HEADER_INCLUDE_DIR})
endforeach()

add_subdirectory(common)
add_subdirectory(osg)
add_subdirectory(matcher)
