cmake_minimum_required(VERSION 3.2)
add_compile_options(-std=c++11)
project(Auction_Algorithm)
SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall") 
set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(
    include
)

#aux_source_directory(src DIR_SRCS)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY libzyre.so)
set(LIBRARY libczmq.so)
set(LIBRARY libzmq.so)
#set(TestPath ${DIR_SRCS})

add_executable(
    TargetOut 
    "${SOURCE_DIR}/src/main.cpp"
    "${SOURCE_DIR}/src/crobot.cpp"
    "${SOURCE_DIR}/src/ctasklist.cpp"
    "${SOURCE_DIR}/src/ctaskpoint.cpp"
    "${SOURCE_DIR}/src/cmultirobotCoordinate.cpp"
)

target_link_libraries(TargetOut pthread)
#add_executable(MyCatchTest "${SOURCE_DIR}/src/MyCatchTest.cpp")
#add_executable(ZYRE_TESTOUT "${SOURCE_DIR}/src/zyre_test.cpp")
#add_executable(ZYRE_JSON_SHOUTEROUT "${SOURCE_DIR}/src/zyre_json_shouter.cpp")
#add_executable(ZYRE_JSON_LISTENEROUT "${SOURCE_DIR}/src/zyre_json_listener.cpp")
#target_link_libraries(ZYRE_JSON_SHOUTEROUT zyre)
#target_link_libraries(ZYRE_JSON_SHOUTEROUT czmq)
#target_link_libraries(ZYRE_JSON_SHOUTEROUT zmq)
#target_link_libraries(ZYRE_JSON_LISTENEROUT zyre)
#target_link_libraries(ZYRE_JSON_LISTENEROUT czmq)
#target_link_libraries(ZYRE_JSON_LISTENEROUT zmq)