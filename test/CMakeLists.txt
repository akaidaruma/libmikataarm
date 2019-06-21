

set(example01_srcs mikata_move_example01.cpp)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libdynamixel/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libarm/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libdynamixel/libaqua/include)

find_package(Threads)

set(ADDITIONAL_LIBRARY ${PROJECT_NAME} dynamixel)
add_executable(mikata_move_example01 ${example01_srcs})
target_link_libraries(mikata_move_example01 ${CMAKE_THREAD_LIBS_INIT} ${ADDITIONAL_LIBRARY})

