cmake_minimum_required(VERSION 3.25)
project(RailwayCBS)

set(CMAKE_CXX_STANDARD 11)

set(BOOST_ROOT "C:/Boost/boost_1_86_0")
set(Boost_USE_STATIC_LIBS OFF)


find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "." )

add_executable(RailwayCBS main.cpp map.cpp map.h dataStruct.h task.cpp task.h lowLevel.cpp lowLevel.h const.h heuristic.cpp heuristic.h highLevel.cpp highLevel.h)
add_library(tinyxml2 tinyxml2.cpp)
target_link_libraries(RailwayCBS tinyxml2)


if(Boost_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})
endif()


set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
#set(CMAKE_CXX_FLAGS_RELEASE "-g -00")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wuninitialized")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O0")