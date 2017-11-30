cmake_minimum_required( VERSION 2.8 )
project( adrc_test )

set( CMAKE_BUILD_TYPE Release )
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )





ADD_EXECUTABLE( ADRC adrc_test.cpp )
TARGET_LINK_LIBRARIES( ADRC
    ${OpenCV_LIBS})


