project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

# Uncomment the next line to use version 0.14 of uWebSockets 
#add_definitions(-DUWS_0_14_X)

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/path_planner.cpp src/ptg.cpp src/helpers.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

include_directories("C:/Program\ Files\ (x86)/vcpkg/installed/x86-windows/include")  # At least if using vcpkg
link_directories("C:/Program\ Files\ (x86)/vcpkg/installed/x86-windows/include/openssl")  # At least if using vcpkg
link_directories("C:/Program\ Files\ (x86)/vcpkg/installed/x86-windows/lib")  # At least if using vcpkg

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

add_executable(path_planning ${sources})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
  # At least if using vcpkg
  target_link_libraries(path_planning zlib ssleay32 libuv uWS)

else(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
  target_link_libraries(path_planning z ssl uv uWS)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 