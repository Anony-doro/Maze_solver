cmake_minimum_required(VERSION 3.10)
project(MazeGenerator)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

if (MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
else()
    # For MinGW, add missing C++11 features
    add_compile_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()


add_executable(BicycleNavigation
        main_bicycle_navigation.cpp
        SimpleMaze10x10.cpp
        SimpleMaze10x10.hpp
        MazeRouter100x100.cpp
        MazeRouter100x100.hpp
        BicycleTemp.cpp
        BicycleTemp.hpp
)


target_include_directories(BicycleNavigation PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(BicycleNavigation PRIVATE ${OpenCV_LIBS})

