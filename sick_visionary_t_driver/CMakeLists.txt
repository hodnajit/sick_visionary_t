cmake_minimum_required(VERSION 3.5)
project(sick_visionary_t_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_executable(sick_visionary_t_driver_node
        src/node.cpp
        src/driver.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

include_directories(
    include
)

ament_target_dependencies(sick_visionary_t_driver_node
    rclcpp
    image_transport
    sensor_msgs
)

target_link_libraries(sick_visionary_t_driver_node
    ${OpenCV_LIBS}
)

install(TARGETS sick_visionary_t_driver_node
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
