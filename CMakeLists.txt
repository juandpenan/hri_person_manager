cmake_minimum_required(VERSION 3.5)
project(hri_person_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hri REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hri_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(backward_ros REQUIRED)


set(dependencies
    rclcpp
    hri  
    tf2
    tf2_ros
    hri_msgs
    std_srvs
    backward_ros   
  )

include_directories(include)

# include_directories(
#     SYSTEM
#     ${catkin_INCLUDE_DIRS}
#     ${Boost_INCLUDE_DIRS}
# )

add_library(${PROJECT_NAME} SHARED
  src/hri_person_manager/managed_person.cpp
  src/hri_person_manager/person_matcher.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})



add_executable(hri_person_manager_node src/hri_person_manager.cpp)
ament_target_dependencies(hri_person_manager_node ${dependencies})
target_link_libraries(hri_person_manager_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  hri_person_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

add_compile_options(-g)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(tests)
# endif()
ament_package()
