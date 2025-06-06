cmake_minimum_required(VERSION 3.8)
project(lab2_grupo9)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#install Python executables
install(PROGRAMS
	nodes/dead_reckoning_nav.py
	nodes/pose_loader.py
	nodes/p_controller.py
	nodes/p_controller_w.py
 	nodes/pi_controller.py
  	nodes/pi_controller_w.py
   	nodes/wall_dodger_p_controller.py
    	nodes/wall_distance.py
     	nodes/p_navigation_node.py
	nodes/path_reader.py
	nodes/follow_the_carrot.py
	nodes/follower_ctrl.py
	nodes/follower_navigation.py
	nodes/demostracion.py
	DESTINATION lib/${PROJECT_NAME})

#install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(
  	FILES nodes/posiciones.txt
  	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
