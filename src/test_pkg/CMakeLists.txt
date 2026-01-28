cmake_minimum_required(VERSION 3.20)
project(test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  PROGRAMS
    scripts/experiment.py
    scripts/evaluation.py
    scripts/structure.py
    scripts/magnetism.py
    scripts/output_plot.py
    scripts/torque_map.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    data/C1.csv
    data/C2.csv
    data/C3.csv
    data/C4.csv
    data/C5.csv
    data/C6.csv
    data/C7.csv
    data/C8.csv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
