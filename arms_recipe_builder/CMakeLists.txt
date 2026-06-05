cmake_minimum_required(VERSION 3.8)
project(arms_recipe_builder)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS arms_recipe_builder/recipe_builder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME recipe_builder_node
)

install(PROGRAMS arms_recipe_builder/recipe_ui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME recipe_ui
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
