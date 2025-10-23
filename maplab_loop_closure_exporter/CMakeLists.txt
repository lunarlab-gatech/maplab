cmake_minimum_required (VERSION 2.8)
project(maplab_loop_closure_exporter)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# This is the key change.
# It tells the compiler to look for headers in our package's
# main folder, so it can find "json.hpp".
include_directories(
  ${PROJECT_SOURCE_DIR}
)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/loop-closure-exporter-plugin.cc)
create_console_plugin(${PROJECT_NAME})

cs_install()
cs_export()