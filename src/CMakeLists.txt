# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
        SRCS ${app_sources}
)

target_add_binary_data(${COMPONENT_TARGET} "web/index.html.gz" BINARY)
target_add_binary_data(${COMPONENT_TARGET} "web/config.html.gz" BINARY)
