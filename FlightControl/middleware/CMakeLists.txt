file(GLOB_RECURSE MIDDLEWARE_SOURCES "*.cpp" "*.c")
add_library(middleware_lib STATIC ${MIDDLEWARE_SOURCES})
target_include_directories(middleware_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Bridge
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter
    ${CMAKE_CURRENT_SOURCE_DIR}/PID
)