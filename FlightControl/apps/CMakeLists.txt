file(GLOB_RECURSE APP "*.cpp" "*.c")
add_library(app_lib STATIC ${APP})
target_include_directories(app_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Tasks
)
target_link_libraries(app_lib PUBLIC 
    stm32cubemx
    middleware_lib
    drivers_lib
)