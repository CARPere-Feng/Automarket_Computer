project(motor)

set(MINILOAD_LIB_NAME miniload_core_motor)

add_library(${MINILOAD_LIB_NAME} SHARED
        src/motor_control.cpp src/doubleMotor.cpp include/miniloadCore/motor/doubleMotor.h)
ament_target_dependencies(${MINILOAD_LIB_NAME}
        rclcpp
        )
link_directories(${MINILOAD_LIB_NAME}
        ${PROJECT_SOURCE_DIR}/include
        )
target_link_libraries(${MINILOAD_LIB_NAME}
        miniload_core_communication
        ${rclcpp_LIBRIRAES}
        )

ament_export_include_directories(include)

install(
        TARGETS ${MINILOAD_LIB_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/ DESTINATION include)
