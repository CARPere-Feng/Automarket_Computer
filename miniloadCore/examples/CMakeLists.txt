project(examples)

set(MINILOAD_LIB_NAME miniload_core_examples)


add_executable(${MINILOAD_LIB_NAME}
        vel_control.cpp)
ament_target_dependencies(${MINILOAD_LIB_NAME}
        rclcpp
        )
target_link_libraries(${MINILOAD_LIB_NAME}
        miniload_core_communication
        miniload_core_motor
        ${rclcpp_LIBRIRAES}
        )

add_executable(testFeedback
        motorFeedback.cpp
        )
ament_target_dependencies(testFeedback
        rclcpp
        )
target_link_libraries(testFeedback
        miniload_core_communication
        miniload_core_motor
        ${rclcpp_LIBRIRAES}
        )

#install(DIRECTORY include/ DESTINATION include)
