set(MINILOAD_LIB_NAME miniload_core_examples)

add_executable(${MINILOAD_LIB_NAME}
        main.cpp
        )
ament_target_dependencies(${MINILOAD_LIB_NAME}
        rclcpp
        )
target_link_libraries(${MINILOAD_LIB_NAME}
        miniload_core_communication
        miniload_core_motor
        ${rclcpp_LIBRIRAES}
        )

#install(DIRECTORY include/ DESTINATION include)