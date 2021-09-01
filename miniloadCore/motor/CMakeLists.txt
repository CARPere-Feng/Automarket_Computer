set(MINILOAD_LIB_NAME miniload_core_motor)

add_library(${MINILOAD_LIB_NAME} SHARED
        src/motor_control.cpp
        )
#ament_target_dependencies(${MINILOAD_LIB_NAME}
#        miniload_core_communication)
target_link_libraries(${MINILOAD_LIB_NAME}
        miniload_core_communication
        CanCmd
        )

install(
        TARGETS ${MINILOAD_LIB_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/ DESTINATION include)