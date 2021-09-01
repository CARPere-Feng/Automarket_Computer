set(MINILOAD_LIB_NAME miniload_core_communication)

link_directories(lib)

add_library(${MINILOAD_LIB_NAME} SHARED
        src/can_drive.cpp
        )

target_link_libraries(${MINILOAD_LIB_NAME}
        CanCmd
        )

install(
        TARGETS ${MINILOAD_LIB_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
