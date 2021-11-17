project(communication)
set(MINILOAD_LIB_NAME miniload_core_communication)

link_directories(lib)

add_library(${MINILOAD_LIB_NAME} SHARED
        src/can_drive.cpp
        )
link_directories(${MINILOAD_LIB_NAME}
        ${PROJECT_SOURCE_DIR}/include
        )
target_link_libraries(${MINILOAD_LIB_NAME}
        ${PROJECT_SOURCE_DIR}/lib/libCanCmd.so
        )

ament_export_include_directories(include)

install(
        TARGETS ${MINILOAD_LIB_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY lib/ DESTINATION lib)

