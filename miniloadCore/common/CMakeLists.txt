project(common)

set(MINILOAD_LIB_NAME miniload_core_common)

add_library(${MINILOAD_LIB_NAME} SHARED
        src/data_type.cpp include/miniloadCore/common/data_type.h include/miniloadCore/common/shelf_pos_config.h)

install(DIRECTORY include/ DESTINATION include)
