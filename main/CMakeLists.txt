idf_component_register(
    SRCS "main.c" 
         "cli_task.c" 
         "FreeRTOS_CLI.c"
         "${CMAKE_CURRENT_LIST_DIR}/../components/protocol_examples_common/connect.c"
    INCLUDE_DIRS "."
)
