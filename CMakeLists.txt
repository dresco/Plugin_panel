add_library(panel INTERFACE)

target_sources(panel INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/panel.c
)

target_include_directories(panel INTERFACE ${CMAKE_CURRENT_LIST_DIR})
