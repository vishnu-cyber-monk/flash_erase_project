# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.25")

# Project definition
project(
	flash_erase_project
	VERSION 1.0
	LANGUAGES C CXX ASM
)

# Include the definition of the slc_flash_erase_project target,
# which contains the content of the SLC project
include(flash_erase_project.cmake)

add_executable(flash_erase_project
    # Add additional sources here
)

target_include_directories(flash_erase_project PUBLIC
    # Add additional include paths here
)

target_compile_definitions(flash_erase_project PUBLIC
    # Add additional macros here
)

target_compile_options(flash_erase_project PUBLIC
    # Set additional compiler flags here
)

target_link_options(flash_erase_project PUBLIC
    # Set additional linker flags here
)

# Link with the content defined in the SLC project
target_link_libraries(flash_erase_project PRIVATE
    slc
)

# Include managed project content if available
include(flash_erase_project_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(flash_erase_project PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET flash_erase_project
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_SREC_CMD} "$<TARGET_FILE:flash_erase_project>" "$<TARGET_FILE_DIR:flash_erase_project>/$<TARGET_FILE_BASE_NAME:flash_erase_project>.s37"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_IHEX_CMD} "$<TARGET_FILE:flash_erase_project>" "$<TARGET_FILE_DIR:flash_erase_project>/$<TARGET_FILE_BASE_NAME:flash_erase_project>.hex"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_BIN_CMD}  "$<TARGET_FILE:flash_erase_project>" "$<TARGET_FILE_DIR:flash_erase_project>/$<TARGET_FILE_BASE_NAME:flash_erase_project>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET flash_erase_project
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()