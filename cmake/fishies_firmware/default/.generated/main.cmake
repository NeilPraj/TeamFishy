include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(fishies_firmware_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(fishies_firmware_default_default_XC8_FILE_TYPE_assemble)
add_library(fishies_firmware_default_default_XC8_assemble OBJECT ${fishies_firmware_default_default_XC8_FILE_TYPE_assemble})
    fishies_firmware_default_default_XC8_assemble_rule(fishies_firmware_default_default_XC8_assemble)
    list(APPEND fishies_firmware_default_library_list "$<TARGET_OBJECTS:fishies_firmware_default_default_XC8_assemble>")

endif()

# Handle files with suffix S, for group default-XC8
if(fishies_firmware_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(fishies_firmware_default_default_XC8_assemblePreprocess OBJECT ${fishies_firmware_default_default_XC8_FILE_TYPE_assemblePreprocess})
    fishies_firmware_default_default_XC8_assemblePreprocess_rule(fishies_firmware_default_default_XC8_assemblePreprocess)
    list(APPEND fishies_firmware_default_library_list "$<TARGET_OBJECTS:fishies_firmware_default_default_XC8_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group default-XC8
if(fishies_firmware_default_default_XC8_FILE_TYPE_compile)
add_library(fishies_firmware_default_default_XC8_compile OBJECT ${fishies_firmware_default_default_XC8_FILE_TYPE_compile})
    fishies_firmware_default_default_XC8_compile_rule(fishies_firmware_default_default_XC8_compile)
    list(APPEND fishies_firmware_default_library_list "$<TARGET_OBJECTS:fishies_firmware_default_default_XC8_compile>")

endif()

# Handle files with suffix elf, for group default-XC8
if(fishies_firmware_default_default_XC8_FILE_TYPE_objcopy_lss)
add_library(fishies_firmware_default_default_XC8_objcopy_lss OBJECT ${fishies_firmware_default_default_XC8_FILE_TYPE_objcopy_lss})
    fishies_firmware_default_default_XC8_objcopy_lss_rule(fishies_firmware_default_default_XC8_objcopy_lss)
    list(APPEND fishies_firmware_default_library_list "$<TARGET_OBJECTS:fishies_firmware_default_default_XC8_objcopy_lss>")

endif()


# Main target for this project
add_executable(fishies_firmware_default_image_qQbP98cg ${fishies_firmware_default_library_list})

set_target_properties(fishies_firmware_default_image_qQbP98cg PROPERTIES
    OUTPUT_NAME "default"
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES "${output_extensions}"
    RUNTIME_OUTPUT_DIRECTORY "${fishies_firmware_default_output_dir}")
target_link_libraries(fishies_firmware_default_image_qQbP98cg PRIVATE ${fishies_firmware_default_default_XC8_FILE_TYPE_link})
# Add the link options from the rule file.
fishies_firmware_default_link_rule( fishies_firmware_default_image_qQbP98cg)


#Add objcopy steps
fishies_firmware_default_objcopy_lss_rule(fishies_firmware_default_image_qQbP98cg)

