#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "arm_2026::arm_2026" for configuration ""
set_property(TARGET arm_2026::arm_2026 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(arm_2026::arm_2026 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libarm_2026.so"
  IMPORTED_SONAME_NOCONFIG "libarm_2026.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS arm_2026::arm_2026 )
list(APPEND _IMPORT_CHECK_FILES_FOR_arm_2026::arm_2026 "${_IMPORT_PREFIX}/lib/libarm_2026.so" )

# Import target "arm_2026::arm_2026_keyboard_teleop" for configuration ""
set_property(TARGET arm_2026::arm_2026_keyboard_teleop APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(arm_2026::arm_2026_keyboard_teleop PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/arm_2026/arm_2026_keyboard_teleop"
  )

list(APPEND _IMPORT_CHECK_TARGETS arm_2026::arm_2026_keyboard_teleop )
list(APPEND _IMPORT_CHECK_FILES_FOR_arm_2026::arm_2026_keyboard_teleop "${_IMPORT_PREFIX}/lib/arm_2026/arm_2026_keyboard_teleop" )

# Import target "arm_2026::arm_2026_joystick_teleop" for configuration ""
set_property(TARGET arm_2026::arm_2026_joystick_teleop APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(arm_2026::arm_2026_joystick_teleop PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/arm_2026/arm_2026_joystick_teleop"
  )

list(APPEND _IMPORT_CHECK_TARGETS arm_2026::arm_2026_joystick_teleop )
list(APPEND _IMPORT_CHECK_FILES_FOR_arm_2026::arm_2026_joystick_teleop "${_IMPORT_PREFIX}/lib/arm_2026/arm_2026_joystick_teleop" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
