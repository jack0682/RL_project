# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:soma_cube_rl_bridge__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:soma_cube_rl_bridge__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:soma_cube_rl_bridge__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:soma_cube_rl_bridge__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:soma_cube_rl_bridge__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:soma_cube_rl_bridge__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:soma_cube_rl_bridge__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:soma_cube_rl_bridge__rosidl_typesupport_cpp;__rosidl_generator_py:soma_cube_rl_bridge__rosidl_generator_py")

# populate soma_cube_rl_bridge_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "soma_cube_rl_bridge::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'soma_cube_rl_bridge' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND soma_cube_rl_bridge_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
