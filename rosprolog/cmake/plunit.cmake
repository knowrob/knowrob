_generate_function_if_testing_is_disabled("catkin_add_plunit")

set(PLUNIT_DIR "${CMAKE_CURRENT_LIST_DIR}") 

#
# Add a custom plunit target that will be called
# when the *run_tests* target is called.
# The function accepts a list of arguments where
# each is a path to a test module on the filesystem
# relative to the *prolog* subdir of the ROS package --
# for example, the arg should be 'knowrob/test' if the
# test module is located at 'prolog/knowrob/test.plt'.
#
function(catkin_add_plunit)
  _warn_if_skip_testing("catkin_add_plunit")
  
  get_filename_component(pkg_name ${CMAKE_BINARY_DIR} NAME)
  set(plunit "${PLUNIT_DIR}/../scripts/rosprolog-test")
  
  foreach(arg IN LISTS ARGN)
    string(REPLACE "/" "_" arg_clean ${arg})
    set(unitid "${pkg_name}_${arg_clean}")
    set(xunit "plunit-${unitid}.xml")
    set(test_dir "${CMAKE_BINARY_DIR}/test_results/${pkg_name}")
    file(MAKE_DIRECTORY ${test_dir})
    
    catkin_run_tests_target(
        "plunit" "${unitid}" "${xunit}"
        COMMAND "${plunit} -o \"${xunit}\" \"${pkg_name}:${arg}\""
        WORKING_DIRECTORY "${test_dir}"
    )
  endforeach()
endfunction()
