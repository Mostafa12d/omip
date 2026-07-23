# pybind11 bindings for omip_core (Phase 5). Produces a Python extension
# module importable as `import omip_core`.
#
# The CMake *target* name is deliberately not "omip_core" (that name is
# already taken by the C++ static library defined in the parent
# CMakeLists.txt) — pybind11_add_module()'s first argument becomes both the
# CMake target name and, by default, the compiled module's internal name.
# We decouple the two: keep the CMake target named `omip_core_python`, but
# override its OUTPUT_NAME so the resulting shared object is still
# `omip_core.<abi-tag>.so`, loadable from Python as `import omip_core`.
pybind11_add_module(omip_core_python omip_core_py.cpp)

set_target_properties(omip_core_python PROPERTIES
  OUTPUT_NAME omip_core
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
)

target_link_libraries(omip_core_python PRIVATE omip_core ${OpenCV_LIBS})

# Phase 5 exit-criterion smoke test: `import omip_core` + round-trip a
# synthetic frame through all 3 stages. Requires numpy in Python3's
# environment; skip (not fail) the whole build if it's missing, since it's
# a smoke test, not a golden-fixture correctness check like tests/.
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import numpy"
  RESULT_VARIABLE OMIP_CORE_NUMPY_CHECK
  OUTPUT_QUIET ERROR_QUIET
)
if(OMIP_CORE_NUMPY_CHECK EQUAL 0)
  add_test(
    NAME test_bindings_smoke
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_bindings_smoke.py"
  )
  set_tests_properties(test_bindings_smoke PROPERTIES
    ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:omip_core_python>"
  )
else()
  message(STATUS "numpy not found for ${Python3_EXECUTABLE} — skipping test_bindings_smoke (module still builds).")
endif()
