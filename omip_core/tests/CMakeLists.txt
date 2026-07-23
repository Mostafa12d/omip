# Golden-fixture test harnesses, one per ported stage. See
# PORTING_NOTES.md's Validation Protocol: each harness runs successfully
# (reporting "ported, unvalidated, awaiting fixtures") even before real
# fixtures land at tests/fixtures/<stage>/.

add_executable(test_feature_tracker test_feature_tracker.cpp)
target_link_libraries(test_feature_tracker PRIVATE omip_core cnpy)
target_compile_features(test_feature_tracker PRIVATE cxx_std_17) # std::filesystem
target_compile_definitions(test_feature_tracker PRIVATE
  OMIP_CORE_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
)
add_test(NAME test_feature_tracker COMMAND test_feature_tracker)

add_executable(test_rb_tracker test_rb_tracker.cpp)
target_link_libraries(test_rb_tracker PRIVATE omip_core cnpy)
target_compile_features(test_rb_tracker PRIVATE cxx_std_17) # std::filesystem
target_compile_definitions(test_rb_tracker PRIVATE
  OMIP_CORE_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
)
add_test(NAME test_rb_tracker COMMAND test_rb_tracker)

add_executable(test_joint_tracker test_joint_tracker.cpp)
target_link_libraries(test_joint_tracker PRIVATE omip_core cnpy)
target_compile_features(test_joint_tracker PRIVATE cxx_std_17) # std::filesystem
target_compile_definitions(test_joint_tracker PRIVATE
  OMIP_CORE_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
)
add_test(NAME test_joint_tracker COMMAND test_joint_tracker)
