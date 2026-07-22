# cnpy: tiny MIT-licensed .npy/.npz reader/writer (github.com/rogersce/cnpy),
# vendored verbatim to read golden-fixture .npz files in tests/ per the
# user's decision recorded in PORTING_NOTES.md (Open Question Q2).

find_package(ZLIB REQUIRED)

add_library(cnpy STATIC cnpy.cpp)
target_include_directories(cnpy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_link_libraries(cnpy PUBLIC ZLIB::ZLIB)
target_compile_features(cnpy PUBLIC cxx_std_14)
