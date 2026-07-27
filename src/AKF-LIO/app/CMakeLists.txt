add_executable(run_mapping_online run_mapping_online.cc)

# Suppress warnings for this target
target_compile_options(run_mapping_online PRIVATE
    -Wno-deprecated-declarations
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-sign-compare
    -Wno-unused-result
    -Wno-maybe-uninitialized
)

target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        )

