vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO Azalea1047/Controller-Toolbox
    REF "v${VERSION}"
    SHA512 0  # Replace with actual SHA512 after first tagged release
    HEAD_REF main
)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        python-bindings CTRL_BUILD_PYTHON_BINDINGS
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DCTRL_BUILD_TESTS=OFF
        -DCTRL_BUILD_BENCHMARKS=OFF
        ${FEATURE_OPTIONS}
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/ControllerToolbox")

# Remove debug include directory (headers are the same for debug and release)
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

# Install license
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
