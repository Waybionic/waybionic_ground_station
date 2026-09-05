cmake_minimum_required(VERSION 3.8)
project(waybionic_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_full_arm_model
    test/test_full_arm_model.py
    TIMEOUT 60
  )
endif()

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Run the mechanical drop's own URDF, unmodified.
#
# urdf/full-arm-smaller.urdf is the 2026-09-05 export byte-for-byte - same
# md5 as sep_05_latest_info/full-arm-smaller.urdf. It resolves its meshes as
#   package://full-arm-smaller/meshes/<Name With Spaces>.STL
# which needs two things that do not exist on their own here:
#
#   1. a package called "full-arm-smaller". The name is invalid for an ament
#      package (REP-144 forbids hyphens), so we cannot create one - but
#      package:// only needs the ament index to name a prefix. Dropping a
#      marker in this package's resource_index makes the lookup succeed and
#      point at our own share/ directory.
#   2. the meshes under their delivered names, spaces and all. Our meshes/ is
#      snake_case, so install a second, renamed copy for the export to find.
#      Same files - no duplication on disk beyond the install tree.
#
# Nothing here modifies the export. Delete this block and the export stops
# resolving, but the two derived models are unaffected.
# ---------------------------------------------------------------------------
set(_EXPORT_PKG "full-arm-smaller")

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${_EXPORT_PKG}" "")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${_EXPORT_PKG}"
  DESTINATION share/ament_index/resource_index/packages
)

# <snake_case source> -> "<delivered name>"
set(_EXPORT_MESHES
  "bottom_base_assembly.STL"  "bottom base assembly.STL"
  "sweep.STL"                 "Sweep.STL"
  "shouldersplit.STL"         "shouldersplit.STL"
  "third_joint_bend.STL"      "3rd joint bend.STL"
  "diff_assembly_pulley.STL"  "diff-assembly-pulley.STL"
  "biomed_lock_mech.STL"      "biomed lock mech.STL"
  "stepper.STL"               "Stepper.STL"
  "nema23.STL"                "nema23.STL"
  "outerring.STL"             "outerring.STL"
  "bevel_gear.STL"            "bevel gear.STL"
  "heat_set_insert.STL"       "heatSetInsert.STL"
  "m3.STL"                    "m3.STL"
  "Full Arm Smaller.STL"      "Full Arm Smaller.STL"
)
list(LENGTH _EXPORT_MESHES _n)
math(EXPR _last "${_n} / 2 - 1")
foreach(_i RANGE ${_last})
  math(EXPR _src_i "${_i} * 2")
  math(EXPR _dst_i "${_src_i} + 1")
  list(GET _EXPORT_MESHES ${_src_i} _src)
  list(GET _EXPORT_MESHES ${_dst_i} _dst)
  install(FILES "meshes/${_src}"
    DESTINATION "share/${_EXPORT_PKG}/meshes"
    RENAME "${_dst}"
  )
endforeach()

ament_package()
