# CMakeLists.txt — Android cross-compile for libmujoco.so + libmjaccess.so
#
# Invoked by build.py --mujoco-android via:
#   cmake -S mujoco-unity/NativeShim -B <build_dir>
#         -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake
#         -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-32
#         -DMUJOCO_SRC=<abs-path-to-mujoco-src>
#         -DCMAKE_BUILD_TYPE=Release

cmake_minimum_required(VERSION 3.22)
project(mjaccess_android C CXX)

# ── MuJoCo source path ────────────────────────────────────────────────────────
if(NOT DEFINED MUJOCO_SRC)
  # Default: sibling mujoco-src directory (two levels up from NativeShim)
  get_filename_component(MUJOCO_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco-src" ABSOLUTE)
endif()

if(NOT EXISTS "${MUJOCO_SRC}/CMakeLists.txt")
  message(FATAL_ERROR
    "MuJoCo source not found at '${MUJOCO_SRC}'.\n"
    "Initialize the submodule: git submodule update --init mujoco-src"
  )
endif()

message(STATUS "MuJoCo source: ${MUJOCO_SRC}")

# ── MuJoCo build options — disable everything except core physics ─────────────
set(MUJOCO_BUILD_EXAMPLES    OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS       OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE    OFF CACHE BOOL "" FORCE)
set(MUJOCO_SAMPLES_USE_GLFW  OFF CACHE BOOL "" FORCE)
set(MUJOCO_ENABLE_AVX_INTRINSICS OFF CACHE BOOL "" FORCE)
set(MUJOCO_ENABLE_RPATH      OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING            OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS        ON  CACHE BOOL "" FORCE)

add_subdirectory("${MUJOCO_SRC}" mujoco_build)

# ── libmjaccess.so ────────────────────────────────────────────────────────────
add_library(mjaccess SHARED
  "${CMAKE_CURRENT_SOURCE_DIR}/mjaccess.c"
)

target_include_directories(mjaccess PRIVATE
  "${MUJOCO_SRC}/include"
)

target_link_libraries(mjaccess PRIVATE mujoco)

# Export all MJA_API symbols
set_target_properties(mjaccess PROPERTIES
  C_VISIBILITY_PRESET   default
  OUTPUT_NAME           mjaccess
)
