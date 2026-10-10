# CMake build for multiview (alternative to run.sh + mobius).
#
# Targets:
#   multiview-cli         src/perceive + src/front-ends (+ src/cuda)
#   multiview-testcases   src/perceive + src/testcases  (+ src/cuda)
#
# Example:
#   cmake -S . -B build/cmake-release -G Ninja -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_CUDA_ARCHITECTURES=86
#   ninja -C build/cmake-release multiview-cli multiview-testcases
#
# or simply use ../../build/build.sh (see ../../build/environment.yml for the
# dependencies). OpenPose pose detection, Triclops, PCL,
# OpenGL and the GUIs are not supported by this build; they remain
# available through run.sh with the legacy /opt/multiview toolchain.

cmake_minimum_required(VERSION 3.24)
project(multiview LANGUAGES C CXX)

option(MULTIVIEW_WITH_CUDA "Build the CUDA kernels in src/cuda (-DWITH_CUDA)" ON)
option(MULTIVIEW_WITH_AWS "Link the AWS S3 SDK (s3:// paths)" ON)
option(MULTIVIEW_PROFILE_MEMORY "Track allocations (-DPERCEIVE_PROFILE_MEMORY)" OFF)
option(MULTIVIEW_WITH_CCACHE "Use ccache, if found, to cache compilations" ON)
include(CMakeDependentOption)
cmake_dependent_option(MULTIVIEW_WITH_LIBSGM
  "Build contrib/libSGM and enable the GPU SGM disparity strategy (-DUSE_SGM)"
  ON "MULTIVIEW_WITH_CUDA" OFF)
cmake_dependent_option(MULTIVIEW_WITH_HYPERPOSE
  "Build HyperPose + TensorRT pose detection (-DWITH_HYPERPOSE)"
  OFF "MULTIVIEW_WITH_CUDA" OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(MULTIVIEW_WITH_CCACHE)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    # The sloppiness lets ccache cache sources compiled with the PCH
    set(MV_LAUNCHER ${CMAKE_COMMAND} -E env
      CCACHE_SLOPPINESS=pch_defines,time_macros,include_file_mtime,include_file_ctime
      ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${MV_LAUNCHER})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${MV_LAUNCHER})
    set(CMAKE_CUDA_COMPILER_LAUNCHER ${MV_LAUNCHER})
    message(STATUS "multiview: using ccache (${CCACHE_PROGRAM})")
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(MV_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(MV_CONTRIB ${CMAKE_CURRENT_SOURCE_DIR}/../contrib)

# ------------------------------------------------------------------ Packages

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost CONFIG REQUIRED COMPONENTS filesystem)
find_package(OpenCV REQUIRED COMPONENTS
  core imgproc imgcodecs video videoio features2d calib3d ml
  ximgproc xfeatures2d bgsegm aruco
  OPTIONAL_COMPONENTS cudastereo cudawarping)
find_package(assimp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TIRPC REQUIRED IMPORTED_TARGET libtirpc)
find_library(SVM_LIBRARY NAMES svm REQUIRED)
find_path(SVM_INCLUDE_DIR NAMES svm.h PATH_SUFFIXES libsvm REQUIRED)
find_library(NCURSES_LIBRARY NAMES ncurses REQUIRED)

set(MV_DEFS USE_CPU_FLOAT_PACKING BOOST_ERROR_CODE_HEADER_ONLY)
set(MV_LIBS)

if(MULTIVIEW_WITH_AWS)
  find_package(AWSSDK REQUIRED COMPONENTS s3)
  list(APPEND MV_LIBS ${AWSSDK_LINK_LIBRARIES})
else()
  list(APPEND MV_DEFS MULTIVIEW_NO_AWS)
endif()

if(MULTIVIEW_WITH_CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES native)
  endif()
  if(NOT DEFINED CMAKE_CUDA_HOST_COMPILER)
    set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
  endif()
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  find_package(CUDAToolkit REQUIRED)
  list(APPEND MV_DEFS WITH_CUDA)
  list(APPEND MV_LIBS CUDA::cudart)
  # OpenCV's CUDA modules (GPU StereoBM, GPU remap) are optional
  if(OpenCV_cudastereo_FOUND AND OpenCV_cudawarping_FOUND)
    list(APPEND MV_DEFS WITH_OPENCV_CUDA)
    message(STATUS "multiview: OpenCV CUDA modules found (WITH_OPENCV_CUDA)")
  else()
    message(STATUS "multiview: OpenCV has no CUDA modules; "
                   "GPU stereo/remap disabled, project CUDA kernels enabled")
  endif()
endif()

# libSGM: plain-CUDA semi-global matching (no OpenCV CUDA modules needed)
if(MULTIVIEW_WITH_LIBSGM)
  set(SGM_DIR ${MV_CONTRIB}/libSGM)
  configure_file(${SGM_DIR}/include/libsgm_config.h.in
                 ${CMAKE_CURRENT_BINARY_DIR}/libsgm/libsgm_config.h)
  file(GLOB SGM_SRCS ${SGM_DIR}/src/*.cu)
  add_library(mv-sgm STATIC ${SGM_SRCS} ${SGM_DIR}/src/stereo_sgm.cpp)
  target_include_directories(mv-sgm SYSTEM PUBLIC
    ${SGM_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/libsgm)
  target_link_libraries(mv-sgm PUBLIC CUDA::cudart)
  target_compile_options(mv-sgm PRIVATE -w)
  list(APPEND MV_DEFS USE_SGM)
  list(APPEND MV_LIBS mv-sgm)
  message(STATUS "multiview: libSGM GPU stereo enabled (USE_SGM)")
endif()

# HyperPose: TensorRT pose detection. TensorRT comes from
# `build/setup-env.sh --with-tensorrt`; HyperPose is fetched at a pinned
# commit and patched for TensorRT 10 (no UFF models, no cuDNN).
if(MULTIVIEW_WITH_HYPERPOSE)
  set(TENSORRT_ROOT "$ENV{CONDA_PREFIX}/opt/tensorrt" CACHE PATH
    "TensorRT installation (headers in include/, libraries in lib/)")
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS ${TENSORRT_ROOT}/include REQUIRED)
  find_library(NVINFER_LIBRARY nvinfer HINTS ${TENSORRT_ROOT}/lib REQUIRED)
  find_library(NVONNXPARSER_LIBRARY nvonnxparser
    HINTS ${TENSORRT_ROOT}/lib REQUIRED)

  include(FetchContent)
  # SOURCE_SUBDIR has no CMakeLists.txt: fetch only, targets are made here
  FetchContent_Declare(hyperpose
    URL https://github.com/tensorlayer/hyperpose/archive/e34c6acb91144e1d090466324f99c521fbf47cdb.tar.gz
    URL_HASH SHA256=68b439c915dc0aa1a4e71b533afd715abcd669549df528d2b70e3e9ffe5396b3
    PATCH_COMMAND patch -p1 -i ${CMAKE_CURRENT_SOURCE_DIR}/cmake/hyperpose-tensorrt10.patch
    SOURCE_SUBDIR include)
  FetchContent_Declare(stdtensor
    URL https://github.com/stdml/stdtensor/archive/refs/tags/v0.9.1.tar.gz
    URL_HASH SHA256=349b7a2ed69c98a2c0d46f393b8b336ce4b91ecb8b052514b81c306e9085b7f5
    SOURCE_SUBDIR include)
  FetchContent_MakeAvailable(hyperpose stdtensor)

  set(HP_DIR ${hyperpose_SOURCE_DIR})
  file(GLOB HP_PIFPAF_SRCS ${HP_DIR}/src/pifpaf_decoder/*.cpp)
  add_library(mv-hyperpose STATIC
    ${HP_DIR}/src/logging.cpp ${HP_DIR}/src/tensorrt.cpp
    ${HP_DIR}/src/pifpaf.cpp ${HP_PIFPAF_SRCS} ${HP_DIR}/src/paf.cpp
    ${HP_DIR}/src/data.cpp ${HP_DIR}/src/stream.cpp
    ${HP_DIR}/src/thread_pool.cpp ${HP_DIR}/src/pose_proposal.cpp
    ${HP_DIR}/src/human.cpp)
  target_include_directories(mv-hyperpose SYSTEM PUBLIC
    ${HP_DIR}/include ${stdtensor_SOURCE_DIR}/include ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(mv-hyperpose PUBLIC
    ${NVINFER_LIBRARY} ${NVONNXPARSER_LIBRARY} CUDA::cudart ${OpenCV_LIBS})
  target_compile_options(mv-hyperpose PRIVATE -w)
  list(APPEND MV_DEFS WITH_HYPERPOSE)
  list(APPEND MV_LIBS mv-hyperpose)
  message(STATUS "multiview: HyperPose pose detection enabled (WITH_HYPERPOSE)")
endif()

execute_process(
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../bin/multiview-version.sh
  OUTPUT_VARIABLE MV_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT MV_VERSION)
  set(MV_VERSION "unknown")
endif()

# ------------------------------------------------------------------- alglib

file(GLOB ALGLIB_SRCS ${MV_CONTRIB}/alglib/cpp/src/*.cpp)
add_library(mv-alglib STATIC ${ALGLIB_SRCS})
target_compile_options(mv-alglib PRIVATE -w)

# ------------------------------------------------------------------ targets
#
# src/perceive (+ src/cuda) is compiled once, into an OBJECT library shared by
# both executables. The sources that depend on the per-executable
# TESTCASE_BUILD define (k_is_testcase_build) are compiled into each one.

set(MV_PER_TARGET_SRCS
  ${MV_SRC}/perceive/config.cpp
  ${MV_SRC}/perceive/io/struct-meta.cpp)

function(mv_glob out)
  set(files)
  foreach(dir ${ARGN})
    file(GLOB_RECURSE f CONFIGURE_DEPENDS
      ${MV_SRC}/${dir}/*.cpp ${MV_SRC}/${dir}/*.cc
      ${MV_SRC}/${dir}/*.cxx ${MV_SRC}/${dir}/*.c)
    list(APPEND files ${f})
    if(MULTIVIEW_WITH_CUDA)
      file(GLOB_RECURSE f CONFIGURE_DEPENDS ${MV_SRC}/${dir}/*.cu)
      list(APPEND files ${f})
    endif()
  endforeach()
  set(${out} ${files} PARENT_SCOPE)
endfunction()

# Usage requirements common to every multiview target
add_library(mv-common INTERFACE)
target_include_directories(mv-common INTERFACE ${MV_SRC})
target_include_directories(mv-common SYSTEM INTERFACE
  ${MV_SRC}/perceive/contrib
  ${MV_SRC}/perceive/contrib/fmt/include
  ${MV_SRC}/perceive/contrib/range-v3/include
  ${MV_CONTRIB}
  ${MV_CONTRIB}/alglib/include
  ${MV_CONTRIB}/static-math_20180201/include
  ${SVM_INCLUDE_DIR})
target_compile_definitions(mv-common INTERFACE ${MV_DEFS}
  MULTIVIEW_VERSION="${MV_VERSION}"
  INSTALLATION_ROOT="${CMAKE_INSTALL_PREFIX}"
  $<$<BOOL:${MULTIVIEW_PROFILE_MEMORY}>:PERCEIVE_PROFILE_MEMORY>)
target_compile_options(mv-common INTERFACE
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
target_link_libraries(mv-common INTERFACE
  mv-alglib
  ${OpenCV_LIBS}
  Boost::filesystem Boost::headers
  Eigen3::Eigen
  assimp::assimp
  PkgConfig::TIRPC
  ${SVM_LIBRARY} ${NCURSES_LIBRARY}
  ${MV_LIBS}
  Threads::Threads ${CMAKE_DL_LIBS} util m)

# The perceive sources assume "stdinc.hpp" is pre-included (see rules.ninja,
# cpp_pch). Third-party code under contrib/ is compiled without it.
function(mv_add_sources name)
  target_sources(${name} PRIVATE ${ARGN})
  target_precompile_headers(${name} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:${MV_SRC}/stdinc.hpp>)
  foreach(s ${ARGN})
    if(s MATCHES "/contrib/" OR NOT s MATCHES "\\.cpp$")
      set_source_files_properties(${s} TARGET_DIRECTORY ${name}
        PROPERTIES SKIP_PRECOMPILE_HEADERS ON)
    endif()
    if(s MATCHES "/contrib/")
      set_source_files_properties(${s} TARGET_DIRECTORY ${name}
        PROPERTIES COMPILE_OPTIONS "-w")
    endif()
  endforeach()
endfunction()

set(MV_LIB_DIRS perceive)
if(MULTIVIEW_WITH_CUDA)
  list(APPEND MV_LIB_DIRS cuda)
endif()
mv_glob(MV_LIB_SRCS ${MV_LIB_DIRS})
list(REMOVE_ITEM MV_LIB_SRCS ${MV_PER_TARGET_SRCS})
add_library(mv-perceive OBJECT)
target_link_libraries(mv-perceive PUBLIC mv-common)
mv_add_sources(mv-perceive ${MV_LIB_SRCS})

function(mv_executable name)
  mv_glob(srcs ${ARGN})
  add_executable(${name})
  mv_add_sources(${name} ${srcs} ${MV_PER_TARGET_SRCS})
  target_compile_definitions(${name} PRIVATE TARGET="${name}")
  target_link_libraries(${name} PRIVATE mv-perceive)
endfunction()

mv_executable(multiview-cli front-ends)

mv_executable(multiview-testcases testcases)
target_compile_definitions(multiview-testcases PRIVATE TESTCASE_BUILD)
