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
# dependencies). Pose detection (HyperPose/OpenPose), Triclops, libSGM, PCL,
# OpenGL and the GUIs are not supported by this build; they remain
# available through run.sh with the legacy /opt/multiview toolchain.

cmake_minimum_required(VERSION 3.24)
project(multiview LANGUAGES C CXX)

option(MULTIVIEW_WITH_CUDA "Build the CUDA kernels in src/cuda (-DWITH_CUDA)" ON)
option(MULTIVIEW_WITH_AWS "Link the AWS S3 SDK (s3:// paths)" ON)
option(MULTIVIEW_PROFILE_MEMORY "Define PERCEIVE_PROFILE_MEMORY for testcases" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
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

function(mv_executable name)
  set(dirs perceive ${ARGN})
  if(MULTIVIEW_WITH_CUDA)
    list(APPEND dirs cuda)
  endif()
  mv_glob(srcs ${dirs})
  add_executable(${name} ${srcs})
  target_include_directories(${name} PRIVATE ${MV_SRC})
  target_include_directories(${name} SYSTEM PRIVATE
    ${MV_SRC}/perceive/contrib
    ${MV_SRC}/perceive/contrib/fmt/include
    ${MV_SRC}/perceive/contrib/range-v3/include
    ${MV_CONTRIB}
    ${MV_CONTRIB}/alglib/include
    ${MV_CONTRIB}/static-math_20180201/include
    ${SVM_INCLUDE_DIR})
  target_compile_definitions(${name} PRIVATE ${MV_DEFS}
    TARGET="${name}"
    MULTIVIEW_VERSION="${MV_VERSION}"
    INSTALLATION_ROOT="${CMAKE_INSTALL_PREFIX}")
  target_compile_options(${name} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
  # The perceive sources assume "stdinc.hpp" is pre-included (see rules.ninja,
  # cpp_pch). Third-party code under contrib/ is compiled without it.
  target_precompile_headers(${name} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:${MV_SRC}/stdinc.hpp>)
  foreach(s ${srcs})
    if(s MATCHES "/contrib/" OR NOT s MATCHES "\\.cpp$")
      set_source_files_properties(${s} TARGET_DIRECTORY ${name}
        PROPERTIES SKIP_PRECOMPILE_HEADERS ON)
    endif()
    if(s MATCHES "/contrib/")
      set_source_files_properties(${s} TARGET_DIRECTORY ${name}
        PROPERTIES COMPILE_OPTIONS "-w")
    endif()
  endforeach()
  target_link_libraries(${name} PRIVATE
    mv-alglib
    ${OpenCV_LIBS}
    Boost::filesystem Boost::headers
    Eigen3::Eigen
    assimp::assimp
    PkgConfig::TIRPC
    ${SVM_LIBRARY} ${NCURSES_LIBRARY}
    ${MV_LIBS}
    Threads::Threads ${CMAKE_DL_LIBS} util m)
endfunction()

mv_executable(multiview-cli front-ends)

mv_executable(multiview-testcases testcases)
target_compile_definitions(multiview-testcases PRIVATE TESTCASE_BUILD
  $<$<BOOL:${MULTIVIEW_PROFILE_MEMORY}>:PERCEIVE_PROFILE_MEMORY>)
