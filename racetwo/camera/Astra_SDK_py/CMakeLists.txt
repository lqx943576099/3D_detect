cmake_minimum_required(VERSION 3.10)
project(astra_pybind LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# User can set ASTRA_ROOT via environment or -DASTRA_ROOT=...
# If not set, auto-detect an SDK folder at source dir or next to this CMakeLists.txt that contains include/ and lib/
if(NOT DEFINED ASTRA_ROOT)
  set(PREFERRED_SDK "${CMAKE_SOURCE_DIR}/AstraSDK-v2.1.3-94bca0f52e-20210611T023312Z-Linux-aarch64")
  if(EXISTS "${PREFERRED_SDK}" AND IS_DIRECTORY "${PREFERRED_SDK}")
    set(ASTRA_ROOT ${PREFERRED_SDK})
    message(STATUS "Using preferred Astra SDK at: ${ASTRA_ROOT}")
  else()
    file(GLOB SDK_CAND_REL RELATIVE "${CMAKE_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}/AstraSDK-*-Linux-aarch64")
    if(SDK_CAND_REL)
      list(GET SDK_CAND_REL 0 SDKREL)
      set(ASTRA_ROOT "${CMAKE_SOURCE_DIR}/${SDKREL}")
      message(STATUS "Auto-detected Astra SDK (aarch64) at: ${ASTRA_ROOT}")
    else()
      if(DEFINED ENV{ASTRA_ROOT})
        set(ASTRA_ROOT $ENV{ASTRA_ROOT})
        message(STATUS "Using ASTRA_ROOT from environment: ${ASTRA_ROOT}")
      else()
        file(GLOB CANDIDATES LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/*")
        foreach(dir ${CANDIDATES})
          if(EXISTS "${dir}/include/astra" AND (EXISTS "${dir}/lib/libastra.so" OR EXISTS "${dir}/lib/astra.lib"))
            set(ASTRA_ROOT ${dir})
            message(STATUS "Auto-detected Astra SDK at: ${ASTRA_ROOT}")
            break()
          endif()
        endforeach()
      endif()
    endif()
  endif()
endif()

if(NOT DEFINED ASTRA_ROOT)
  message(FATAL_ERROR "ASTRA_ROOT not set and no SDK auto-detected. Set -DASTRA_ROOT=/path/to/astra-sdk or export ASTRA_ROOT.")
endif()

# 优先从当前虚拟环境选 Python（venv/conda），以保证使用该环境里的 pybind11
if(NOT Python3_EXECUTABLE)
  if(DEFINED ENV{VIRTUAL_ENV} AND EXISTS "$ENV{VIRTUAL_ENV}/bin/python3")
    set(Python3_EXECUTABLE "$ENV{VIRTUAL_ENV}/bin/python3")
    message(STATUS "Python3 from VIRTUAL_ENV: ${Python3_EXECUTABLE}")
  elseif(DEFINED ENV{CONDA_PREFIX} AND EXISTS "$ENV{CONDA_PREFIX}/bin/python3")
    set(Python3_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python3")
    message(STATUS "Python3 from CONDA_PREFIX: ${Python3_EXECUTABLE}")
  endif()
endif()
set(Python3_FIND_VIRTUALENV FIRST)

# 简单校验 ASTRA_ROOT 内容
if(NOT EXISTS "${ASTRA_ROOT}/include/astra/astra.hpp")
  message(FATAL_ERROR "ASTRA_ROOT appears incorrect or missing headers: ${ASTRA_ROOT}/include/astra/astra.hpp not found. Set ASTRA_ROOT to the Astra SDK root (contains include/ and lib/).")
endif()
if(NOT EXISTS "${ASTRA_ROOT}/lib")
  message(FATAL_ERROR "ASTRA_ROOT/lib not found: ${ASTRA_ROOT}/lib")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Get pybind11 headers path from Python (pybind11 must be installed in selected Python env)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import pybind11,sys; sys.stdout.write(pybind11.get_include())"
  OUTPUT_VARIABLE PYBIND11_INCLUDE
  RESULT_VARIABLE _pybind11_res
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT _pybind11_res EQUAL 0 OR PYBIND11_INCLUDE STREQUAL "")
  message(FATAL_ERROR "Failed to query pybind11 include path from Python. Ensure pybind11 is installed in the selected Python environment.")
endif()

# create the python extension module (pybind11 based)
add_library(astra_py MODULE py_astra_camera.cpp)

# target-based include directories
target_include_directories(astra_py PRIVATE
  ${ASTRA_ROOT}/include
  ${PYBIND11_INCLUDE}
  ${Python3_INCLUDE_DIRS}
)

# tell linker where SDK libs live (for linking step)
if(COMMAND target_link_directories)
  target_link_directories(astra_py PRIVATE ${ASTRA_ROOT}/lib)
else()
  link_directories(${ASTRA_ROOT}/lib)
endif()

# 设置 RPATH，使生成的 astra_py.so 在运行时能找到 SDK 的 so（长期方案）
set_target_properties(astra_py PROPERTIES
  PREFIX ""
  SUFFIX ".so"
  BUILD_WITH_INSTALL_RPATH ON
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;${ASTRA_ROOT}/lib"
)

# 链接底层库（按 SDK 提供的名称）
target_link_libraries(astra_py PRIVATE
  astra
  astra_core
  astra_core_api
  ${Python3_LIBRARIES}
)

# 可选：默认 Release 构建
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Configuration summary:")
message(STATUS "  ASTRA_ROOT = ${ASTRA_ROOT}")
message(STATUS "  PYBIND11_INCLUDE = ${PYBIND11_INCLUDE}")
message(STATUS "  Python3 include = ${Python3_INCLUDE_DIRS}")
message(STATUS "  Python3 libs = ${Python3_LIBRARIES}")


# ---------------- Packaging: create a relocatable SDK folder ----------------
add_custom_target(package_sdk ALL
  DEPENDS astra_py
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_BINARY_DIR}/dist
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/dist/astra_py_sdk/python
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/dist/astra_py_sdk/lib
  # copy extension module
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:astra_py> ${CMAKE_BINARY_DIR}/dist/astra_py_sdk/python/
  # copy SDK libs and plugins
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${ASTRA_ROOT}/lib ${CMAKE_BINARY_DIR}/dist/astra_py_sdk/lib
  # ensure astra.toml exists (overwrite with a minimal one if needed)
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ASTRA_ROOT}/lib/astra.toml ${CMAKE_BINARY_DIR}/dist/astra_py_sdk/lib/astra.toml
  COMMENT "Packaging relocatable SDK to dist/astra_py_sdk"
)

add_custom_command(TARGET package_sdk POST_BUILD
  COMMAND ${CMAKE_COMMAND} -DOUT=${CMAKE_BINARY_DIR}/dist/astra_py_sdk/README.txt -P ${CMAKE_SOURCE_DIR}/cmake/write_readme.cmake
)


