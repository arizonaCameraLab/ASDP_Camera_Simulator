cmake_minimum_required(VERSION 3.16.0)
project(ASDP_BSP LANGUAGES CXX)

#-----------------------------------------------------------------------------
# Set your favorite C++ standard for compiling the internal files.  The library
# will export things as extern "C" so this will not impact client programs.
# We require C++17 for the use of std::filesystem.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#-----------------------------------------------------------------------------
if(WIN32)
    add_definitions(-DNOMINMAX)
    # Make Windows export all symbols by default, which will create the required .lib file.
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif(WIN32)

#-----------------------------------------------------------------------------
# Local CMake Modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#-----------------------------------------------------------------------------
# Enable sorting projects within the solution on Visual Studio, to group
# Test and Example files together.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#-----------------------------------------------------------------------------
# Checking for optional and required packages

find_package(ASDP_Core_API REQUIRED)
find_package(Doxygen)
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#-----------------------------------------------------------------------------
# Build options.
option(BUILD_TESTS "Build test programs" ON)

if (DOXYGEN_FOUND)
    option(BUILD_DOXYGEN "Build DOxygen documentation" ON)
endif (DOXYGEN_FOUND)

if (BUILD_DOXYGEN)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
endif (BUILD_DOXYGEN)

add_definitions(-DCONFIG_FILE_PATH="${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}")

#-----------------------------------------------------------------------------
# Build the helper static libraries.

set (ImageSource_HEADERS
  ImageSource.h
)
set (ImageSource_SOURCES
  ImageSource.cpp
)
add_library(ImageSource STATIC ${ImageSource_SOURCES} ${ImageSource_HEADERS})
target_include_directories(ImageSource PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

set (Trigger_HEADERS
  Trigger.h
)
set (Trigger_SOURCES
  Trigger.cpp
)
add_library(Trigger STATIC ${Trigger_SOURCES} ${Trigger_HEADERS})
target_link_libraries(Trigger PUBLIC ImageSource ASDP_Core_API::ASDP_Core_API)
target_include_directories(Trigger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

#-----------------------------------------------------------------------------
# Build the shared library to implement ASDP_BSP, which is in the ASDP_BSP_API submodule directory.

set (ASDP_Camera_Simulator_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API/BspInclude.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API/Timing.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API/Temperature.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API/Pose.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API/Camera.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API/UdpInserter.hpp
)

set (ASDP_Camera_Simulator_SOURCES
	ASDP_Camera_Simulator.cpp
)

add_library(ASDP_BSP SHARED ${ASDP_Camera_Simulator_SOURCES} ${ASDP_Camera_Simulator_HEADERS})
target_link_libraries(ASDP_BSP PRIVATE ImageSource Trigger)
target_include_directories(ASDP_BSP PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(ASDP_BSP PROPERTIES PUBLIC_HEADER "${ASDP_Camera_Simulator_HEADERS}")

include(GNUInstallDirs)
install(TARGETS ASDP_BSP EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION bin COMPONENT lib
  LIBRARY DESTINATION lib${LIB_SUFFIX} COMPONENT lib
  ARCHIVE DESTINATION lib${LIB_SUFFIX} COMPONENT lib
  INCLUDES DESTINATION include
  PUBLIC_HEADER DESTINATION include
)

# Install the configuration file from the source directory
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ASDP_Camera_Simulator.conf
  DESTINATION bin
)


# Configure the project for use by client projects using CMake
# This makes the project available as a package for use with find_package()
# and provides the configuration for the build tree.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION 1.0
  COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/ASDP_BSP_API/cmake/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)
install(EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

#-----------------------------------------------------------------------------
# Build tests if we've been asked to.

if(BUILD_TESTS)

  enable_testing()

  set (CPP_TESTS
    test_ImageSource
    test_Trigger
  )
  foreach (BASE ${CPP_TESTS})
    set (APP ${BASE})
    add_executable (${APP} tests_cpp/${BASE}.cpp)
    set_target_properties(${APP} PROPERTIES FOLDER tests/C++)
    target_link_libraries(${APP} PRIVATE
      ASDP_BSP
      ImageSource
      Trigger
    )
    install(TARGETS ${APP} EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION bin
    )
    add_test(${APP} ${APP})
  endforeach (BASE)

endif(BUILD_TESTS)
