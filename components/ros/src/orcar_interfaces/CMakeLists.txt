cmake_minimum_required(VERSION 3.18)
project(orcar_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# ___ PROTOBUF Generation ____
add_library(protocol)
target_include_directories(protocol
  PUBLIC
  ${Protobuf_INCLUDE_DIRS}
)

find_package(Protobuf REQUIRED)
set(PROTO_SOURCES protobuf/Particles.proto)
protobuf_generate_cpp(LIB_SOURCES LIB_HEADERS ${PROTO_SOURCES} )
protobuf_generate_python(PROTO_PY ${PROTO_SOURCES})

target_link_libraries(protocol ${Protobuf_LIBRARIES})
target_sources(protocol
  PRIVATE
  ${LIB_HEADERS}
  ${LIB_SOURCES}
)

add_custom_target(PythonProto ALL
  DEPENDS ${PROTO_PY})

# -DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")  \
# -DPYTHON_LIBRARY=$(python -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))")

# Build ROS Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadioPacket.msg"
  "msg/TaggedRadioPacket.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime Protobuf)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install( FILES ${LIB_HEADERS} DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/${DIR} )

install(TARGETS protocol
  EXPORT protocolTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ament_python_install_module(${PROTO_PY}
#   DESTINATION_SUFFIX ${PROJECT_NAME}
# )

install(FILES ${PROTO_PY} DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_export_libraries(protocol)
ament_export_targets(protocolTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()

