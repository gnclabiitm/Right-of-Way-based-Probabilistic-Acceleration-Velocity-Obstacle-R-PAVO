# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "move_robot: 3 messages, 3 services")

set(MSG_I_FLAGS "-Imove_robot:/home/vivek/catkin_ws/src/move_robot/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(move_robot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg" NAME_WE)
add_custom_target(_move_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_robot" "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg" ""
)

get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg" NAME_WE)
add_custom_target(_move_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_robot" "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg" ""
)

get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg" NAME_WE)
add_custom_target(_move_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_robot" "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg" ""
)

get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv" NAME_WE)
add_custom_target(_move_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_robot" "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv" ""
)

get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv" NAME_WE)
add_custom_target(_move_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_robot" "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv" ""
)

get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv" NAME_WE)
add_custom_target(_move_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move_robot" "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
)
_generate_msg_cpp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
)
_generate_msg_cpp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
)

### Generating Services
_generate_srv_cpp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
)
_generate_srv_cpp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
)
_generate_srv_cpp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
)

### Generating Module File
_generate_module_cpp(move_robot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(move_robot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(move_robot_generate_messages move_robot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_cpp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_cpp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_cpp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_cpp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_cpp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_cpp _move_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_robot_gencpp)
add_dependencies(move_robot_gencpp move_robot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_robot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
)
_generate_msg_eus(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
)
_generate_msg_eus(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
)

### Generating Services
_generate_srv_eus(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
)
_generate_srv_eus(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
)
_generate_srv_eus(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
)

### Generating Module File
_generate_module_eus(move_robot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(move_robot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(move_robot_generate_messages move_robot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_eus _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_eus _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_eus _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_eus _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_eus _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_eus _move_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_robot_geneus)
add_dependencies(move_robot_geneus move_robot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_robot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
)
_generate_msg_lisp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
)
_generate_msg_lisp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
)

### Generating Services
_generate_srv_lisp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
)
_generate_srv_lisp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
)
_generate_srv_lisp(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
)

### Generating Module File
_generate_module_lisp(move_robot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(move_robot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(move_robot_generate_messages move_robot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_lisp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_lisp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_lisp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_lisp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_lisp _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_lisp _move_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_robot_genlisp)
add_dependencies(move_robot_genlisp move_robot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_robot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
)
_generate_msg_nodejs(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
)
_generate_msg_nodejs(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
)

### Generating Services
_generate_srv_nodejs(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
)
_generate_srv_nodejs(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
)
_generate_srv_nodejs(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
)

### Generating Module File
_generate_module_nodejs(move_robot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(move_robot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(move_robot_generate_messages move_robot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_nodejs _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_nodejs _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_nodejs _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_nodejs _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_nodejs _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_nodejs _move_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_robot_gennodejs)
add_dependencies(move_robot_gennodejs move_robot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_robot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
)
_generate_msg_py(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
)
_generate_msg_py(move_robot
  "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
)

### Generating Services
_generate_srv_py(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
)
_generate_srv_py(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
)
_generate_srv_py(move_robot
  "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
)

### Generating Module File
_generate_module_py(move_robot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(move_robot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(move_robot_generate_messages move_robot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/las_mes.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_py _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_py _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/msg/mission_plot_data.msg" NAME_WE)
add_dependencies(move_robot_generate_messages_py _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/xycoord.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_py _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/OGM.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_py _move_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vivek/catkin_ws/src/move_robot/srv/SyncMotion.srv" NAME_WE)
add_dependencies(move_robot_generate_messages_py _move_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_robot_genpy)
add_dependencies(move_robot_genpy move_robot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_robot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move_robot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(move_robot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move_robot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(move_robot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move_robot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(move_robot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move_robot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(move_robot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move_robot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(move_robot_generate_messages_py std_msgs_generate_messages_py)
endif()
