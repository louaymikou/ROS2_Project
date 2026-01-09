// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interfaces:action/NavigateToAruco.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
#include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
#include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


// Include directives for member types
// Member `color_choice`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_Goal__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_member_array[3] = {
  {
    "target_aruco_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Goal, target_aruco_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "return_to_zero",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Goal, return_to_zero),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "color_choice",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Goal, color_choice),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_Goal",  // message name
  3,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_Goal),
  custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_Goal)() {
  if (!custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_Goal__rosidl_typesupport_introspection_c__NavigateToAruco_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_Result__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_member_array[6] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_aruco_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Result, final_aruco_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "went_forward",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Result, went_forward),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "navigation_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Result, navigation_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_traveled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Result, distance_traveled),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "returned_to_zero",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Result, returned_to_zero),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_Result",  // message name
  6,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_Result),
  custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_Result)() {
  if (!custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_Result__rosidl_typesupport_introspection_c__NavigateToAruco_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


// Include directives for member types
// Member `current_direction`
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_Feedback__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_member_array[6] = {
  {
    "current_aruco_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Feedback, current_aruco_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_direction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Feedback, current_direction),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "elapsed_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Feedback, elapsed_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Feedback, status_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obstacle_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Feedback, obstacle_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obstacle_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_Feedback, obstacle_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_Feedback",  // message name
  6,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_Feedback),
  custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_Feedback)() {
  if (!custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_Feedback__rosidl_typesupport_introspection_c__NavigateToAruco_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "custom_interfaces/action/navigate_to_aruco.h"
// Member `goal`
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_SendGoal_Request__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_SendGoal_Request),
  custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_SendGoal_Request)() {
  custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_Goal)();
  if (!custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_SendGoal_Request__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_SendGoal_Response__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_SendGoal_Response),
  custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_SendGoal_Response)() {
  custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_SendGoal_Response__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_service_members = {
  "custom_interfaces__action",  // service namespace
  "NavigateToAruco_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_service_type_support_handle = {
  0,
  &custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_SendGoal)() {
  if (!custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_service_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_SendGoal_Response)()->data;
  }

  return &custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_GetResult_Request__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_GetResult_Request),
  custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_GetResult_Request)() {
  custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_GetResult_Request__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "custom_interfaces/action/navigate_to_aruco.h"
// Member `result`
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_GetResult_Response__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_GetResult_Response),
  custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_GetResult_Response)() {
  custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_Result)();
  if (!custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_GetResult_Response__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_service_members = {
  "custom_interfaces__action",  // service namespace
  "NavigateToAruco_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_service_type_support_handle = {
  0,
  &custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_GetResult)() {
  if (!custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_service_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_GetResult_Response)()->data;
  }

  return &custom_interfaces__action__detail__navigate_to_aruco__rosidl_typesupport_introspection_c__NavigateToAruco_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__functions.h"
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "custom_interfaces/action/navigate_to_aruco.h"
// Member `feedback`
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__NavigateToAruco_FeedbackMessage__init(message_memory);
}

void custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_fini_function(void * message_memory)
{
  custom_interfaces__action__NavigateToAruco_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__NavigateToAruco_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_members = {
  "custom_interfaces__action",  // message namespace
  "NavigateToAruco_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__NavigateToAruco_FeedbackMessage),
  custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_member_array,  // message members
  custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_type_support_handle = {
  0,
  &custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_FeedbackMessage)() {
  custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, NavigateToAruco_Feedback)();
  if (!custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__action__NavigateToAruco_FeedbackMessage__rosidl_typesupport_introspection_c__NavigateToAruco_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
