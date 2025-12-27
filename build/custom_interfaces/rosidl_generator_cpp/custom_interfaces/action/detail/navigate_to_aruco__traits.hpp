// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:action/NavigateToAruco.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__TRAITS_HPP_
#define CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/action/detail/navigate_to_aruco__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_aruco_id
  {
    out << "target_aruco_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_aruco_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_aruco_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_aruco_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_aruco_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_Goal & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_Goal>()
{
  return "custom_interfaces::action::NavigateToAruco_Goal";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_Goal>()
{
  return "custom_interfaces/action/NavigateToAruco_Goal";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: final_aruco_id
  {
    out << "final_aruco_id: ";
    rosidl_generator_traits::value_to_yaml(msg.final_aruco_id, out);
    out << ", ";
  }

  // member: went_forward
  {
    out << "went_forward: ";
    rosidl_generator_traits::value_to_yaml(msg.went_forward, out);
    out << ", ";
  }

  // member: navigation_time
  {
    out << "navigation_time: ";
    rosidl_generator_traits::value_to_yaml(msg.navigation_time, out);
    out << ", ";
  }

  // member: distance_traveled
  {
    out << "distance_traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_traveled, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: final_aruco_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_aruco_id: ";
    rosidl_generator_traits::value_to_yaml(msg.final_aruco_id, out);
    out << "\n";
  }

  // member: went_forward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "went_forward: ";
    rosidl_generator_traits::value_to_yaml(msg.went_forward, out);
    out << "\n";
  }

  // member: navigation_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "navigation_time: ";
    rosidl_generator_traits::value_to_yaml(msg.navigation_time, out);
    out << "\n";
  }

  // member: distance_traveled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_traveled, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_Result & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_Result>()
{
  return "custom_interfaces::action::NavigateToAruco_Result";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_Result>()
{
  return "custom_interfaces/action/NavigateToAruco_Result";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_aruco_id
  {
    out << "current_aruco_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_aruco_id, out);
    out << ", ";
  }

  // member: current_direction
  {
    out << "current_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.current_direction, out);
    out << ", ";
  }

  // member: elapsed_time
  {
    out << "elapsed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_time, out);
    out << ", ";
  }

  // member: status_message
  {
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_aruco_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_aruco_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_aruco_id, out);
    out << "\n";
  }

  // member: current_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_direction: ";
    rosidl_generator_traits::value_to_yaml(msg.current_direction, out);
    out << "\n";
  }

  // member: elapsed_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elapsed_time: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_time, out);
    out << "\n";
  }

  // member: status_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_Feedback & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_Feedback>()
{
  return "custom_interfaces::action::NavigateToAruco_Feedback";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_Feedback>()
{
  return "custom_interfaces/action/NavigateToAruco_Feedback";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "custom_interfaces/action/detail/navigate_to_aruco__traits.hpp"

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_SendGoal_Request & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_SendGoal_Request>()
{
  return "custom_interfaces::action::NavigateToAruco_SendGoal_Request";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_SendGoal_Request>()
{
  return "custom_interfaces/action/NavigateToAruco_SendGoal_Request";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<custom_interfaces::action::NavigateToAruco_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<custom_interfaces::action::NavigateToAruco_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_SendGoal_Response & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_SendGoal_Response>()
{
  return "custom_interfaces::action::NavigateToAruco_SendGoal_Response";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_SendGoal_Response>()
{
  return "custom_interfaces/action/NavigateToAruco_SendGoal_Response";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_SendGoal>()
{
  return "custom_interfaces::action::NavigateToAruco_SendGoal";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_SendGoal>()
{
  return "custom_interfaces/action/NavigateToAruco_SendGoal";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interfaces::action::NavigateToAruco_SendGoal_Request>::value &&
    has_fixed_size<custom_interfaces::action::NavigateToAruco_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interfaces::action::NavigateToAruco_SendGoal_Request>::value &&
    has_bounded_size<custom_interfaces::action::NavigateToAruco_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<custom_interfaces::action::NavigateToAruco_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interfaces::action::NavigateToAruco_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interfaces::action::NavigateToAruco_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_GetResult_Request & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_GetResult_Request>()
{
  return "custom_interfaces::action::NavigateToAruco_GetResult_Request";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_GetResult_Request>()
{
  return "custom_interfaces/action/NavigateToAruco_GetResult_Request";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__traits.hpp"

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_GetResult_Response & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_GetResult_Response>()
{
  return "custom_interfaces::action::NavigateToAruco_GetResult_Response";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_GetResult_Response>()
{
  return "custom_interfaces/action/NavigateToAruco_GetResult_Response";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<custom_interfaces::action::NavigateToAruco_Result>::value> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<custom_interfaces::action::NavigateToAruco_Result>::value> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_GetResult>()
{
  return "custom_interfaces::action::NavigateToAruco_GetResult";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_GetResult>()
{
  return "custom_interfaces/action/NavigateToAruco_GetResult";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interfaces::action::NavigateToAruco_GetResult_Request>::value &&
    has_fixed_size<custom_interfaces::action::NavigateToAruco_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interfaces::action::NavigateToAruco_GetResult_Request>::value &&
    has_bounded_size<custom_interfaces::action::NavigateToAruco_GetResult_Response>::value
  >
{
};

template<>
struct is_service<custom_interfaces::action::NavigateToAruco_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interfaces::action::NavigateToAruco_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interfaces::action::NavigateToAruco_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__traits.hpp"

namespace custom_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToAruco_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToAruco_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToAruco_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::action::NavigateToAruco_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::action::NavigateToAruco_FeedbackMessage & msg)
{
  return custom_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::action::NavigateToAruco_FeedbackMessage>()
{
  return "custom_interfaces::action::NavigateToAruco_FeedbackMessage";
}

template<>
inline const char * name<custom_interfaces::action::NavigateToAruco_FeedbackMessage>()
{
  return "custom_interfaces/action/NavigateToAruco_FeedbackMessage";
}

template<>
struct has_fixed_size<custom_interfaces::action::NavigateToAruco_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<custom_interfaces::action::NavigateToAruco_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<custom_interfaces::action::NavigateToAruco_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<custom_interfaces::action::NavigateToAruco_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<custom_interfaces::action::NavigateToAruco_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<custom_interfaces::action::NavigateToAruco>
  : std::true_type
{
};

template<>
struct is_action_goal<custom_interfaces::action::NavigateToAruco_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<custom_interfaces::action::NavigateToAruco_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<custom_interfaces::action::NavigateToAruco_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__TRAITS_HPP_
