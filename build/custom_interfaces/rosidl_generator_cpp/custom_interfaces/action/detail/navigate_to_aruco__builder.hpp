// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:action/NavigateToAruco.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__BUILDER_HPP_
#define CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/action/detail/navigate_to_aruco__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_Goal_color_choice
{
public:
  explicit Init_NavigateToAruco_Goal_color_choice(::custom_interfaces::action::NavigateToAruco_Goal & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::NavigateToAruco_Goal color_choice(::custom_interfaces::action::NavigateToAruco_Goal::_color_choice_type arg)
  {
    msg_.color_choice = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Goal msg_;
};

class Init_NavigateToAruco_Goal_return_to_zero
{
public:
  explicit Init_NavigateToAruco_Goal_return_to_zero(::custom_interfaces::action::NavigateToAruco_Goal & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Goal_color_choice return_to_zero(::custom_interfaces::action::NavigateToAruco_Goal::_return_to_zero_type arg)
  {
    msg_.return_to_zero = std::move(arg);
    return Init_NavigateToAruco_Goal_color_choice(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Goal msg_;
};

class Init_NavigateToAruco_Goal_target_aruco_id
{
public:
  Init_NavigateToAruco_Goal_target_aruco_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToAruco_Goal_return_to_zero target_aruco_id(::custom_interfaces::action::NavigateToAruco_Goal::_target_aruco_id_type arg)
  {
    msg_.target_aruco_id = std::move(arg);
    return Init_NavigateToAruco_Goal_return_to_zero(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_Goal>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_Goal_target_aruco_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_Result_returned_to_zero
{
public:
  explicit Init_NavigateToAruco_Result_returned_to_zero(::custom_interfaces::action::NavigateToAruco_Result & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::NavigateToAruco_Result returned_to_zero(::custom_interfaces::action::NavigateToAruco_Result::_returned_to_zero_type arg)
  {
    msg_.returned_to_zero = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Result msg_;
};

class Init_NavigateToAruco_Result_distance_traveled
{
public:
  explicit Init_NavigateToAruco_Result_distance_traveled(::custom_interfaces::action::NavigateToAruco_Result & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Result_returned_to_zero distance_traveled(::custom_interfaces::action::NavigateToAruco_Result::_distance_traveled_type arg)
  {
    msg_.distance_traveled = std::move(arg);
    return Init_NavigateToAruco_Result_returned_to_zero(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Result msg_;
};

class Init_NavigateToAruco_Result_navigation_time
{
public:
  explicit Init_NavigateToAruco_Result_navigation_time(::custom_interfaces::action::NavigateToAruco_Result & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Result_distance_traveled navigation_time(::custom_interfaces::action::NavigateToAruco_Result::_navigation_time_type arg)
  {
    msg_.navigation_time = std::move(arg);
    return Init_NavigateToAruco_Result_distance_traveled(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Result msg_;
};

class Init_NavigateToAruco_Result_went_forward
{
public:
  explicit Init_NavigateToAruco_Result_went_forward(::custom_interfaces::action::NavigateToAruco_Result & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Result_navigation_time went_forward(::custom_interfaces::action::NavigateToAruco_Result::_went_forward_type arg)
  {
    msg_.went_forward = std::move(arg);
    return Init_NavigateToAruco_Result_navigation_time(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Result msg_;
};

class Init_NavigateToAruco_Result_final_aruco_id
{
public:
  explicit Init_NavigateToAruco_Result_final_aruco_id(::custom_interfaces::action::NavigateToAruco_Result & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Result_went_forward final_aruco_id(::custom_interfaces::action::NavigateToAruco_Result::_final_aruco_id_type arg)
  {
    msg_.final_aruco_id = std::move(arg);
    return Init_NavigateToAruco_Result_went_forward(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Result msg_;
};

class Init_NavigateToAruco_Result_success
{
public:
  Init_NavigateToAruco_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToAruco_Result_final_aruco_id success(::custom_interfaces::action::NavigateToAruco_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_NavigateToAruco_Result_final_aruco_id(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_Result>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_Result_success();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_Feedback_obstacle_distance
{
public:
  explicit Init_NavigateToAruco_Feedback_obstacle_distance(::custom_interfaces::action::NavigateToAruco_Feedback & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::NavigateToAruco_Feedback obstacle_distance(::custom_interfaces::action::NavigateToAruco_Feedback::_obstacle_distance_type arg)
  {
    msg_.obstacle_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Feedback msg_;
};

class Init_NavigateToAruco_Feedback_obstacle_detected
{
public:
  explicit Init_NavigateToAruco_Feedback_obstacle_detected(::custom_interfaces::action::NavigateToAruco_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Feedback_obstacle_distance obstacle_detected(::custom_interfaces::action::NavigateToAruco_Feedback::_obstacle_detected_type arg)
  {
    msg_.obstacle_detected = std::move(arg);
    return Init_NavigateToAruco_Feedback_obstacle_distance(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Feedback msg_;
};

class Init_NavigateToAruco_Feedback_status_message
{
public:
  explicit Init_NavigateToAruco_Feedback_status_message(::custom_interfaces::action::NavigateToAruco_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Feedback_obstacle_detected status_message(::custom_interfaces::action::NavigateToAruco_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_NavigateToAruco_Feedback_obstacle_detected(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Feedback msg_;
};

class Init_NavigateToAruco_Feedback_elapsed_time
{
public:
  explicit Init_NavigateToAruco_Feedback_elapsed_time(::custom_interfaces::action::NavigateToAruco_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Feedback_status_message elapsed_time(::custom_interfaces::action::NavigateToAruco_Feedback::_elapsed_time_type arg)
  {
    msg_.elapsed_time = std::move(arg);
    return Init_NavigateToAruco_Feedback_status_message(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Feedback msg_;
};

class Init_NavigateToAruco_Feedback_current_direction
{
public:
  explicit Init_NavigateToAruco_Feedback_current_direction(::custom_interfaces::action::NavigateToAruco_Feedback & msg)
  : msg_(msg)
  {}
  Init_NavigateToAruco_Feedback_elapsed_time current_direction(::custom_interfaces::action::NavigateToAruco_Feedback::_current_direction_type arg)
  {
    msg_.current_direction = std::move(arg);
    return Init_NavigateToAruco_Feedback_elapsed_time(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Feedback msg_;
};

class Init_NavigateToAruco_Feedback_current_aruco_id
{
public:
  Init_NavigateToAruco_Feedback_current_aruco_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToAruco_Feedback_current_direction current_aruco_id(::custom_interfaces::action::NavigateToAruco_Feedback::_current_aruco_id_type arg)
  {
    msg_.current_aruco_id = std::move(arg);
    return Init_NavigateToAruco_Feedback_current_direction(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_Feedback>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_Feedback_current_aruco_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_SendGoal_Request_goal
{
public:
  explicit Init_NavigateToAruco_SendGoal_Request_goal(::custom_interfaces::action::NavigateToAruco_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::NavigateToAruco_SendGoal_Request goal(::custom_interfaces::action::NavigateToAruco_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_SendGoal_Request msg_;
};

class Init_NavigateToAruco_SendGoal_Request_goal_id
{
public:
  Init_NavigateToAruco_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToAruco_SendGoal_Request_goal goal_id(::custom_interfaces::action::NavigateToAruco_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateToAruco_SendGoal_Request_goal(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_SendGoal_Request>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_SendGoal_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_SendGoal_Response_stamp
{
public:
  explicit Init_NavigateToAruco_SendGoal_Response_stamp(::custom_interfaces::action::NavigateToAruco_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::NavigateToAruco_SendGoal_Response stamp(::custom_interfaces::action::NavigateToAruco_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_SendGoal_Response msg_;
};

class Init_NavigateToAruco_SendGoal_Response_accepted
{
public:
  Init_NavigateToAruco_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToAruco_SendGoal_Response_stamp accepted(::custom_interfaces::action::NavigateToAruco_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_NavigateToAruco_SendGoal_Response_stamp(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_SendGoal_Response>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_SendGoal_Response_accepted();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_GetResult_Request_goal_id
{
public:
  Init_NavigateToAruco_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::NavigateToAruco_GetResult_Request goal_id(::custom_interfaces::action::NavigateToAruco_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_GetResult_Request>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_GetResult_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_GetResult_Response_result
{
public:
  explicit Init_NavigateToAruco_GetResult_Response_result(::custom_interfaces::action::NavigateToAruco_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::NavigateToAruco_GetResult_Response result(::custom_interfaces::action::NavigateToAruco_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_GetResult_Response msg_;
};

class Init_NavigateToAruco_GetResult_Response_status
{
public:
  Init_NavigateToAruco_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToAruco_GetResult_Response_result status(::custom_interfaces::action::NavigateToAruco_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavigateToAruco_GetResult_Response_result(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_GetResult_Response>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_GetResult_Response_status();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_NavigateToAruco_FeedbackMessage_feedback
{
public:
  explicit Init_NavigateToAruco_FeedbackMessage_feedback(::custom_interfaces::action::NavigateToAruco_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::NavigateToAruco_FeedbackMessage feedback(::custom_interfaces::action::NavigateToAruco_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_FeedbackMessage msg_;
};

class Init_NavigateToAruco_FeedbackMessage_goal_id
{
public:
  Init_NavigateToAruco_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigateToAruco_FeedbackMessage_feedback goal_id(::custom_interfaces::action::NavigateToAruco_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_NavigateToAruco_FeedbackMessage_feedback(msg_);
  }

private:
  ::custom_interfaces::action::NavigateToAruco_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::NavigateToAruco_FeedbackMessage>()
{
  return custom_interfaces::action::builder::Init_NavigateToAruco_FeedbackMessage_goal_id();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__BUILDER_HPP_
