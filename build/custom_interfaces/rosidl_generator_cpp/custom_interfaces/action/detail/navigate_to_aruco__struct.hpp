// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:action/NavigateToAruco.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__STRUCT_HPP_
#define CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_Goal __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_Goal __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_Goal_
{
  using Type = NavigateToAruco_Goal_<ContainerAllocator>;

  explicit NavigateToAruco_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_aruco_id = 0l;
      this->return_to_zero = false;
      this->color_choice = "";
    }
  }

  explicit NavigateToAruco_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color_choice(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_aruco_id = 0l;
      this->return_to_zero = false;
      this->color_choice = "";
    }
  }

  // field types and members
  using _target_aruco_id_type =
    int32_t;
  _target_aruco_id_type target_aruco_id;
  using _return_to_zero_type =
    bool;
  _return_to_zero_type return_to_zero;
  using _color_choice_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _color_choice_type color_choice;

  // setters for named parameter idiom
  Type & set__target_aruco_id(
    const int32_t & _arg)
  {
    this->target_aruco_id = _arg;
    return *this;
  }
  Type & set__return_to_zero(
    const bool & _arg)
  {
    this->return_to_zero = _arg;
    return *this;
  }
  Type & set__color_choice(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->color_choice = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_Goal
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_Goal
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_Goal_ & other) const
  {
    if (this->target_aruco_id != other.target_aruco_id) {
      return false;
    }
    if (this->return_to_zero != other.return_to_zero) {
      return false;
    }
    if (this->color_choice != other.color_choice) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_Goal_

// alias to use template instance with default allocator
using NavigateToAruco_Goal =
  custom_interfaces::action::NavigateToAruco_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_Result __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_Result __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_Result_
{
  using Type = NavigateToAruco_Result_<ContainerAllocator>;

  explicit NavigateToAruco_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->final_aruco_id = 0l;
      this->went_forward = false;
      this->navigation_time = 0.0f;
      this->distance_traveled = 0.0f;
      this->returned_to_zero = false;
    }
  }

  explicit NavigateToAruco_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->final_aruco_id = 0l;
      this->went_forward = false;
      this->navigation_time = 0.0f;
      this->distance_traveled = 0.0f;
      this->returned_to_zero = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _final_aruco_id_type =
    int32_t;
  _final_aruco_id_type final_aruco_id;
  using _went_forward_type =
    bool;
  _went_forward_type went_forward;
  using _navigation_time_type =
    float;
  _navigation_time_type navigation_time;
  using _distance_traveled_type =
    float;
  _distance_traveled_type distance_traveled;
  using _returned_to_zero_type =
    bool;
  _returned_to_zero_type returned_to_zero;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__final_aruco_id(
    const int32_t & _arg)
  {
    this->final_aruco_id = _arg;
    return *this;
  }
  Type & set__went_forward(
    const bool & _arg)
  {
    this->went_forward = _arg;
    return *this;
  }
  Type & set__navigation_time(
    const float & _arg)
  {
    this->navigation_time = _arg;
    return *this;
  }
  Type & set__distance_traveled(
    const float & _arg)
  {
    this->distance_traveled = _arg;
    return *this;
  }
  Type & set__returned_to_zero(
    const bool & _arg)
  {
    this->returned_to_zero = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_Result
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_Result
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->final_aruco_id != other.final_aruco_id) {
      return false;
    }
    if (this->went_forward != other.went_forward) {
      return false;
    }
    if (this->navigation_time != other.navigation_time) {
      return false;
    }
    if (this->distance_traveled != other.distance_traveled) {
      return false;
    }
    if (this->returned_to_zero != other.returned_to_zero) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_Result_

// alias to use template instance with default allocator
using NavigateToAruco_Result =
  custom_interfaces::action::NavigateToAruco_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_Feedback __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_Feedback_
{
  using Type = NavigateToAruco_Feedback_<ContainerAllocator>;

  explicit NavigateToAruco_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_aruco_id = 0l;
      this->current_direction = "";
      this->elapsed_time = 0.0f;
      this->status_message = "";
      this->obstacle_detected = false;
      this->obstacle_distance = 0.0f;
    }
  }

  explicit NavigateToAruco_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_direction(_alloc),
    status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_aruco_id = 0l;
      this->current_direction = "";
      this->elapsed_time = 0.0f;
      this->status_message = "";
      this->obstacle_detected = false;
      this->obstacle_distance = 0.0f;
    }
  }

  // field types and members
  using _current_aruco_id_type =
    int32_t;
  _current_aruco_id_type current_aruco_id;
  using _current_direction_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_direction_type current_direction;
  using _elapsed_time_type =
    float;
  _elapsed_time_type elapsed_time;
  using _status_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_message_type status_message;
  using _obstacle_detected_type =
    bool;
  _obstacle_detected_type obstacle_detected;
  using _obstacle_distance_type =
    float;
  _obstacle_distance_type obstacle_distance;

  // setters for named parameter idiom
  Type & set__current_aruco_id(
    const int32_t & _arg)
  {
    this->current_aruco_id = _arg;
    return *this;
  }
  Type & set__current_direction(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_direction = _arg;
    return *this;
  }
  Type & set__elapsed_time(
    const float & _arg)
  {
    this->elapsed_time = _arg;
    return *this;
  }
  Type & set__status_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status_message = _arg;
    return *this;
  }
  Type & set__obstacle_detected(
    const bool & _arg)
  {
    this->obstacle_detected = _arg;
    return *this;
  }
  Type & set__obstacle_distance(
    const float & _arg)
  {
    this->obstacle_distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_Feedback
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_Feedback
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_Feedback_ & other) const
  {
    if (this->current_aruco_id != other.current_aruco_id) {
      return false;
    }
    if (this->current_direction != other.current_direction) {
      return false;
    }
    if (this->elapsed_time != other.elapsed_time) {
      return false;
    }
    if (this->status_message != other.status_message) {
      return false;
    }
    if (this->obstacle_detected != other.obstacle_detected) {
      return false;
    }
    if (this->obstacle_distance != other.obstacle_distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_Feedback_

// alias to use template instance with default allocator
using NavigateToAruco_Feedback =
  custom_interfaces::action::NavigateToAruco_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "custom_interfaces/action/detail/navigate_to_aruco__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_SendGoal_Request_
{
  using Type = NavigateToAruco_SendGoal_Request_<ContainerAllocator>;

  explicit NavigateToAruco_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit NavigateToAruco_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const custom_interfaces::action::NavigateToAruco_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Request
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Request
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_SendGoal_Request_

// alias to use template instance with default allocator
using NavigateToAruco_SendGoal_Request =
  custom_interfaces::action::NavigateToAruco_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_SendGoal_Response_
{
  using Type = NavigateToAruco_SendGoal_Response_<ContainerAllocator>;

  explicit NavigateToAruco_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit NavigateToAruco_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Response
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_SendGoal_Response
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_SendGoal_Response_

// alias to use template instance with default allocator
using NavigateToAruco_SendGoal_Response =
  custom_interfaces::action::NavigateToAruco_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace action
{

struct NavigateToAruco_SendGoal
{
  using Request = custom_interfaces::action::NavigateToAruco_SendGoal_Request;
  using Response = custom_interfaces::action::NavigateToAruco_SendGoal_Response;
};

}  // namespace action

}  // namespace custom_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_GetResult_Request_
{
  using Type = NavigateToAruco_GetResult_Request_<ContainerAllocator>;

  explicit NavigateToAruco_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit NavigateToAruco_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Request
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Request
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_GetResult_Request_

// alias to use template instance with default allocator
using NavigateToAruco_GetResult_Request =
  custom_interfaces::action::NavigateToAruco_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_GetResult_Response_
{
  using Type = NavigateToAruco_GetResult_Response_<ContainerAllocator>;

  explicit NavigateToAruco_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit NavigateToAruco_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const custom_interfaces::action::NavigateToAruco_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Response
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_GetResult_Response
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_GetResult_Response_

// alias to use template instance with default allocator
using NavigateToAruco_GetResult_Response =
  custom_interfaces::action::NavigateToAruco_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace action
{

struct NavigateToAruco_GetResult
{
  using Request = custom_interfaces::action::NavigateToAruco_GetResult_Request;
  using Response = custom_interfaces::action::NavigateToAruco_GetResult_Response;
};

}  // namespace action

}  // namespace custom_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__action__NavigateToAruco_FeedbackMessage __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct NavigateToAruco_FeedbackMessage_
{
  using Type = NavigateToAruco_FeedbackMessage_<ContainerAllocator>;

  explicit NavigateToAruco_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit NavigateToAruco_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const custom_interfaces::action::NavigateToAruco_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_FeedbackMessage
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__action__NavigateToAruco_FeedbackMessage
    std::shared_ptr<custom_interfaces::action::NavigateToAruco_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigateToAruco_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigateToAruco_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigateToAruco_FeedbackMessage_

// alias to use template instance with default allocator
using NavigateToAruco_FeedbackMessage =
  custom_interfaces::action::NavigateToAruco_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace custom_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace custom_interfaces
{

namespace action
{

struct NavigateToAruco
{
  /// The goal message defined in the action definition.
  using Goal = custom_interfaces::action::NavigateToAruco_Goal;
  /// The result message defined in the action definition.
  using Result = custom_interfaces::action::NavigateToAruco_Result;
  /// The feedback message defined in the action definition.
  using Feedback = custom_interfaces::action::NavigateToAruco_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = custom_interfaces::action::NavigateToAruco_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = custom_interfaces::action::NavigateToAruco_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = custom_interfaces::action::NavigateToAruco_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct NavigateToAruco NavigateToAruco;

}  // namespace action

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__STRUCT_HPP_
