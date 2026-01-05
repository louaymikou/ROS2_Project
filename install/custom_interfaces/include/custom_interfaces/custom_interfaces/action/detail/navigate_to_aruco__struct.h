// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:action/NavigateToAruco.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__STRUCT_H_
#define CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_Goal
{
  int32_t target_aruco_id;
  /// Retourner à ArUco 0 après avoir atteint la cible
  bool return_to_zero;
} custom_interfaces__action__NavigateToAruco_Goal;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_Goal.
typedef struct custom_interfaces__action__NavigateToAruco_Goal__Sequence
{
  custom_interfaces__action__NavigateToAruco_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_Result
{
  /// ========================================
  /// RESULT - Résultat de la navigation
  /// ========================================
  /// Navigation réussie?
  bool success;
  /// Numéro ArUco final atteint
  int32_t final_aruco_id;
  /// Direction utilisée (true=avant, false=arrière)
  bool went_forward;
  /// Temps total de navigation (secondes)
  float navigation_time;
  /// Distance parcourue (approximative)
  float distance_traveled;
  /// A effectué le retour à ArUco 0
  bool returned_to_zero;
} custom_interfaces__action__NavigateToAruco_Result;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_Result.
typedef struct custom_interfaces__action__NavigateToAruco_Result__Sequence
{
  custom_interfaces__action__NavigateToAruco_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_direction'
// Member 'status_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_Feedback
{
  /// ========================================
  /// FEEDBACK - Progression pendant navigation
  /// ========================================
  /// Numéro ArUco actuellement détecté (0 si aucun)
  int32_t current_aruco_id;
  /// Direction de navigation actuelle
  rosidl_runtime_c__String current_direction;
  /// Temps écoulé (secondes)
  float elapsed_time;
  /// État de la navigation
  rosidl_runtime_c__String status_message;
  /// Obstacle détecté (true si obstacle présent)
  bool obstacle_detected;
  /// Distance de l'obstacle (mètres, 0.0 si pas d'obstacle)
  float obstacle_distance;
} custom_interfaces__action__NavigateToAruco_Feedback;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_Feedback.
typedef struct custom_interfaces__action__NavigateToAruco_Feedback__Sequence
{
  custom_interfaces__action__NavigateToAruco_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interfaces__action__NavigateToAruco_Goal goal;
} custom_interfaces__action__NavigateToAruco_SendGoal_Request;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_SendGoal_Request.
typedef struct custom_interfaces__action__NavigateToAruco_SendGoal_Request__Sequence
{
  custom_interfaces__action__NavigateToAruco_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} custom_interfaces__action__NavigateToAruco_SendGoal_Response;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_SendGoal_Response.
typedef struct custom_interfaces__action__NavigateToAruco_SendGoal_Response__Sequence
{
  custom_interfaces__action__NavigateToAruco_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} custom_interfaces__action__NavigateToAruco_GetResult_Request;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_GetResult_Request.
typedef struct custom_interfaces__action__NavigateToAruco_GetResult_Request__Sequence
{
  custom_interfaces__action__NavigateToAruco_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_GetResult_Response
{
  int8_t status;
  custom_interfaces__action__NavigateToAruco_Result result;
} custom_interfaces__action__NavigateToAruco_GetResult_Response;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_GetResult_Response.
typedef struct custom_interfaces__action__NavigateToAruco_GetResult_Response__Sequence
{
  custom_interfaces__action__NavigateToAruco_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "custom_interfaces/action/detail/navigate_to_aruco__struct.h"

/// Struct defined in action/NavigateToAruco in the package custom_interfaces.
typedef struct custom_interfaces__action__NavigateToAruco_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interfaces__action__NavigateToAruco_Feedback feedback;
} custom_interfaces__action__NavigateToAruco_FeedbackMessage;

// Struct for a sequence of custom_interfaces__action__NavigateToAruco_FeedbackMessage.
typedef struct custom_interfaces__action__NavigateToAruco_FeedbackMessage__Sequence
{
  custom_interfaces__action__NavigateToAruco_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__NavigateToAruco_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_ARUCO__STRUCT_H_
