// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ur_dashboard_msgs:srv/RawRequest.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__FUNCTIONS_H_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ur_dashboard_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ur_dashboard_msgs/srv/detail/raw_request__struct.h"

/// Initialize srv/RawRequest message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ur_dashboard_msgs__srv__RawRequest_Request
 * )) before or use
 * ur_dashboard_msgs__srv__RawRequest_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Request__init(ur_dashboard_msgs__srv__RawRequest_Request * msg);

/// Finalize srv/RawRequest message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Request__fini(ur_dashboard_msgs__srv__RawRequest_Request * msg);

/// Create srv/RawRequest message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ur_dashboard_msgs__srv__RawRequest_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
ur_dashboard_msgs__srv__RawRequest_Request *
ur_dashboard_msgs__srv__RawRequest_Request__create();

/// Destroy srv/RawRequest message.
/**
 * It calls
 * ur_dashboard_msgs__srv__RawRequest_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Request__destroy(ur_dashboard_msgs__srv__RawRequest_Request * msg);

/// Check for srv/RawRequest message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Request__are_equal(const ur_dashboard_msgs__srv__RawRequest_Request * lhs, const ur_dashboard_msgs__srv__RawRequest_Request * rhs);

/// Copy a srv/RawRequest message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Request__copy(
  const ur_dashboard_msgs__srv__RawRequest_Request * input,
  ur_dashboard_msgs__srv__RawRequest_Request * output);

/// Initialize array of srv/RawRequest messages.
/**
 * It allocates the memory for the number of elements and calls
 * ur_dashboard_msgs__srv__RawRequest_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Request__Sequence__init(ur_dashboard_msgs__srv__RawRequest_Request__Sequence * array, size_t size);

/// Finalize array of srv/RawRequest messages.
/**
 * It calls
 * ur_dashboard_msgs__srv__RawRequest_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Request__Sequence__fini(ur_dashboard_msgs__srv__RawRequest_Request__Sequence * array);

/// Create array of srv/RawRequest messages.
/**
 * It allocates the memory for the array and calls
 * ur_dashboard_msgs__srv__RawRequest_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
ur_dashboard_msgs__srv__RawRequest_Request__Sequence *
ur_dashboard_msgs__srv__RawRequest_Request__Sequence__create(size_t size);

/// Destroy array of srv/RawRequest messages.
/**
 * It calls
 * ur_dashboard_msgs__srv__RawRequest_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Request__Sequence__destroy(ur_dashboard_msgs__srv__RawRequest_Request__Sequence * array);

/// Check for srv/RawRequest message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Request__Sequence__are_equal(const ur_dashboard_msgs__srv__RawRequest_Request__Sequence * lhs, const ur_dashboard_msgs__srv__RawRequest_Request__Sequence * rhs);

/// Copy an array of srv/RawRequest messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Request__Sequence__copy(
  const ur_dashboard_msgs__srv__RawRequest_Request__Sequence * input,
  ur_dashboard_msgs__srv__RawRequest_Request__Sequence * output);

/// Initialize srv/RawRequest message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ur_dashboard_msgs__srv__RawRequest_Response
 * )) before or use
 * ur_dashboard_msgs__srv__RawRequest_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Response__init(ur_dashboard_msgs__srv__RawRequest_Response * msg);

/// Finalize srv/RawRequest message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Response__fini(ur_dashboard_msgs__srv__RawRequest_Response * msg);

/// Create srv/RawRequest message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ur_dashboard_msgs__srv__RawRequest_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
ur_dashboard_msgs__srv__RawRequest_Response *
ur_dashboard_msgs__srv__RawRequest_Response__create();

/// Destroy srv/RawRequest message.
/**
 * It calls
 * ur_dashboard_msgs__srv__RawRequest_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Response__destroy(ur_dashboard_msgs__srv__RawRequest_Response * msg);

/// Check for srv/RawRequest message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Response__are_equal(const ur_dashboard_msgs__srv__RawRequest_Response * lhs, const ur_dashboard_msgs__srv__RawRequest_Response * rhs);

/// Copy a srv/RawRequest message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Response__copy(
  const ur_dashboard_msgs__srv__RawRequest_Response * input,
  ur_dashboard_msgs__srv__RawRequest_Response * output);

/// Initialize array of srv/RawRequest messages.
/**
 * It allocates the memory for the number of elements and calls
 * ur_dashboard_msgs__srv__RawRequest_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Response__Sequence__init(ur_dashboard_msgs__srv__RawRequest_Response__Sequence * array, size_t size);

/// Finalize array of srv/RawRequest messages.
/**
 * It calls
 * ur_dashboard_msgs__srv__RawRequest_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Response__Sequence__fini(ur_dashboard_msgs__srv__RawRequest_Response__Sequence * array);

/// Create array of srv/RawRequest messages.
/**
 * It allocates the memory for the array and calls
 * ur_dashboard_msgs__srv__RawRequest_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
ur_dashboard_msgs__srv__RawRequest_Response__Sequence *
ur_dashboard_msgs__srv__RawRequest_Response__Sequence__create(size_t size);

/// Destroy array of srv/RawRequest messages.
/**
 * It calls
 * ur_dashboard_msgs__srv__RawRequest_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
void
ur_dashboard_msgs__srv__RawRequest_Response__Sequence__destroy(ur_dashboard_msgs__srv__RawRequest_Response__Sequence * array);

/// Check for srv/RawRequest message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Response__Sequence__are_equal(const ur_dashboard_msgs__srv__RawRequest_Response__Sequence * lhs, const ur_dashboard_msgs__srv__RawRequest_Response__Sequence * rhs);

/// Copy an array of srv/RawRequest messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ur_dashboard_msgs
bool
ur_dashboard_msgs__srv__RawRequest_Response__Sequence__copy(
  const ur_dashboard_msgs__srv__RawRequest_Response__Sequence * input,
  ur_dashboard_msgs__srv__RawRequest_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__RAW_REQUEST__FUNCTIONS_H_
