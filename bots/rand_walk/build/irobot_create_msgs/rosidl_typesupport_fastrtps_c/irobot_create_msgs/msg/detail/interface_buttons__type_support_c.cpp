// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from irobot_create_msgs:msg/InterfaceButtons.idl
// generated code does not contain a copyright notice
#include "irobot_create_msgs/msg/detail/interface_buttons__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "irobot_create_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "irobot_create_msgs/msg/detail/interface_buttons__struct.h"
#include "irobot_create_msgs/msg/detail/interface_buttons__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "irobot_create_msgs/msg/detail/button__functions.h"  // button_1, button_2, button_power
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
size_t get_serialized_size_irobot_create_msgs__msg__Button(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_irobot_create_msgs__msg__Button(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, Button)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_irobot_create_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_irobot_create_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_irobot_create_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _InterfaceButtons__ros_msg_type = irobot_create_msgs__msg__InterfaceButtons;

static bool _InterfaceButtons__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _InterfaceButtons__ros_msg_type * ros_message = static_cast<const _InterfaceButtons__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: button_1
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, Button
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->button_1, cdr))
    {
      return false;
    }
  }

  // Field name: button_power
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, Button
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->button_power, cdr))
    {
      return false;
    }
  }

  // Field name: button_2
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, Button
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->button_2, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _InterfaceButtons__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _InterfaceButtons__ros_msg_type * ros_message = static_cast<_InterfaceButtons__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: button_1
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, Button
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->button_1))
    {
      return false;
    }
  }

  // Field name: button_power
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, Button
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->button_power))
    {
      return false;
    }
  }

  // Field name: button_2
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, Button
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->button_2))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_irobot_create_msgs
size_t get_serialized_size_irobot_create_msgs__msg__InterfaceButtons(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _InterfaceButtons__ros_msg_type * ros_message = static_cast<const _InterfaceButtons__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name button_1

  current_alignment += get_serialized_size_irobot_create_msgs__msg__Button(
    &(ros_message->button_1), current_alignment);
  // field.name button_power

  current_alignment += get_serialized_size_irobot_create_msgs__msg__Button(
    &(ros_message->button_power), current_alignment);
  // field.name button_2

  current_alignment += get_serialized_size_irobot_create_msgs__msg__Button(
    &(ros_message->button_2), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _InterfaceButtons__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_irobot_create_msgs__msg__InterfaceButtons(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_irobot_create_msgs
size_t max_serialized_size_irobot_create_msgs__msg__InterfaceButtons(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: button_1
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_irobot_create_msgs__msg__Button(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: button_power
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_irobot_create_msgs__msg__Button(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: button_2
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_irobot_create_msgs__msg__Button(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _InterfaceButtons__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_irobot_create_msgs__msg__InterfaceButtons(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_InterfaceButtons = {
  "irobot_create_msgs::msg",
  "InterfaceButtons",
  _InterfaceButtons__cdr_serialize,
  _InterfaceButtons__cdr_deserialize,
  _InterfaceButtons__get_serialized_size,
  _InterfaceButtons__max_serialized_size
};

static rosidl_message_type_support_t _InterfaceButtons__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_InterfaceButtons,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, irobot_create_msgs, msg, InterfaceButtons)() {
  return &_InterfaceButtons__type_support;
}

#if defined(__cplusplus)
}
#endif
