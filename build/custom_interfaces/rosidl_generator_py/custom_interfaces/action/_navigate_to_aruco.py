# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_interfaces:action/NavigateToAruco.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NavigateToAruco_Goal(type):
    """Metaclass of message 'NavigateToAruco_Goal'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_Goal(metaclass=Metaclass_NavigateToAruco_Goal):
    """Message class 'NavigateToAruco_Goal'."""

    __slots__ = [
        '_target_aruco_id',
        '_return_to_zero',
        '_color_choice',
    ]

    _fields_and_field_types = {
        'target_aruco_id': 'int32',
        'return_to_zero': 'boolean',
        'color_choice': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.target_aruco_id = kwargs.get('target_aruco_id', int())
        self.return_to_zero = kwargs.get('return_to_zero', bool())
        self.color_choice = kwargs.get('color_choice', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.target_aruco_id != other.target_aruco_id:
            return False
        if self.return_to_zero != other.return_to_zero:
            return False
        if self.color_choice != other.color_choice:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def target_aruco_id(self):
        """Message field 'target_aruco_id'."""
        return self._target_aruco_id

    @target_aruco_id.setter
    def target_aruco_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'target_aruco_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'target_aruco_id' field must be an integer in [-2147483648, 2147483647]"
        self._target_aruco_id = value

    @builtins.property
    def return_to_zero(self):
        """Message field 'return_to_zero'."""
        return self._return_to_zero

    @return_to_zero.setter
    def return_to_zero(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'return_to_zero' field must be of type 'bool'"
        self._return_to_zero = value

    @builtins.property
    def color_choice(self):
        """Message field 'color_choice'."""
        return self._color_choice

    @color_choice.setter
    def color_choice(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'color_choice' field must be of type 'str'"
        self._color_choice = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateToAruco_Result(type):
    """Metaclass of message 'NavigateToAruco_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__result

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_Result(metaclass=Metaclass_NavigateToAruco_Result):
    """Message class 'NavigateToAruco_Result'."""

    __slots__ = [
        '_success',
        '_final_aruco_id',
        '_went_forward',
        '_navigation_time',
        '_distance_traveled',
        '_returned_to_zero',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'final_aruco_id': 'int32',
        'went_forward': 'boolean',
        'navigation_time': 'float',
        'distance_traveled': 'float',
        'returned_to_zero': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.final_aruco_id = kwargs.get('final_aruco_id', int())
        self.went_forward = kwargs.get('went_forward', bool())
        self.navigation_time = kwargs.get('navigation_time', float())
        self.distance_traveled = kwargs.get('distance_traveled', float())
        self.returned_to_zero = kwargs.get('returned_to_zero', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.final_aruco_id != other.final_aruco_id:
            return False
        if self.went_forward != other.went_forward:
            return False
        if self.navigation_time != other.navigation_time:
            return False
        if self.distance_traveled != other.distance_traveled:
            return False
        if self.returned_to_zero != other.returned_to_zero:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def final_aruco_id(self):
        """Message field 'final_aruco_id'."""
        return self._final_aruco_id

    @final_aruco_id.setter
    def final_aruco_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'final_aruco_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'final_aruco_id' field must be an integer in [-2147483648, 2147483647]"
        self._final_aruco_id = value

    @builtins.property
    def went_forward(self):
        """Message field 'went_forward'."""
        return self._went_forward

    @went_forward.setter
    def went_forward(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'went_forward' field must be of type 'bool'"
        self._went_forward = value

    @builtins.property
    def navigation_time(self):
        """Message field 'navigation_time'."""
        return self._navigation_time

    @navigation_time.setter
    def navigation_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'navigation_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'navigation_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._navigation_time = value

    @builtins.property
    def distance_traveled(self):
        """Message field 'distance_traveled'."""
        return self._distance_traveled

    @distance_traveled.setter
    def distance_traveled(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_traveled' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_traveled' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_traveled = value

    @builtins.property
    def returned_to_zero(self):
        """Message field 'returned_to_zero'."""
        return self._returned_to_zero

    @returned_to_zero.setter
    def returned_to_zero(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'returned_to_zero' field must be of type 'bool'"
        self._returned_to_zero = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateToAruco_Feedback(type):
    """Metaclass of message 'NavigateToAruco_Feedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_Feedback(metaclass=Metaclass_NavigateToAruco_Feedback):
    """Message class 'NavigateToAruco_Feedback'."""

    __slots__ = [
        '_current_aruco_id',
        '_current_direction',
        '_elapsed_time',
        '_status_message',
        '_obstacle_detected',
        '_obstacle_distance',
    ]

    _fields_and_field_types = {
        'current_aruco_id': 'int32',
        'current_direction': 'string',
        'elapsed_time': 'float',
        'status_message': 'string',
        'obstacle_detected': 'boolean',
        'obstacle_distance': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current_aruco_id = kwargs.get('current_aruco_id', int())
        self.current_direction = kwargs.get('current_direction', str())
        self.elapsed_time = kwargs.get('elapsed_time', float())
        self.status_message = kwargs.get('status_message', str())
        self.obstacle_detected = kwargs.get('obstacle_detected', bool())
        self.obstacle_distance = kwargs.get('obstacle_distance', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.current_aruco_id != other.current_aruco_id:
            return False
        if self.current_direction != other.current_direction:
            return False
        if self.elapsed_time != other.elapsed_time:
            return False
        if self.status_message != other.status_message:
            return False
        if self.obstacle_detected != other.obstacle_detected:
            return False
        if self.obstacle_distance != other.obstacle_distance:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_aruco_id(self):
        """Message field 'current_aruco_id'."""
        return self._current_aruco_id

    @current_aruco_id.setter
    def current_aruco_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_aruco_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'current_aruco_id' field must be an integer in [-2147483648, 2147483647]"
        self._current_aruco_id = value

    @builtins.property
    def current_direction(self):
        """Message field 'current_direction'."""
        return self._current_direction

    @current_direction.setter
    def current_direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'current_direction' field must be of type 'str'"
        self._current_direction = value

    @builtins.property
    def elapsed_time(self):
        """Message field 'elapsed_time'."""
        return self._elapsed_time

    @elapsed_time.setter
    def elapsed_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'elapsed_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'elapsed_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._elapsed_time = value

    @builtins.property
    def status_message(self):
        """Message field 'status_message'."""
        return self._status_message

    @status_message.setter
    def status_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status_message' field must be of type 'str'"
        self._status_message = value

    @builtins.property
    def obstacle_detected(self):
        """Message field 'obstacle_detected'."""
        return self._obstacle_detected

    @obstacle_detected.setter
    def obstacle_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'obstacle_detected' field must be of type 'bool'"
        self._obstacle_detected = value

    @builtins.property
    def obstacle_distance(self):
        """Message field 'obstacle_distance'."""
        return self._obstacle_distance

    @obstacle_distance.setter
    def obstacle_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obstacle_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'obstacle_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._obstacle_distance = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateToAruco_SendGoal_Request(type):
    """Metaclass of message 'NavigateToAruco_SendGoal_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__send_goal__request

            from custom_interfaces.action import NavigateToAruco
            if NavigateToAruco.Goal.__class__._TYPE_SUPPORT is None:
                NavigateToAruco.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_SendGoal_Request(metaclass=Metaclass_NavigateToAruco_SendGoal_Request):
    """Message class 'NavigateToAruco_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'custom_interfaces/NavigateToAruco_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_interfaces', 'action'], 'NavigateToAruco_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Goal
        self.goal = kwargs.get('goal', NavigateToAruco_Goal())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Goal
            assert \
                isinstance(value, NavigateToAruco_Goal), \
                "The 'goal' field must be a sub message of type 'NavigateToAruco_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateToAruco_SendGoal_Response(type):
    """Metaclass of message 'NavigateToAruco_SendGoal_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_SendGoal_Response(metaclass=Metaclass_NavigateToAruco_SendGoal_Response):
    """Message class 'NavigateToAruco_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_NavigateToAruco_SendGoal(type):
    """Metaclass of service 'NavigateToAruco_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__navigate_to_aruco__send_goal

            from custom_interfaces.action import _navigate_to_aruco
            if _navigate_to_aruco.Metaclass_NavigateToAruco_SendGoal_Request._TYPE_SUPPORT is None:
                _navigate_to_aruco.Metaclass_NavigateToAruco_SendGoal_Request.__import_type_support__()
            if _navigate_to_aruco.Metaclass_NavigateToAruco_SendGoal_Response._TYPE_SUPPORT is None:
                _navigate_to_aruco.Metaclass_NavigateToAruco_SendGoal_Response.__import_type_support__()


class NavigateToAruco_SendGoal(metaclass=Metaclass_NavigateToAruco_SendGoal):
    from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_SendGoal_Request as Request
    from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateToAruco_GetResult_Request(type):
    """Metaclass of message 'NavigateToAruco_GetResult_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_GetResult_Request(metaclass=Metaclass_NavigateToAruco_GetResult_Request):
    """Message class 'NavigateToAruco_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateToAruco_GetResult_Response(type):
    """Metaclass of message 'NavigateToAruco_GetResult_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__get_result__response

            from custom_interfaces.action import NavigateToAruco
            if NavigateToAruco.Result.__class__._TYPE_SUPPORT is None:
                NavigateToAruco.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_GetResult_Response(metaclass=Metaclass_NavigateToAruco_GetResult_Response):
    """Message class 'NavigateToAruco_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'custom_interfaces/NavigateToAruco_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_interfaces', 'action'], 'NavigateToAruco_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Result
        self.result = kwargs.get('result', NavigateToAruco_Result())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Result
            assert \
                isinstance(value, NavigateToAruco_Result), \
                "The 'result' field must be a sub message of type 'NavigateToAruco_Result'"
        self._result = value


class Metaclass_NavigateToAruco_GetResult(type):
    """Metaclass of service 'NavigateToAruco_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__navigate_to_aruco__get_result

            from custom_interfaces.action import _navigate_to_aruco
            if _navigate_to_aruco.Metaclass_NavigateToAruco_GetResult_Request._TYPE_SUPPORT is None:
                _navigate_to_aruco.Metaclass_NavigateToAruco_GetResult_Request.__import_type_support__()
            if _navigate_to_aruco.Metaclass_NavigateToAruco_GetResult_Response._TYPE_SUPPORT is None:
                _navigate_to_aruco.Metaclass_NavigateToAruco_GetResult_Response.__import_type_support__()


class NavigateToAruco_GetResult(metaclass=Metaclass_NavigateToAruco_GetResult):
    from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_GetResult_Request as Request
    from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateToAruco_FeedbackMessage(type):
    """Metaclass of message 'NavigateToAruco_FeedbackMessage'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_to_aruco__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_to_aruco__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_to_aruco__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_to_aruco__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_to_aruco__feedback_message

            from custom_interfaces.action import NavigateToAruco
            if NavigateToAruco.Feedback.__class__._TYPE_SUPPORT is None:
                NavigateToAruco.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateToAruco_FeedbackMessage(metaclass=Metaclass_NavigateToAruco_FeedbackMessage):
    """Message class 'NavigateToAruco_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'custom_interfaces/NavigateToAruco_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['custom_interfaces', 'action'], 'NavigateToAruco_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Feedback
        self.feedback = kwargs.get('feedback', NavigateToAruco_Feedback())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Feedback
            assert \
                isinstance(value, NavigateToAruco_Feedback), \
                "The 'feedback' field must be a sub message of type 'NavigateToAruco_Feedback'"
        self._feedback = value


class Metaclass_NavigateToAruco(type):
    """Metaclass of action 'NavigateToAruco'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.action.NavigateToAruco')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__navigate_to_aruco

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from custom_interfaces.action import _navigate_to_aruco
            if _navigate_to_aruco.Metaclass_NavigateToAruco_SendGoal._TYPE_SUPPORT is None:
                _navigate_to_aruco.Metaclass_NavigateToAruco_SendGoal.__import_type_support__()
            if _navigate_to_aruco.Metaclass_NavigateToAruco_GetResult._TYPE_SUPPORT is None:
                _navigate_to_aruco.Metaclass_NavigateToAruco_GetResult.__import_type_support__()
            if _navigate_to_aruco.Metaclass_NavigateToAruco_FeedbackMessage._TYPE_SUPPORT is None:
                _navigate_to_aruco.Metaclass_NavigateToAruco_FeedbackMessage.__import_type_support__()


class NavigateToAruco(metaclass=Metaclass_NavigateToAruco):

    # The goal message defined in the action definition.
    from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Goal as Goal
    # The result message defined in the action definition.
    from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Result as Result
    # The feedback message defined in the action definition.
    from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from custom_interfaces.action._navigate_to_aruco import NavigateToAruco_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
