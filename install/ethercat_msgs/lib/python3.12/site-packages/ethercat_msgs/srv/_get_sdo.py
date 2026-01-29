# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ethercat_msgs:srv/GetSdo.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetSdo_Request(type):
    """Metaclass of message 'GetSdo_Request'."""

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
            module = import_type_support('ethercat_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ethercat_msgs.srv.GetSdo_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_sdo__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_sdo__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_sdo__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_sdo__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_sdo__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetSdo_Request(metaclass=Metaclass_GetSdo_Request):
    """Message class 'GetSdo_Request'."""

    __slots__ = [
        '_master_id',
        '_slave_position',
        '_sdo_index',
        '_sdo_subindex',
        '_sdo_data_type',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'master_id': 'int16',
        'slave_position': 'uint16',
        'sdo_index': 'uint16',
        'sdo_subindex': 'uint8',
        'sdo_data_type': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.master_id = kwargs.get('master_id', int())
        self.slave_position = kwargs.get('slave_position', int())
        self.sdo_index = kwargs.get('sdo_index', int())
        self.sdo_subindex = kwargs.get('sdo_subindex', int())
        self.sdo_data_type = kwargs.get('sdo_data_type', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.master_id != other.master_id:
            return False
        if self.slave_position != other.slave_position:
            return False
        if self.sdo_index != other.sdo_index:
            return False
        if self.sdo_subindex != other.sdo_subindex:
            return False
        if self.sdo_data_type != other.sdo_data_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def master_id(self):
        """Message field 'master_id'."""
        return self._master_id

    @master_id.setter
    def master_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'master_id' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'master_id' field must be an integer in [-32768, 32767]"
        self._master_id = value

    @builtins.property
    def slave_position(self):
        """Message field 'slave_position'."""
        return self._slave_position

    @slave_position.setter
    def slave_position(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'slave_position' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'slave_position' field must be an unsigned integer in [0, 65535]"
        self._slave_position = value

    @builtins.property
    def sdo_index(self):
        """Message field 'sdo_index'."""
        return self._sdo_index

    @sdo_index.setter
    def sdo_index(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'sdo_index' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'sdo_index' field must be an unsigned integer in [0, 65535]"
        self._sdo_index = value

    @builtins.property
    def sdo_subindex(self):
        """Message field 'sdo_subindex'."""
        return self._sdo_subindex

    @sdo_subindex.setter
    def sdo_subindex(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'sdo_subindex' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sdo_subindex' field must be an unsigned integer in [0, 255]"
        self._sdo_subindex = value

    @builtins.property
    def sdo_data_type(self):
        """Message field 'sdo_data_type'."""
        return self._sdo_data_type

    @sdo_data_type.setter
    def sdo_data_type(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'sdo_data_type' field must be of type 'str'"
        self._sdo_data_type = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetSdo_Response(type):
    """Metaclass of message 'GetSdo_Response'."""

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
            module = import_type_support('ethercat_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ethercat_msgs.srv.GetSdo_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_sdo__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_sdo__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_sdo__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_sdo__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_sdo__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetSdo_Response(metaclass=Metaclass_GetSdo_Response):
    """Message class 'GetSdo_Response'."""

    __slots__ = [
        '_success',
        '_sdo_return_message',
        '_sdo_return_value_string',
        '_sdo_return_value',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'sdo_return_message': 'string',
        'sdo_return_value_string': 'string',
        'sdo_return_value': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.sdo_return_message = kwargs.get('sdo_return_message', str())
        self.sdo_return_value_string = kwargs.get('sdo_return_value_string', str())
        self.sdo_return_value = kwargs.get('sdo_return_value', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.sdo_return_message != other.sdo_return_message:
            return False
        if self.sdo_return_value_string != other.sdo_return_value_string:
            return False
        if self.sdo_return_value != other.sdo_return_value:
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
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def sdo_return_message(self):
        """Message field 'sdo_return_message'."""
        return self._sdo_return_message

    @sdo_return_message.setter
    def sdo_return_message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'sdo_return_message' field must be of type 'str'"
        self._sdo_return_message = value

    @builtins.property
    def sdo_return_value_string(self):
        """Message field 'sdo_return_value_string'."""
        return self._sdo_return_value_string

    @sdo_return_value_string.setter
    def sdo_return_value_string(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'sdo_return_value_string' field must be of type 'str'"
        self._sdo_return_value_string = value

    @builtins.property
    def sdo_return_value(self):
        """Message field 'sdo_return_value'."""
        return self._sdo_return_value

    @sdo_return_value.setter
    def sdo_return_value(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'sdo_return_value' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'sdo_return_value' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._sdo_return_value = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetSdo_Event(type):
    """Metaclass of message 'GetSdo_Event'."""

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
            module = import_type_support('ethercat_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ethercat_msgs.srv.GetSdo_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_sdo__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_sdo__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_sdo__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_sdo__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_sdo__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetSdo_Event(metaclass=Metaclass_GetSdo_Event):
    """Message class 'GetSdo_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<ethercat_msgs/GetSdo_Request, 1>',
        'response': 'sequence<ethercat_msgs/GetSdo_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['ethercat_msgs', 'srv'], 'GetSdo_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['ethercat_msgs', 'srv'], 'GetSdo_Response'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from ethercat_msgs.srv import GetSdo_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, GetSdo_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'GetSdo_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from ethercat_msgs.srv import GetSdo_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, GetSdo_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'GetSdo_Response'"
        self._response = value


class Metaclass_GetSdo(type):
    """Metaclass of service 'GetSdo'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ethercat_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ethercat_msgs.srv.GetSdo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_sdo

            from ethercat_msgs.srv import _get_sdo
            if _get_sdo.Metaclass_GetSdo_Request._TYPE_SUPPORT is None:
                _get_sdo.Metaclass_GetSdo_Request.__import_type_support__()
            if _get_sdo.Metaclass_GetSdo_Response._TYPE_SUPPORT is None:
                _get_sdo.Metaclass_GetSdo_Response.__import_type_support__()
            if _get_sdo.Metaclass_GetSdo_Event._TYPE_SUPPORT is None:
                _get_sdo.Metaclass_GetSdo_Event.__import_type_support__()


class GetSdo(metaclass=Metaclass_GetSdo):
    from ethercat_msgs.srv._get_sdo import GetSdo_Request as Request
    from ethercat_msgs.srv._get_sdo import GetSdo_Response as Response
    from ethercat_msgs.srv._get_sdo import GetSdo_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
