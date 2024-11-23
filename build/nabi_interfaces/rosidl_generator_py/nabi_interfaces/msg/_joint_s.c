// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from nabi_interfaces:msg/Joint.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "nabi_interfaces/msg/detail/joint__struct.h"
#include "nabi_interfaces/msg/detail/joint__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool nabi_interfaces__msg__joint__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("nabi_interfaces.msg._joint.Joint", full_classname_dest, 32) == 0);
  }
  nabi_interfaces__msg__Joint * ros_message = _ros_message;
  {  // can_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "can_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->can_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // joint_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_angle");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->joint_angle = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->velocity = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "acceleration");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->acceleration = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * nabi_interfaces__msg__joint__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Joint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("nabi_interfaces.msg._joint");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Joint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  nabi_interfaces__msg__Joint * ros_message = (nabi_interfaces__msg__Joint *)raw_ros_message;
  {  // can_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->can_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "can_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_angle
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->joint_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acceleration
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
