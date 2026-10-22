#include <Python.h>
#include "python_controller.hpp"
#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <memory>
#include <set>
#include <stdexcept>

#ifndef CONTROL_KIT_PYTHON_HOME
#error "Build this bridge with the Python prefix used for its headers and library"
#endif

namespace control_kit_python {
namespace {
// This bridge retains the process interpreter until process exit. Other plugins
// may own Python objects or threads, so unloading a controller must not finalize it.
void ensure_python() {
  static std::mutex initialization_mutex;
  std::lock_guard<std::mutex> lock(initialization_mutex);
  // Gazebo loads plugins locally. CPython extension modules require the Python
  // symbols in the global loader namespace. Retain this handle until exit.
  static void * python_library = []() {
    Dl_info bound;
    const char * path = dladdr(reinterpret_cast<void *>(&Py_IsInitialized), &bound) && bound.dli_fname ?
        bound.dli_fname : CONTROL_KIT_PYTHON_LIBRARY;
    return dlopen(path, RTLD_NOW | RTLD_GLOBAL);
  }();
  if (!python_library) throw std::runtime_error(dlerror());
  const auto expected_version = std::to_string(PY_MAJOR_VERSION) + "." +
      std::to_string(PY_MINOR_VERSION) + ".";
  if (std::string(Py_GetVersion()).find(expected_version) != 0)
    throw std::runtime_error("Existing Python runtime has a different major/minor version");
  if (Py_IsInitialized()) return;
  PyConfig config;
  PyConfig_InitIsolatedConfig(&config);
  config.install_signal_handlers = 0;
  PyStatus status = PyConfig_SetBytesString(&config, &config.home, CONTROL_KIT_PYTHON_HOME);
  if (!PyStatus_Exception(status)) status = Py_InitializeFromConfig(&config);
  const std::string error = PyStatus_Exception(status) ?
      (status.err_msg ? status.err_msg : "Python initialization failed") : "";
  PyConfig_Clear(&config);
  if (!error.empty()) throw std::runtime_error(error);
  PyEval_SaveThread();
}
struct Gil {
  PyGILState_STATE state;
  Gil() : state(PyGILState_Ensure()) {}
  ~Gil() { PyGILState_Release(state); }
};
struct Decref { void operator()(PyObject * p) const { Py_XDECREF(p); } };
using Object = std::unique_ptr<PyObject, Decref>;
Object checked(PyObject * object) {
  if (!object) {
    // PyErr_Print treats SystemExit as a request to exit the host process.
    // Convert every Python exception, including SystemExit, into a ROS error.
    PyObject * type = nullptr, * value = nullptr, * traceback = nullptr;
    PyErr_Fetch(&type, &value, &traceback);
    PyErr_NormalizeException(&type, &value, &traceback);
    Object type_owner(type), value_owner(value), traceback_owner(traceback);
    Object message(value ? PyObject_Str(value) : nullptr);
    const char * text = message ? PyUnicode_AsUTF8(message.get()) : nullptr;
    std::string error = text ? text : "Python controller call failed";
    PyErr_Clear();
    throw std::runtime_error(error);
  }
  return Object(object);
}
Object numbers(const std::vector<double> & values) {
  auto result = checked(PyList_New(values.size()));
  for (size_t i = 0; i < values.size(); ++i) {
    auto value = checked(PyFloat_FromDouble(values[i]));
    PyList_SET_ITEM(result.get(), i, value.release());
  }
  return result;
}
void accepted(Object result) {
  if (result.get() == Py_False) throw std::runtime_error("Python controller rejected operation");
}
void optional_call(PyObject * controller, const char * method, PyObject * argument = nullptr) {
  Object callable(PyObject_GetAttrString(controller, method));
  if (!callable) {
    if (PyErr_ExceptionMatches(PyExc_AttributeError)) { PyErr_Clear(); return; }
    checked(nullptr);
  }
  accepted(checked(argument ? PyObject_CallOneArg(callable.get(), argument) :
      PyObject_CallNoArgs(callable.get())));
}
}  // namespace
using controller_interface::CallbackReturn;
using controller_interface::return_type;
PythonController::PythonController() = default;
PythonController::~PythonController() {
  subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  if (controller_ && Py_IsInitialized()) { Gil gil; Py_DECREF(controller_); }
}
CallbackReturn PythonController::on_init() {
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("python_module", "");
  auto_declare<std::string>("python_class", "");
  auto_declare<std::string>("python_path", "");
  auto_declare<std::string>("python_options", "{}");
  return CallbackReturn::SUCCESS;
}
CallbackReturn PythonController::on_configure(const rclcpp_lifecycle::State &) {
  subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  try {
    auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    const auto path = node->get_parameter("python_path").as_string();
    if (joints_.empty() || std::set<std::string>(joints_.begin(), joints_.end()).size() != joints_.size() ||
        std::any_of(joints_.begin(), joints_.end(), [](const auto & j) { return j.empty(); }) ||
        path.empty() || path.front() != '/') {
      throw std::runtime_error("Provide unique joints and an absolute python_path");
    }
    ensure_python();
    Gil gil;
    auto host_prefix = PySys_GetObject("prefix");
    const char * host_home = host_prefix ? PyUnicode_AsUTF8(host_prefix) : nullptr;
    if (!host_home) checked(nullptr);
    if (std::string(host_home) != CONTROL_KIT_PYTHON_HOME)
      RCLCPP_INFO(node->get_logger(), "Using existing Python prefix %s (plugin built with %s)",
          host_home, CONTROL_KIT_PYTHON_HOME);
    Py_CLEAR(controller_);
    auto directory = checked(PyUnicode_FromString(path.c_str()));
    auto search_path = PySys_GetObject("path");
    if (!search_path || PyList_Insert(search_path, 0, directory.get()) < 0) checked(nullptr);
    auto module = checked(PyImport_ImportModule(node->get_parameter("python_module").as_string().c_str()));
    auto cls = checked(PyObject_GetAttrString(module.get(), node->get_parameter("python_class").as_string().c_str()));
    auto instance = checked(PyObject_CallNoArgs(cls.get()));
    auto names = checked(PyList_New(joints_.size()));
    for (size_t i = 0; i < joints_.size(); ++i) {
      auto name = checked(PyUnicode_FromString(joints_[i].c_str()));
      PyList_SET_ITEM(names.get(), i, name.release());
    }
    auto json = checked(PyImport_ImportModule("json"));
    auto options = checked(PyObject_CallMethod(json.get(), "loads", "s",
        node->get_parameter("python_options").as_string().c_str()));
    if (!PyDict_Check(options.get())) throw std::runtime_error("python_options must encode a JSON object");
    accepted(checked(PyObject_CallMethod(instance.get(), "configure", "OO", names.get(), options.get())));
    state_index_.resize(joints_.size());
    command_index_.resize(joints_.size());
    controller_ = instance.release();
    subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          if (!set_target(msg->data)) RCLCPP_WARN(get_node()->get_logger(), "Rejected Python controller target");
        });
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return CallbackReturn::ERROR;
  }
}
controller_interface::InterfaceConfiguration PythonController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) result.names.push_back(joint + "/position");
  return result;
}
controller_interface::InterfaceConfiguration PythonController::state_interface_configuration() const {
  return command_interface_configuration();
}
bool PythonController::read_positions(std::vector<double> & positions) const {
  positions.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto value = state_interfaces_[state_index_[i]].get_optional();
    if (!value || !std::isfinite(*value)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unavailable or nonfinite position: %s", joints_[i].c_str());
      return false;
    }
    positions[i] = *value;
  }
  return true;
}
CallbackReturn PythonController::on_activate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  try {
    if (!controller_ || state_interfaces_.size() != joints_.size() || command_interfaces_.size() != joints_.size())
      throw std::runtime_error("Python controller interfaces are unavailable");
    for (size_t i = 0; i < joints_.size(); ++i) {
      auto name = joints_[i] + "/position";
      auto state = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
          [&](const auto & item) { return item.get_name() == name; });
      auto command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
          [&](const auto & item) { return item.get_name() == name; });
      if (state == state_interfaces_.end() || command == command_interfaces_.end())
        throw std::runtime_error("Python controller position interface is missing");
      state_index_[i] = std::distance(state_interfaces_.begin(), state);
      command_index_[i] = std::distance(command_interfaces_.begin(), command);
    }
    std::vector<double> positions;
    if (!read_positions(positions)) throw std::runtime_error("Invalid measured position");
    Gil gil;
    auto values = numbers(positions);
    optional_call(controller_, "activate", values.get());
    active_ = true;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return CallbackReturn::ERROR;
  }
}
CallbackReturn PythonController::on_deactivate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  try {
    if (controller_) { Gil gil; optional_call(controller_, "deactivate"); }
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return CallbackReturn::ERROR;
  }
}
bool PythonController::set_target(const std::vector<double> & target) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !controller_) return false;
  try {
    Gil gil;
    auto values = numbers(target);
    accepted(checked(PyObject_CallMethod(controller_, "set_target", "O", values.get())));
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return false;
  }
}
return_type PythonController::update(const rclcpp::Time &, const rclcpp::Duration & period) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    const double dt = period.seconds();
    if (!active_) throw std::runtime_error("Python controller update called while inactive");
    // Gazebo can repeat a simulation timestamp. Keep the previous command.
    if (dt == 0.0) return return_type::OK;
    if (!std::isfinite(dt) || dt < 0)
      throw std::runtime_error("Invalid controller update period: " + std::to_string(dt));
    std::vector<double> positions;
    if (!read_positions(positions)) return return_type::ERROR;
    std::vector<double> command(joints_.size());
    {
      Gil gil;
      auto values = numbers(positions);
      auto result = checked(PyObject_CallMethod(controller_, "update", "Od", values.get(), dt));
      auto sequence = checked(PySequence_Fast(result.get(), "update must return a position sequence"));
      if (PySequence_Fast_GET_SIZE(sequence.get()) != static_cast<Py_ssize_t>(joints_.size()))
        throw std::runtime_error("Python update returned the wrong number of positions");
      for (size_t i = 0; i < command.size(); ++i) {
        command[i] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(sequence.get(), i));
        if (PyErr_Occurred()) checked(nullptr);
        if (!std::isfinite(command[i])) throw std::runtime_error("Python update returned a nonfinite position");
      }
    }
    // Validate the entire Python result before writing any loaned interface.
    for (size_t i = 0; i < command.size(); ++i)
      if (!command_interfaces_[command_index_[i]].set_value(command[i]))
        throw std::runtime_error("Could not write position command: " + joints_[i]);
    return return_type::OK;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return return_type::ERROR;
  }
}
}  // namespace control_kit_python
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control_kit_python::PythonController, controller_interface::ControllerInterface)
