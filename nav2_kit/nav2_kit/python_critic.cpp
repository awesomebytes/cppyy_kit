#include <Python.h>
#include "python_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include <cmath>
#include <dlfcn.h>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
namespace nav2_kit_python {
namespace {
// This bridge retains the process interpreter until process exit. Other plugins
// may own Python objects or threads, so unloading a controller must not finalize it.
void ensure_python() {
  static std::mutex initialization_mutex;
  std::lock_guard<std::mutex> lock(initialization_mutex);
  const auto expected_version = std::to_string(PY_MAJOR_VERSION) + "." +
      std::to_string(PY_MINOR_VERSION) + ".";
  if (std::string(Py_GetVersion()).find(expected_version) != 0)
    throw std::runtime_error("Existing Python runtime has a different major/minor version");
  if (Py_IsInitialized()) return;
  // Nav2 may load plugins locally. CPython extension modules require the Python
  // symbols in the global loader namespace. Retain this handle until exit.
  static void * python_library = []() {
    Dl_info bound;
    const char * path = dladdr(reinterpret_cast<void *>(&Py_IsInitialized), &bound) && bound.dli_fname ?
        bound.dli_fname : NAV2_KIT_PYTHON_LIBRARY;
    return dlopen(path, RTLD_NOW | RTLD_GLOBAL);
  }();
  if (!python_library) {
    const char * error = dlerror();
    throw std::runtime_error(error ? error : "Could not load Python runtime globally");
  }
  PyConfig config;
  PyConfig_InitIsolatedConfig(&config);
  config.install_signal_handlers = 0;
  PyStatus status = PyConfig_SetBytesString(&config, &config.home, NAV2_KIT_PYTHON_HOME);
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
    std::string error = text ? text : "Python critic call failed";
    PyErr_Clear();
    throw std::runtime_error(error);
  }
  return Object(object);
}
}  // namespace
PythonCritic::~PythonCritic() {
  if (Py_IsInitialized()) {
    Gil gil;
    Py_XDECREF(score_); Py_XDECREF(prepare_); Py_XDECREF(policy_);
  }
}
void PythonCritic::onInit() {
  auto node = node_.lock();
  if (!node) throw std::runtime_error("Python critic node expired");
  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  for (const auto & item : std::vector<std::pair<std::string, std::string>>{
      {"python_module", ""}, {"python_class", ""}, {"python_path", ""}, {"python_options", "{}"}}) {
    if (!node->has_parameter(prefix + item.first))
      node->declare_parameter(prefix + item.first, item.second);
  }
  if (!node->has_parameter(prefix + "lookahead_distance"))
    node->declare_parameter(prefix + "lookahead_distance", lookahead_);
  lookahead_ = node->get_parameter(prefix + "lookahead_distance").as_double();
  const auto path = node->get_parameter(prefix + "python_path").as_string();
  if (path.empty() || path.front() != '/' || !std::isfinite(lookahead_) || lookahead_ <= 0)
    throw std::runtime_error("Python critic needs absolute python_path and positive lookahead_distance");
  ensure_python();
  Gil gil;
  auto directory = checked(PyUnicode_FromString(path.c_str()));
  auto search_path = PySys_GetObject("path");
  if (!search_path || PyList_Insert(search_path, 0, directory.get()) < 0) checked(nullptr);
  auto module = checked(PyImport_ImportModule(node->get_parameter(prefix + "python_module").as_string().c_str()));
  auto cls = checked(PyObject_GetAttrString(module.get(), node->get_parameter(prefix + "python_class").as_string().c_str()));
  auto instance = checked(PyObject_CallNoArgs(cls.get()));
  auto json = checked(PyImport_ImportModule("json"));
  auto options = checked(PyObject_CallMethod(json.get(), "loads", "s",
    node->get_parameter(prefix + "python_options").as_string().c_str()));
  if (!PyDict_Check(options.get())) throw std::runtime_error("python_options must be a JSON object");
  auto result = checked(PyObject_CallMethod(instance.get(), "configure", "O", options.get()));
  if (result.get() == Py_False) throw std::runtime_error("Python critic configure rejected options");
  auto prepare = checked(PyObject_GetAttrString(instance.get(), "prepare"));
  auto score = checked(PyObject_GetAttrString(instance.get(), "score"));
  if (!PyCallable_Check(prepare.get()) || !PyCallable_Check(score.get()))
    throw std::runtime_error("Python critic prepare and score must be callable");
  Py_XDECREF(score_); Py_XDECREF(prepare_); Py_XDECREF(policy_);
  policy_ = instance.release(); prepare_ = prepare.release(); score_ = score.release();
}
bool PythonCritic::prepare(const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &, const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & path) {
  prepared_ = false;
  try {
    if (!policy_ || path.poses.empty()) throw std::runtime_error("Missing Python critic policy or plan");
    size_t nearest = 0;
    double distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const double d = std::hypot(path.poses[i].x - pose.x, path.poses[i].y - pose.y);
      if (d < distance) { distance = d; nearest = i; }
    }
    double heading = pose.theta, walked = 0;
    size_t target = nearest;
    for (size_t i = nearest + 1; i < path.poses.size(); ++i) {
      const double dx = path.poses[i].x - path.poses[i-1].x;
      const double dy = path.poses[i].y - path.poses[i-1].y;
      const double segment = std::hypot(dx, dy);
      if (segment > 1e-6) { target = i; walked += segment; }
      if (walked >= lookahead_) break;
    }
    // Follow the look-ahead point instead of the final grid segment's tangent.
    // Individual horizontal/diagonal segments can alternate after replanning.
    const double dx = path.poses[target].x - pose.x;
    const double dy = path.poses[target].y - pose.y;
    if (std::hypot(dx, dy) > 1e-6) heading = std::atan2(dy, dx);
    const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
    if (!std::isfinite(pose.theta) || !std::isfinite(heading) || !std::isfinite(goal_distance))
      throw std::runtime_error("Nonfinite Python critic context");
    Gil gil;
    auto result = checked(PyObject_CallFunction(prepare_, "ddd", pose.theta, heading, goal_distance));
    if (result.get() == Py_False) throw std::runtime_error("Python critic prepare rejected context");
    prepared_ = true;
    return true;
  } catch (const std::exception & error) {
    throw dwb_core::IllegalTrajectoryException(name_, error.what());
  }
}
double PythonCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) {
  try {
    if (!prepared_ || trajectory.poses.empty()) throw std::runtime_error("Python critic has no prepared trajectory");
    const auto & velocity = trajectory.velocity;
    const double heading = trajectory.poses.back().theta;
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
        !std::isfinite(velocity.theta) || !std::isfinite(heading))
      throw std::runtime_error("Nonfinite trajectory");
    Gil gil;
    auto result = checked(PyObject_CallFunction(score_, "dddd", velocity.x, velocity.y, velocity.theta, heading));
    if (result.get() == Py_None) throw std::runtime_error("Python policy rejected trajectory");
    const double value = PyFloat_AsDouble(result.get());
    if (PyErr_Occurred()) checked(nullptr);
    if (!std::isfinite(value) || value < 0)
      throw std::runtime_error("Python critic score must be finite and nonnegative");
    return value;
  } catch (const std::exception & error) {
    throw dwb_core::IllegalTrajectoryException(name_, error.what());
  }
}
}  // namespace nav2_kit_python
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_kit_python::PythonCritic, dwb_core::TrajectoryCritic)
