// clang off
#include "ITaskBase.h"
// clang on

#include <functional>
#include <map>
#include <memory>
#include <string>

// #include "TaskShotcrete.h"
// #include "TaskSurfaceFinishing.h"
#include "TaskShotcreteKUL.h"
/**
 * @brief The TaskFactory class is responsible for creating instances of tasks based on their names.
 *
 * The TaskFactory class provides a mechanism to register different task types and their corresponding factory functions.
 * It allows creating instances of tasks by specifying their names and providing the necessary parameters.
 */
class TaskFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<ITaskBase>(double freq, std::string robotName)>;

  /**
   * @brief Constructs a TaskFactory object.
   *
   * The constructor initializes the factoryFunctionRegistry and registers the default task types.
   */
  TaskFactory() {
    registerTask("shotcreteKUL", [](double freq, std::string robotName) {
      return std::make_unique<TaskShotcreteKUL>(freq, robotName);
    });
  }

  /**
   * @brief Registers a task type with its corresponding factory function.
   *
   * @param name The name of the task type.
   * @param function The factory function that creates instances of the task type.
   */
  void registerTask(std::string name, FactoryFunction function) { factoryFunctionRegistry[name] = function; }

  /**
   * @brief Creates an instance of a task based on its name.
   *
   * @param name The name of the task type.
   * @param freq The frequency of the task.
   * @param robotName The name of the robot associated with the task.
   * @return A unique pointer to the created task instance.
   * @throws std::runtime_error if the specified task name is invalid.
   */
  std::unique_ptr<ITaskBase> createTask(std::string name, double freq, std::string robotName) {
    auto it = factoryFunctionRegistry.find(name);

    if (it != factoryFunctionRegistry.end()) {
      return it->second(freq, robotName);
    } else {
      throw std::runtime_error("Invalid name");
    }
  }

  /**
   * @brief Retrieves the list of registered task types.
   *
   * @return A vector containing the names of the registered task types.
   */
  std::vector<std::string> getTaskTypes() {
    std::vector<std::string> keys;

    for (const auto& pair : factoryFunctionRegistry) {
      keys.push_back(pair.first);
    }

    return keys;
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};