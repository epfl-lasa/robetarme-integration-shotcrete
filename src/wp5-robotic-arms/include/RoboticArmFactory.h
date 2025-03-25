#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "IRoboticArmBase.h"
#include "RoboticArmCobod.h"

class RoboticArmFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<IRoboticArmBase>()>;

  //TODO(lmunier) : Add compatibility with the other robotic arms
  RoboticArmFactory() {
    registerRobotArm("robetarme", []() { return std::make_unique<RoboticArmCobod>(); });
  }

  void registerRobotArm(std::string name, FactoryFunction function) { factoryFunctionRegistry[name] = function; }

  std::unique_ptr<IRoboticArmBase> createRoboticArm(std::string name) {
    auto it = factoryFunctionRegistry.find(name);

    if (it != factoryFunctionRegistry.end()) {
      return it->second();
    } else {
      throw std::runtime_error("Invalid name");
    }
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};