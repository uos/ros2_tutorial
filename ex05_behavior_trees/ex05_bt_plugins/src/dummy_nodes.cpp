#include <ex05_bt_plugins/dummy_nodes.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    DummyNodes::RegisterNodes(factory);
}

namespace DummyNodes
{

BT::NodeStatus CheckBattery()
{
  std::cout << "Checking battery ..." << std::endl;
  std::this_thread::sleep_for(3s);
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckTemperature()
{
  std::cout << "Checking temperature ..." << std::endl;
  std::this_thread::sleep_for(3s);
  std::cout << "[ Temperature: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SayHello()
{
  std::cout << "Robot says: Hello World" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::open()
{
  std::cout << "Started to open the gripper ..." << std::endl;
  std::this_thread::sleep_for(3s);
  _opened = true;
  std::cout << "Finished." << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close()
{
  std::cout << "Started to close the gripper ..." << std::endl;
  std::this_thread::sleep_for(3s);
  _opened = false;
  std::cout << "Finished." << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachObject::tick()
{
  std::cout << "ApproachObject: " << this->name() << " ..." << std::endl;
  std::this_thread::sleep_for(5s);
  std::cout << "Finished." << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomething::tick()
{
  auto msg = getInput<std::string>("message");
  if (!msg)
  {
    throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomethingSimple(BT::TreeNode &self)
{
  auto msg = self.getInput<std::string>("message");
  if (!msg)
  {
      throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

} // namespace DummyNodes