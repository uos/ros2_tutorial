
#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>

// clang-format off
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="battery_ok"/>
            <OpenGripper    name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";

std::string create_path_to_plugin_lib_local_ws(
  std::string workspace_path,
  std::string package_name,
  std::string plugin_name)
{
  return workspace_path + "/install/" + package_name + "/lib/lib" + plugin_name + ".so";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // load plugin from install directory of your workspace
  std::string workspace_path = "/home/amock/ceres_ws";
  std::string package_name = "ex05_bt_plugins";
  std::string plugin_name = "bt_plugin_dummy_nodes";

  std::string path_to_plugin = create_path_to_plugin_lib_local_ws(workspace_path, package_name, plugin_name);
  factory.registerFromPlugin(path_to_plugin);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickRootWhileRunning();
  }
  
  return 0;
}

/* Expected output (in a loop):
*
  Loop begin
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/

