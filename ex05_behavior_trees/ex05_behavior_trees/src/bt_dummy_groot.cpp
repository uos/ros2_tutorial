
#include "behaviortree_cpp_v3/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;
  std::string workspace_path = "/home/amock/ceres_ws";
  factory.registerFromPlugin(workspace_path + "/install/ex05_bt_plugins/lib/libbt_plugin_dummy_nodes.so");


  auto tree = factory.createTreeFromText(xml_text);

  // enabling this line we can connect the groot monitor
  BT::PublisherZMQ publisher_zmq(tree);

  tree.tickRootWhileRunning();

  return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/

