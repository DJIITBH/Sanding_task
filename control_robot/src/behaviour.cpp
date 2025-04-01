#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

//**********node class***************
// sync - synchronous task, blocking call unless this node is finished
class ApproachObject : public BT::SyncActionNode
{
public:
  explicit ApproachObject(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Approach Object: " << this->name() << std::endl;

    std::this_thread::sleep_for(5s);
    return BT::NodeStatus::SUCCESS;
  }
};


int main()
{
  BT::BehaviorTreeFactory factory;

//   class that inherits node from behaviour library
  factory.registerNodeType<ApproachObject>("ApproachObject");

  //Create Tree
  std::string pkg_share_path = ament_index_cpp::get_package_share_directory("control_robot");
  std::string tree_file = pkg_share_path + "/behaviour_trees/bt_tree.xml";
  auto tree = factory.createTreeFromFile(tree_file);
  
  //execute the tree
  tree.tickWhileRunning();

  return 0;
}