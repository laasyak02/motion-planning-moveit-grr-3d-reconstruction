#include <memory>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <cmath>

int i;
Eigen::Quaterniond quaternion;
const std::string end_effector_link = "tool0";
int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  //rclcpp::executors::SingleThreadedExecutor executor;
  //executor.add_node(node);
  //std::thread spinner = std::thread([&executor]() { executor.spin(); });

  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "ur10_arm");

//It plans in the cartesian space.
/*
// Set a target Pose
auto const target_pose = []{

  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();
//const geometry_msgs::msg::Pose msg1 = msg;
move_group_interface.setPoseTarget(target_pose);
move_group_interface.setGoalTolerance(0.01);
//setPoseTarget
// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
RCLCPP_ERROR(logger, "Waiting");
sleep(5);
*/
/*
// Define multiple target poses
  std::vector<geometry_msgs::msg::Pose> target_poses = {
    []{
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = 0.28;
      pose.position.y = -0.2;
      pose.position.z = 0.5;
      return pose;
    }(),
    []{
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = 0.4;
      pose.position.y = -0.1;
      pose.position.z = 0.4;
      return pose;
    }(),
    []{
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = 0.2;
      pose.position.y = 0.1;
      pose.position.z = 0.3;
      return pose;
    }()
  };
*/
    // Circle parameters
    Eigen::Vector3d center(0.4, 0.0, 0.5); // Circle center
    double radius = 0.5;                   // Circle radius
    int num_points = 10;                  // Number of waypoints -10
    double epsilon = M_PI / 6.0; 
	
    // Fixed x-coordinate (circle in y-z plane)
    double fixed_x = center.x();

    // Waypoints for the vertical circle
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 0; i < num_points; ++i) {
        double theta = (-epsilon) + (M_PI + 2 * epsilon) * i / num_points;

        // Point on the semicircle
        Eigen::Vector3d point_on_circle(
            fixed_x,                           // Fixed x-coordinate
            center.y() + radius * cos(theta), // y-coordinate
            center.z() + radius * sin(theta)  // z-coordinate
        );

        // Direction vector to the center
        Eigen::Vector3d direction = center - point_on_circle;
        direction.normalize();

        // Define "up" vector
        Eigen::Vector3d up(1, 0, 0); // Adjust based on fixed x-axis
        if (direction.isApprox(up)) {
            up = Eigen::Vector3d(0, 1, 0); // Avoid singularity
        }

        // Compute side and recompute up for orthogonality
        Eigen::Vector3d side = up.cross(direction).normalized();
        up = direction.cross(side);

        // Form rotation matrix
        Eigen::Matrix3d rotation;
        rotation.col(0) = side;
        rotation.col(1) = up;
        rotation.col(2) = direction;

        // Convert to quaternion
        Eigen::Quaterniond quaternion(rotation);

        // Create pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = point_on_circle.x();
        pose.position.y = point_on_circle.y();
        pose.position.z = point_on_circle.z();
        pose.orientation.w = quaternion.w();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();

        waypoints.push_back(pose);
    }
  /*
  int i=1;
  
  // Iterate through the target poses
  for (const auto& target_pose : waypoints) {
    // Set the target pose
    move_group_interface.setPoseTarget(target_pose);
    move_group_interface.setGoalTolerance(0.01);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan if successful
    if (success) {
      RCLCPP_INFO(logger, "Executing plan for pose %d...", i);
      move_group_interface.execute(plan);
      rclcpp::sleep_for(std::chrono::seconds(2)); // Pause between movements
    } else {
      RCLCPP_ERROR(logger, "Planning failed for pose %d", i);
    }
    i=i+1;
  }
  */
  // Plan and execute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    move_group_interface.setMaxVelocityScalingFactor(0.01); // Reduce to 10% speed
    move_group_interface.setMaxAccelerationScalingFactor(0.01);
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 2.0, 6.28, trajectory, true
    );
    if (fraction < 1.0) {
        RCLCPP_ERROR(logger, "Could not compute full Cartesian path (%.2f%% completed)", fraction * 100);
    } else {
        RCLCPP_INFO(logger, "Cartesian path computed successfully!");
        move_group_interface.execute(trajectory);
        /*
        //Breaking down the trajectory into sub-trajectories and then executing them
        // Split trajectory into smaller segments
    	auto& points = trajectory.joint_trajectory.points;
        //RCLCPP_ERROR(logger, "Number of points %ld", points.size());
        for (size_t i = 0; i < points.size(); ++i) {
        // Create a sub-trajectory
        moveit_msgs::msg::RobotTrajectory sub_trajectory;
        sub_trajectory.joint_trajectory.header = trajectory.joint_trajectory.header;
        sub_trajectory.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
        sub_trajectory.joint_trajectory.points.push_back(points[i]);

        // Execute the sub-trajectory
        move_group_interface.execute(sub_trajectory);
        
        //Delay
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
        */
    }
  
  rclcpp::shutdown();
  return 0;
}

