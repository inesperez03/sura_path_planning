#include <chrono>
#include <cstdint>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

class SimpleNavigator : public rclcpp::Node
{
public:
  using Plan = nav2_msgs::action::ComputePathToPose;
  using Follow = nav2_msgs::action::FollowPath;
  using FollowHandle = rclcpp_action::ClientGoalHandle<Follow>;

  SimpleNavigator() : Node("simple_navigator")
  {
    planner_ = rclcpp_action::create_client<Plan>(this, "compute_path_to_pose");
    follower_ = rclcpp_action::create_client<Follow>(this, "follow_path");
    goals_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr goal) {navigate(*goal);});
    createGoalMarker();
    RCLCPP_INFO(get_logger(), "Ready for goals on /goal_pose or the RViz marker");
  }

private:
  void createGoalMarker()
  {
    using Control = visualization_msgs::msg::InteractiveMarkerControl;
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "blueboat/map";
    marker.name = "nav2_goal";
    marker.pose.orientation.w = 1.0;
    marker.scale = 0.7;

    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = 0.22;
    sphere.scale.y = 0.22;
    sphere.scale.z = 0.22;
    sphere.color.r = 0.2F;
    sphere.color.g = 0.8F;
    sphere.color.b = 1.0F;
    sphere.color.a = 1.0F;

    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.45;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.08;
    arrow.color.r = 1.0F;
    arrow.color.g = 0.7F;
    arrow.color.b = 0.1F;
    arrow.color.a = 1.0F;

    Control visual;
    visual.name = "goal_menu";
    visual.interaction_mode = Control::MENU;
    visual.always_visible = true;
    visual.markers.push_back(sphere);
    visual.markers.push_back(arrow);
    marker.controls.push_back(visual);

    Control move_x;
    move_x.name = "move_x";
    move_x.orientation.w = 0.70710678;
    move_x.orientation.x = 0.70710678;
    move_x.orientation_mode = Control::FIXED;
    move_x.interaction_mode = Control::MOVE_AXIS;
    marker.controls.push_back(move_x);

    Control move_y;
    move_y.name = "move_y";
    move_y.orientation.w = 0.70710678;
    move_y.orientation.z = 0.70710678;
    move_y.orientation_mode = Control::FIXED;
    move_y.interaction_mode = Control::MOVE_AXIS;
    marker.controls.push_back(move_y);

    Control rotate;
    rotate.name = "rotate_yaw";
    rotate.interaction_mode = Control::ROTATE_AXIS;
    rotate.orientation_mode = Control::FIXED;
    rotate.orientation.w = 0.70710678;
    rotate.orientation.y = 0.70710678;
    marker.controls.push_back(rotate);

    marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "nav2/interactive_markers", this);
    marker_server_->insert(marker);
    menu_handler_ = std::make_unique<interactive_markers::MenuHandler>();
    menu_handler_->insert("Go to pose", [this](const auto & feedback) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "blueboat/map";
      goal.header.stamp = now();
      goal.pose = feedback->pose;
      goal.pose.position.z = 0.0;
      navigate(goal);
    });
    menu_handler_->apply(*marker_server_, marker.name);
    marker_server_->applyChanges();
  }

  void navigate(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (pose.header.frame_id.empty()) {
      RCLCPP_ERROR(get_logger(), "Goal has no frame_id");
      return;
    }
    ++goal_number_;
    const auto number = goal_number_;
    if (follow_handle_) {
      follower_->async_cancel_goal(follow_handle_);
      follow_handle_.reset();
    }
    if (!planner_->wait_for_action_server(std::chrono::seconds(1)) ||
      !follower_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(get_logger(), "Nav2 planner or controller is not ready; send the goal again");
      return;
    }

    Plan::Goal request;
    request.goal = pose;
    request.planner_id = "GridBased";
    request.use_start = false;
    rclcpp_action::Client<Plan>::SendGoalOptions options;
    options.goal_response_callback = [this, number](auto handle) {
      if (number == goal_number_ && !handle) {
        RCLCPP_ERROR(get_logger(), "Planner rejected the goal");
      }
    };
    options.result_callback = [this, number](const auto & result) {
      if (number != goal_number_) {return;}
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !result.result || result.result->path.poses.empty())
      {
        RCLCPP_ERROR(get_logger(), "No path to the destination");
        return;
      }
      follow(result.result->path, number);
    };
    planner_->async_send_goal(request, options);
  }

  void follow(const nav_msgs::msg::Path & path, uint64_t number)
  {
    Follow::Goal request;
    request.path = path;
    request.controller_id = "FollowPath";
    request.goal_checker_id = "goal_checker";
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback = [this, number](FollowHandle::SharedPtr handle) {
      if (number != goal_number_) {
        if (handle) {follower_->async_cancel_goal(handle);}
        return;
      }
      follow_handle_ = handle;
      if (!handle) {RCLCPP_ERROR(get_logger(), "Controller rejected the path");}
    };
    options.result_callback = [this, number](const auto & result) {
      if (number != goal_number_) {return;}
      follow_handle_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Destination reached");
      } else {
        RCLCPP_ERROR(get_logger(), "Path following stopped");
      }
    };
    follower_->async_send_goal(request, options);
  }

  rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp_action::Client<Follow>::SharedPtr follower_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  std::unique_ptr<interactive_markers::MenuHandler> menu_handler_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  FollowHandle::SharedPtr follow_handle_;
  uint64_t goal_number_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNavigator>());
  rclcpp::shutdown();
  return 0;
}
