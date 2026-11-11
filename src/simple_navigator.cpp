#include <chrono>
#include <cstdint>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

class SimpleNavigator : public rclcpp::Node
{
public:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using NavigateHandle = rclcpp_action::ClientGoalHandle<Navigate>;
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using ComputePathHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

  SimpleNavigator() : Node("simple_navigator")
  {
    navigator_ = rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
    planner_ = rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
    goals_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr goal) {stageGoal(*goal);});
    createGoalMarker();
    RCLCPP_INFO(get_logger(), "Set a goal, then use Compute path and Go to pose on the RViz marker");
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
    marker_server_->insert(marker, [this](const auto & feedback) {
      if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "blueboat/map";
        goal.header.stamp = now();
        goal.pose = feedback->pose;
        staged_goal_ = goal;
        invalidatePreview();
      }
    });
    menu_handler_ = std::make_unique<interactive_markers::MenuHandler>();
    menu_handler_->insert("Compute path", [this](const auto & feedback) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "blueboat/map";
      goal.header.stamp = now();
      goal.pose = feedback->pose;
      staged_goal_ = goal;
      computePath();
    });
    go_menu_entry_ = menu_handler_->insert("Go to pose", [this](const auto & feedback) {
      (void)feedback;
      if (!path_ready_) {
        RCLCPP_WARN(get_logger(), "Compute path first and wait for the preview");
        return;
      }
      auto goal = staged_goal_;
      goal.header.stamp = now();
      navigate(goal);
    });
    menu_handler_->setVisible(go_menu_entry_, false);
    menu_handler_->apply(*marker_server_, marker.name);
    marker_server_->applyChanges();
  }

  void stageGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (goal.header.frame_id != "blueboat/map") {
      RCLCPP_ERROR(get_logger(), "Goal must be in blueboat/map, got '%s'", goal.header.frame_id.c_str());
      return;
    }
    staged_goal_ = goal;
    staged_goal_.pose.position.z = 0.0;
    marker_server_->setPose("nav2_goal", staged_goal_.pose);
    marker_server_->applyChanges();
    invalidatePreview();
    RCLCPP_INFO(get_logger(), "Goal staged; use Compute path on the RViz marker");
  }

  void invalidatePreview()
  {
    ++preview_revision_;
    path_ready_ = false;
    menu_handler_->setVisible(go_menu_entry_, false);
    menu_handler_->reApply(*marker_server_);
    marker_server_->applyChanges();
    nav_msgs::msg::Path empty_plan;
    empty_plan.header.frame_id = "blueboat/map";
    empty_plan.header.stamp = now();
    plan_publisher_->publish(empty_plan);
  }

  void computePath()
  {
    invalidatePreview();
    const auto revision = preview_revision_;
    if (!planner_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "Nav2 planner is not ready; try Compute path again");
      return;
    }

    ComputePath::Goal request;
    request.goal = staged_goal_;
    request.goal.pose.position.z = 0.0;
    request.planner_id = "GridBased";
    request.use_start = false;
    rclcpp_action::Client<ComputePath>::SendGoalOptions options;
    options.goal_response_callback = [this, revision](ComputePathHandle::SharedPtr handle) {
      if (!handle && revision == preview_revision_) {
        RCLCPP_ERROR(get_logger(), "Planner rejected the goal");
      }
    };
    options.result_callback = [this, revision](const auto & result) {
      if (revision != preview_revision_) {return;}
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !result.result || result.result->path.poses.empty())
      {
        RCLCPP_ERROR(get_logger(), "Could not compute a path to this pose");
        return;
      }
      plan_publisher_->publish(result.result->path);
      path_ready_ = true;
      menu_handler_->setVisible(go_menu_entry_, true);
      menu_handler_->reApply(*marker_server_);
      marker_server_->applyChanges();
      RCLCPP_INFO(get_logger(), "Path ready (%zu poses); use Go to pose on the RViz marker",
        result.result->path.poses.size());
    };
    planner_->async_send_goal(request, options);
    RCLCPP_INFO(get_logger(), "Computing path...");
  }

  void navigate(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (pose.header.frame_id.empty()) {
      RCLCPP_ERROR(get_logger(), "Goal has no frame_id");
      return;
    }
    ++goal_number_;
    const auto number = goal_number_;
    if (navigate_handle_) {
      navigator_->async_cancel_goal(navigate_handle_);
      navigate_handle_.reset();
    }
    if (!navigator_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "Nav2 navigator is not ready; send the goal again");
      return;
    }

    Navigate::Goal request;
    request.pose = pose;
    rclcpp_action::Client<Navigate>::SendGoalOptions options;
    options.goal_response_callback = [this, number](NavigateHandle::SharedPtr handle) {
      if (number != goal_number_) {
        if (handle) {navigator_->async_cancel_goal(handle);}
        return;
      }
      navigate_handle_ = handle;
      if (!handle) {RCLCPP_ERROR(get_logger(), "Navigator rejected the goal");}
    };
    options.result_callback = [this, number](const auto & result) {
      if (number != goal_number_) {return;}
      navigate_handle_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Destination reached");
      } else {
        RCLCPP_ERROR(get_logger(), "Navigation stopped");
      }
    };
    navigator_->async_send_goal(request, options);
  }

  rclcpp_action::Client<Navigate>::SharedPtr navigator_;
  rclcpp_action::Client<ComputePath>::SharedPtr planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
  std::unique_ptr<interactive_markers::MenuHandler> menu_handler_;
  interactive_markers::MenuHandler::EntryHandle go_menu_entry_{};
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  NavigateHandle::SharedPtr navigate_handle_;
  geometry_msgs::msg::PoseStamped staged_goal_;
  uint64_t preview_revision_{0};
  bool path_ready_{false};
  uint64_t goal_number_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNavigator>());
  rclcpp::shutdown();
  return 0;
}
