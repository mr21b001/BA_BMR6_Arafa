#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

// Struct to define goal positions (x, y, theta)
struct GoalPosition {
    double x;
    double y;
    double th;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "husky_goals");
    ros::NodeHandle nh;

    // Alias for the MoveBaseClient type
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // Wait for the move_base action server to come up
    while (!ac.waitForServer(ros::Duration(30.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
        ros::Duration(0.1).sleep();
    }

    // Create a TF2 listener for frame transformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Define a list of 5 goals with (x, y, yaw) values, including the start position at (0, 0, 0)
    GoalPosition goals[5] = {
        {6.0, 5.0, 0.0},   // Start Position
        {7.0, -6.0, 0.0},   // Goal 1
        {0.0, -5.0, -1.57},  // Goal 2 (90 degrees turn)
        {0.0, 0.0, 0.0},  // Goal 3 (180 degrees turn)
       // {0.0, 0.0, 0}  // Goal 4 (-90 degrees turn)
    };

    // Create a goal message
    move_base_msgs::MoveBaseGoal mygoal;
    mygoal.target_pose.header.frame_id = "map";  // Goals will be sent in the "map" frame

    // Iterate through each goal and send it to move_base
    for (int i = 0; i < 5; i++) {
        ROS_INFO("Sending goal %d: (x: %f, y: %f, th: %f)", i, goals[i].x, goals[i].y, goals[i].th);

        // Set goal position
        mygoal.target_pose.pose.position.x = goals[i].x;
        mygoal.target_pose.pose.position.y = goals[i].y;

        // Set goal orientation using quaternion (convert yaw to quaternion)
        mygoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goals[i].th);

        // Send the goal to move_base
        mygoal.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(mygoal);

        // Wait for the result
        bool finished_before_timeout = ac.waitForResult(ros::Duration(500.0));

        // Check if the goal was reached successfully
        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Goal %d reached. Action finished: %s", i, state.toString().c_str());
        }
        else {
            ROS_WARN("Goal %d not reached before timeout. Canceling the goal.", i);
            ac.cancelGoal();
            break;  // Stop if a goal fails
        }

        ros::Duration(2.0).sleep();  // Small delay before sending the next goal
    }

    return 0;
}
