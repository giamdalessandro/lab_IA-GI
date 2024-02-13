#include <sstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

    std::string robotname = "robot_0";
    std::ostringstream ss;
    ss << "barbablu_gotopose_" << robotname;
    std::string nodename = ss.str();
    ros::init(argc, argv, nodename);

    std::string movebase_topic = "/"+robotname+"/move_base";

    MoveBaseClient ac(movebase_topic, true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for move_base action server to come up");
    }

    // Read time
    double secs =ros::Time::now().toSec();
    while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
        ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
        ros::Duration(1.0).sleep();
        secs =ros::Time::now().toSec();
    }

    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (!listener.waitForTransform("/map", "/robot_0/base_link", ros::Time(0), ros::Duration(1.0))){
        ROS_INFO("Transform not ok...");
        sleep(1);
    }
    listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
    float original_x = transform.getOrigin().x();
    float original_y = transform.getOrigin().y();
    float x = 0, y = 0;
    ROS_INFO("Got original pos: %.2f,%.2f", original_x, original_y);


    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 1.54;
    goal.target_pose.pose.position.y = -3.48;
    goal.target_pose.pose.orientation.z = 180;
    goal.target_pose.pose.orientation.w = 180;


    ROS_INFO("Sending goal %.2f %.2f %.2f %.2f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);
    ac.sendGoal(goal);

    int i = 0;
    while (!ac.waitForResult(ros::Duration(1.0))) {
        listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        ROS_INFO("Current pos: %.2f,%.2f", x, y);
        i++;
        if (i == 10){
            ac.cancelAllGoals();
            ROS_INFO("Goal cancelled. Coming back to original position.");
        }
    }
    sleep(1);
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = original_x;   //0;
    goal.target_pose.pose.position.y = original_y;   //1.8;
    goal.target_pose.pose.orientation.z = 180;
    goal.target_pose.pose.orientation.w = 180;

    ROS_INFO("Sending goal %.2f %.2f", original_x, original_y);
    ac.sendGoal(goal);

    while (!ac.waitForResult(ros::Duration(5.0))){
        listener.lookupTransform("/map", "/robot_0/base_link", ros::Time(0), transform);
        x = transform.getOrigin().x(), y = transform.getOrigin().y();
        ROS_INFO("Current position: %.2f,%.2f", x, y);
    }

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base reached the goal position.");
    else
        ROS_INFO("The base failed to reach the goal for some reason");

    return 0;
}

