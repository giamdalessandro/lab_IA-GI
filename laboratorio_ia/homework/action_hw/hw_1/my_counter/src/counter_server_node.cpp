#include <iostream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_counter/CounterAction.h>  // Note: "Action" is appended
using namespace std;

class CounterAction{

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_counter::CounterAction> as_; // !!NodeHandle instance must be created before this line!!
  std::string action_name_;
  my_counter::CounterFeedback feedback_;
  my_counter::CounterResult result_;

public:
  CounterAction(std::string name) :
    as_(nh_, name, boost::bind(&CounterAction::executeCB, this, _1), false),
    action_name_(name){
      as_.start();
    }

  ~CounterAction(void){}

  void executeCB(const my_counter::CounterGoalConstPtr &goal){
    ros::Rate r(1); // 1 Hz
    bool success = true;

    cerr << "Timeout to " << goal->timeout << endl;
    for(feedback_.current = goal->timeout; feedback_.current > 0; feedback_.current--){
      if (as_.isPreemptRequested() || !ros::ok()) {    // check that preempt has not been requested by the client
        cerr << action_name_.c_str() << " preempted" << endl;
        as_.setPreempted(); // set the action state to preempted
        success = false;
        break;
      }
      if (goal->timeout < 0){
        cerr << "timeout stopped!" << endl;
        as_.setSucceeded(result_);
        break;
      }
      as_.publishFeedback(feedback_);   // publish the feedback
      cerr << feedback_.current << endl;
      sleep(1);    // 1 Hz
    }

    if(success){
      cerr<<action_name_.c_str()<<" succeded"<<endl;
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "Counter");

    cerr << "server alive & ready, babe" << endl;

    CounterAction Counter("Counter");
    ros::spin();

    return 0;
}
