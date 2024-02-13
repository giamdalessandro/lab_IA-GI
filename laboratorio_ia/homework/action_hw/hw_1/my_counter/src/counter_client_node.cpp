#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_counter/CounterAction.h>
using namespace std;
int main (int argc, char **argv){
  ros::init(argc, argv, "test_Counter");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<my_counter::CounterAction> ac("Counter", true);

  cerr << "Waiting for action server to start." << endl;
  ac.waitForServer();

  cerr << "Action server started, sending goal." << endl;
  // send a goal to the action
  my_counter::CounterGoal goal;
  goal.timeout = 20;
  ac.sendGoal(goal);
  sleep(5);
  goal.timeout = -1;
  cerr << "I'm going to stop in 2 sec..." << endl;
  sleep(2);
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout){
    actionlib::SimpleClientGoalState state = ac.getState();
    cerr << "Action finished: " << state.toString().c_str() << endl;
  }
  else
    cerr << "Action did not finish before the time out." << endl;
  return 0;
}

/// source ~/workspaces/labaigi_ws/devel/setup.bash
