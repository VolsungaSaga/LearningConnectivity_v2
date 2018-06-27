
//Modified from wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
// Author: Robert Pierce (rtp4qg@virginia.edu)
#include "../include/goal-sender.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;

//Note: This probably needs to be generalized. Didn't quite work with ~/ , unless I made some mistake, which is entirely possible.
std::string traj_filename = "/home/robot6/catkin_ws/src/learning_connectivity/trajectory_rectified.csv";
std::vector<move_base_msgs::MoveBaseGoal> goals;


//Reads points from csv file into a vector of goals.
void read_trajectory(std::string filename){
  std::ifstream file;
  std::cout << filename.data() << std::endl;
  file.open(filename.data());
  //Using boost, getting csv values is relatively simple. Just define a seperator character...
  boost::char_separator<char> sep(",");

  if( file.is_open()){

    std::string row;

    while(getline(file, row)){
      // ... a Tokenizer object ...
      Tokenizer coordinates(row, sep);

      std::vector<double> coord_vec;
      // ... and iterate through!
      for(Tokenizer::iterator iter = coordinates.begin(); iter !=coordinates.end(); ++ iter){
        coord_vec.push_back(strtod(iter->c_str(), 0));
      }
      std::cout << coord_vec[0] << ", " << coord_vec[1] << std::endl; 

      move_base_msgs::MoveBaseGoal goal = makeGoalMessage(coord_vec[0], coord_vec[1]);
      goal.target_pose.header.frame_id = "world";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = coord_vec[0];
      goal.target_pose.pose.position.y = coord_vec[1];
      goal.target_pose.pose.orientation.w = 1.0;

      goals.push_back(goal);

      std::cout << "Size of Goals: " << goals.size() << std::endl;
    }
  }

  else{
    std::cout << "Could not open trajectory file: " << filename << std::endl;
  }
  
  file.close();



}

//In case push comes to shove and this planner ends up being no good either, we can use a PID controller of our own making.
void PIDController(std::vector<double> des_xy_coords, std::vector<double> parameters,
                    std::vector<double> u ){
  double Kp_forward;
  double Ki_forward;
  double Kd_forward;

  double Kp_angle;
  double Ki_angle;
  double Kd_angle;

}

//Helper function to construct a goal message object, given x and y coordinates.
move_base_msgs::MoveBaseGoal makeGoalMessage(double x, double y){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "world";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;

    return goal;

}

//Allows user to specify a custom trajectory on the command line, if no file is supplied. 
void getCustomTrajectory(){

  bool cont = true;
  char cont_char;
  int pointNum = 1;
  
  double goalX; double goalY;
  
  while(cont){
    std::cout << "Enter two numbers (X, Y) for Point " << pointNum << ", separated by a space:" << std::endl;

    std::cin >> goalX >> goalY;

    move_base_msgs::MoveBaseGoal goal = makeGoalMessage(goalX, goalY);
    goals.push_back(goal);

    std::cout << "Continue? (Y/N): ";
    std::cin >> cont_char;

    if(cont_char == 'N' or cont_char =='n'){ 
      cont = false;}
    else if(cont_char == 'Y' or cont_char =='y'){ 
      pointNum ++;
      continue;
    }
    else{
      std::cout << "Please enter 'Y/y' or 'N/n' for your choice" << std::endl;
    }
  }

}


int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_goals");

  ros::NodeHandle nh;

  //Read trajectory file, if specified.
  if(argc == 2){
    read_trajectory(argv[1]);
  }
  //If no trajectory file is specified, then 
  else if(argc == 1){
    getCustomTrajectory();
  }

  else{
    std::cout << "usage: rosrun learning_connectivity goal-sender [filepath]" << std::endl;
  }

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  std::cout << "Final Size of Goals:" << goals.size() << std::endl;
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //std::cout<< "Here!" << std::endl;

  for(int i = 0; i < goals.size(); i++){
      move_base_msgs::MoveBaseGoal goal = goals[i];

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Achieved goal point");
      else
        ROS_INFO("Did not achieve goal point");


}

  return 0;
}


