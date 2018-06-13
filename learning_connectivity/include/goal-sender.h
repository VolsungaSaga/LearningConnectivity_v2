#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <cstdlib>

void read_trajectory(std::string);
move_base_msgs::MoveBaseGoal makeGoalMessage(double x, double y);
void PIDController(std::vector<double> des_xy_coords, std::vector<double> parameters,
                    std::vector<double> u );
void getCustomTrajectory();