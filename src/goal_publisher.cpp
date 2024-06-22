# include <ros/ros.h>
# include <actionlib/client/simple_action_client.h>
# include <move_base_msgs/MoveBaseAction.h>
# include <tf/transform_datatypes.h>
# include <fstream>
# include <iostream>
# include <string>
# include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x;
    double y;
    double theta;
};

//read the goals from the file csv from waypoints.csv file
std::vector<Goal> readGoals(const std::string &filename) {
    std::vector<Goal> goals; // create a vector of type Goal to store the goals for the robot
    std::ifstream file(filename); //open the file

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file");
        return goals;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token; 
        std::vector<std::string> tokens; 
        //split the line by commas: x,y,theta and store in the vector token temporarily
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token); 
        }
        Goal goal; //create a goal struct

        //convert the string to double and store in the goal struct
        goal.x = std::stod(tokens[0]); 
        goal.y = std::stod(tokens[1]); 
        goal.theta = std::stod(tokens[2]);
        goals.push_back(goal); //store the goal struct in the vector goals
        ROS_INFO("Read goal: x=%f, y=%f, theta=%f", goal.x, goal.y, goal.theta);
    }
    file.close(); //close the file
    return goals; //return the vector of goals
}

//publish the goals to the robot
void publishGoals(const std::vector<Goal> &goals) {
    MoveBaseClient ac("move_base", true); //create a move base client
    ac.waitForServer(); //wait for the server to start

    for (Goal goal : goals) {
        move_base_msgs::MoveBaseGoal move_goal; //create a move base goal
        move_goal.target_pose.header.frame_id = "map"; //set the frame id
        move_goal.target_pose.header.stamp = ros::Time::now(); //set the time
        move_goal.target_pose.pose.position.x = goal.x; //set the x position
        move_goal.target_pose.pose.position.y = goal.y; //set the y position
        ROS_INFO("Sending goal: x=%f, y=%f, theta=%f", goal.x, goal.y, goal.theta);
        move_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal.theta); //set the orientation
            ac.sendGoal(move_goal);
            ac.waitForResult(); //wait for the result
               if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached successfully");
        } else {
            ROS_WARN("Failed to reach goal");
        }
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher"); //initialize the node
    ros::NodeHandle nh("~"); //create a node handle

    std::string filename; //create a string to store the file path
    nh.param<std::string>("csv_path", filename, "src/waypoints.csv"); //get the file path from the parameter server


    std::vector<Goal> goals = readGoals(filename); //read the goals from the file
    
    publishGoals(goals); //publish the goals to the robot
    return 0;
}