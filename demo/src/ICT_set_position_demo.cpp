// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// std header
#include <sstream>
#include <cstdlib>
#include <queue>

// TM Driver header
#include "tm_msgs/SetPositions.h"
#include "tm_msgs/FeedbackState.h"

class MyROSNode {
  public:
      MyROSNode() {
          ros::NodeHandle nh;

          // Create two subscribers
          state_sub = nh.subscribe("/feedback_states", 1000, &MyROSNode::state_callback, this);
          start_sub = nh.subscribe("/start_cmd", 10, &MyROSNode::start_callback, this);
          timer = nh.createTimer(ros::Duration(0.1), &MyROSNode::check_callback, this);

          // Create a client for a custom service
          custom_service_client = nh.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");

          flag = 0;
          std::vector<float> position1 = {0, 0, 0, 0, 0, 0};
          std::vector<float> position2 = {0.1,
                                0.1,
                                0.1,
                                0.1,
                                0.1,
                                0.1};
          point_set.push(position1);
          point_set.push(position2);

      }

      void state_callback(const tm_msgs::FeedbackState::ConstPtr& msg) {
        for(int i = 0;i < 6;i++)
        {
          pos_state[i] = msg->joint_pos[i];
          // std::cout << pos_state[i] << " "; 
          // ROS_INFO("Received vel data: %f", pos_state[i]);
        }
        // std::cout << std::endl;
      }
      void start_callback(const std_msgs::Int32::ConstPtr& msg) {
        if(msg->data == 1)
        {
          flag = 1;
        }
      }

      void check_callback(const ros::TimerEvent& event){
        if(flag == 1){
          flag = 2;
          if(!point_set.empty()){
            SendPositionCmd(point_set.front());
            target_point = point_set.front();
            point_set.pop();
          }
          else{
            flag = 0;
          }
            
        }
        if(flag == 2){
          bool a = true;
          for(int i = 0; i < 6; i++){
            if(std::abs(pos_state[i] - target_point[i]) > 0.0001)
              a = false;
          }
          if(a == true)
            flag = 1;
        }
        
      }

  private:

    void SendPositionCmd(std::vector<float> position_cmd){
      tm_msgs::SetPositions srv;
        srv.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
        for(auto i :position_cmd)
            srv.request.positions.push_back(i);

        srv.request.velocity = 0.4;//rad/s
        srv.request.acc_time = 0.2;
        srv.request.blend_percentage = 10;
        srv.request.fine_goal  = false;

        if (custom_service_client.call(srv))                             
        {
          if (srv.response.ok) ROS_INFO_STREAM("SetPositions to robot");
          else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
        }
        else
        {
          ROS_ERROR_STREAM("Error SetPositions to robot");
          exit;
        }
        ROS_INFO("Start to go to point");
    }

    ros::Subscriber state_sub;
    ros::Subscriber start_sub;
    ros::Timer timer;
    ros::ServiceClient custom_service_client;
    float pos_state[6] = {10};
    float goal_pos_state[6] = {10};
    int flag;
    std::queue<std::vector<float>> point_set;
    std::vector<float> target_point;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "my_ros_node");
    MyROSNode node;
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    // ros::spin();
}


