#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <signal.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <fstream>
#include <string>

move_base_msgs::MoveBaseGoal goal;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
ros::Publisher cmdVelPub;
ros::Publisher marker_pub;
int32_t file_line = 0;//路径点总行数


//$ 获取保存地址文件的行数
void got_lines(std::string& filename, int& lines) {
    char buffer[1024] = {0};  // 定义一个字节数组存放数据
    std::fstream out;
    int line = 1;
    out.open(filename, std::ios::in);
    while (!out.eof()) {
        line += 1;
        out.getline(buffer, 1024);
    }
    out.close();
    lines = line - 2;
    std::cout << "lines = " << lines << std::endl;
}

/*当接收到SIGINT信号后，进入此程序，关闭movebase导航*/
void shutdown(int sig) {
    cmdVelPub.publish(geometry_msgs::Twist());
    ros::Duration(1).sleep();  // sleep for a second
    ROS_INFO("nav_square.cpp enden!");
    ros::shutdown();
}



//% result
void activeCb() {
    ROS_INFO("Goal Received");
}

//% feedback
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    float x,y;
    x = feedback->base_position.pose.position.x;
    y = feedback->base_position.pose.position.y;

    tf2::Quaternion quation1, quation2;
    quation1.setW(feedback->base_position.pose.orientation.w);
    quation1.setX(feedback->base_position.pose.orientation.x);
    quation1.setY(feedback->base_position.pose.orientation.y);
    quation1.setZ(feedback->base_position.pose.orientation.z);

    quation2.setW(goal.target_pose.pose.orientation.w);
    quation2.setX(goal.target_pose.pose.orientation.x);
    quation2.setY(goal.target_pose.pose.orientation.y);
    quation2.setZ(goal.target_pose.pose.orientation.z);

    double roll1=0,pitch1=0,yaw1=0;
    double roll2=0,pitch2=0,yaw2=0;

    tf2::Matrix3x3 m1(quation1);
    tf2::Matrix3x3 m2(quation2);

    m1.getRPY(roll1,pitch1,yaw1);
    m2.getRPY(roll2,pitch2,yaw2);

    float distance = sqrt(pow((x-goal.target_pose.pose.position.x),2) + pow((y-goal.target_pose.pose.position.y),2));
    float angle = fabs(yaw1 - yaw2);
    printf("distance =%f\r\n",distance);
    printf("angle=%f\r\n",angle);
}

// ￥ 导航坐标点的名称
std::string filename = "/home/jhr/Desktop/pose.txt";

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_move_base");
    std::string topic = "/cmd_vel";
    ros::NodeHandle node;
    // Subscribe to the move base action server;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    signal(SIGINT, shutdown);
    ROS_INFO("move_base_square.cpp start...");

    // How big is the square we want the robot to navigate
    double square_size = 1.0;

    got_lines(filename, file_line);/*计算有几个导航点*/
    geometry_msgs::Pose pose_list[file_line];

    // ￥ 加载导航数据点
    char buffer[1024] = {0};
    std::fstream out;
    out.open(filename, std::ios::in);

    // ￥ 将坐标点加载到 pose_list
    for (int L = 0; L < file_line; L++) {
        int b = 0;
        out.getline(buffer, 1024);
        // ￥ 解析每行数据
        for (int a = 0; a < 7; a++) {
            std::string data;
            while (buffer[b] != ',') {
                data += buffer[b];
                b++;
            }
            b += 1;  //% 加1是隔开','
            // std::cout<<"data: "<<std::atof(data.c_str())<<std::endl;

            switch (a) {
                case 0:
                    pose_list[L].position.x = std::atof(data.c_str());
                case 1:
                    pose_list[L].position.y = std::atof(data.c_str());
                case 2:
                    pose_list[L].position.z = std::atof(data.c_str());
                case 3:
                    pose_list[L].orientation.w = std::atof(data.c_str());
                case 4:
                    pose_list[L].orientation.x = std::atof(data.c_str());
                case 5:
                    pose_list[L].orientation.y = std::atof(data.c_str());
                case 6:
                    pose_list[L].orientation.z = std::atof(data.c_str());
            }
        }
    }
    ROS_INFO("waiting for move_base action server");

    // wait 60 seconds for the action server to beconme available
    if (!ac.waitForServer(ros::Duration(60))) {
        ROS_INFO("Can't connected th move base server");
        return 1;
    }
    ROS_INFO("Connected to move base server");
    ROS_INFO("Starting navigation test");

    // Initialize a counter to track waypoints
    int count = 0;
    // Cycle through the four waypoints
    while ((count < file_line) && ros::ok()) {

        // Use the map frame to define goal poses
        goal.target_pose.header.frame_id = "map";

        // set the time stamp to now
        goal.target_pose.header.stamp = ros::Time::now();

        // set the goal pose to the i-th waypoint
        goal.target_pose.pose = pose_list[count];

        // start the robot moving toward the goal
        // send the goal pose to the movebaseaction server
        // ￥ 发布goal action
        ac.sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
        printf("x=%f\ny=%f\r\n",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        
        //$ 等待movebase的action服务，在180秒后是否发布result
        bool finished_within_time = ac.waitForResult(ros::Duration(180));

        // if we dont get there in time abort the goal
        if (!finished_within_time) {
            ac.cancelGoal();
            ROS_INFO("Timed out achieving goal");
        } else {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal succeeded");
            } else {
                ROS_INFO("the base failed for some reason");
            }
        }
        //for the next goal
        count += 1;

        //cycle pub
        if(count == file_line){
            count = 0;
        }

        ROS_ERROR("ready for the next goal\n");
    }
    ROS_INFO("move_base_square.cpp end");
    return 0;
}