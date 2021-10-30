#include <ros/ros.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;//当前状态

void callBack(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
    ROS_INFO("hello world!");
    //printf("message get callback!\n");
}//callBack回调函数

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, callBack);

    ros::Rate rate(20.0);//频率为20hz

    while (ros::ok())
    {
        ros::spinOnce();//调用一次回调函数
        rate.sleep();//等待1/20秒
    }
    return 0;
}

/*启动命令
首先
make px4_sitl_default gazebo
启动仿真

roscore 启动ROS核心
打开QGC
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
启动MAVROS
rosrun 包名 C++节点
*/

