#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

//订阅 mavros/state 的回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char** argv)
{
    //设置编码格式支持中文
    setlocale(LC_ALL,"");

    //初始化
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //订阅 mavros/state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    //公告 mavros/setpoint_position/local
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    //连接服务 mavros/cmd/arming
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    //连接服务 mavros/set_mode
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //初始化循环频率对象
    ros::Rate rate(20.0);

    //一直等待 current_state.connected 为 true，表示我们已经得到了 mavros/state 消息，飞控已经成功连接 mavros
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();//执行消息回调
        rate.sleep();//睡眠保证频率
    }

    //实例化一个 geometry_msgs::PoseStamped 类，用于发布，并把 Z 轴赋予 2 米
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //实例化mavros_msgs::SetMode类，offb_set_mode.request.custom_mode赋值为"OFFBOARD"，表示我们要进入OFFBOARD模式
    //apm固件也许为“GUILDED”
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //实例化 mavros_msgs::CommandBool，arm_cmd.request.value = true;表示我们要解锁，= false 为加锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //得到当前系统时间
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) 
        //如果模式还不为 offboard，并且距离上一次执行进入 offboard 命令已经过去了 5秒，进入这里
        {
            if (set_mode_client.call(offb_set_mode) &&offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard模式启动完成！");//如果请求服务成功且服务返回执行成功打印消息
            }
            last_request = ros::Time::now();//更新最后一次请求服务时间
        }
        else
        {
            //如果已经处于 offboard 模式进入这里
            if (!current_state.armed &&(ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                //如果没有打开保险，并且距离上次此请求解锁已经过去 5 秒，进入这里
                if (arming_client.call(arm_cmd) &&arm_cmd.response.success)
                {
                    //如果请求服务成功且服务返回执行成功打印消息
                    ROS_INFO("飞行器已解锁");
                }
                last_request = ros::Time::now();//更新最后一次请求解锁时间阿木实验室(AMOVLAB) wwww.amovauto.com
            }
        }

        local_pos_pub.publish(pose);//发布位置消息
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

/*启动命令

BY:键盘超人
https://github.com/996sky/test_myros

首先
make px4_sitl_default gazebo
启动仿真

roscore 启动ROS核心
打开QGC
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
启动MAVROS
rosrun fly takeoff 包名 C++节点
*/