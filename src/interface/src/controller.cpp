#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/GetModelState.h>
#include <unistd.h>
#include <Eigen/Core>

#define pi 3.14159265358979

geometry_msgs::PoseStamped control_cmd;
void pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    control_cmd = *msg;
}

geometry_msgs::PoseStamped current_pos;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    current_pos = *msg;
     current_pos.header.frame_id = "world";
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    // std::cout << "get state message" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 10, state_cb);
    ros::Subscriber pose_cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>("/fast_planner/output_port/pose_command", 1, pose_sub_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, pose_cb);     //订阅飞机位置pose

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/iris_0/mavros/setpoint_position/local", 10);
    ros::Publisher current_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/iris_0/pose", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped> ("/iris_0/mavros/setpoint_velocity/cmd_vel", 10); 

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
    ros::ServiceClient state_client = nh.serviceClient<gazebo_msgs::GetModelState> ("/gazebo/get_model_state");

    ros::Rate rate(20.0);
    
    std::cout<<"waiting for connecting"<<std::endl;
    int num = 0;
    while(ros::ok() && !current_state.connected)
    {
        std::cout<<num++<<"     Connecting..."<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    std::cout<<"Connected"<<std::endl;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5;
    // publish the init pose for the pose topic
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout<<std::endl;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    gazebo_msgs::GetModelState iris_state;
    iris_state.request.model_name = "iris_0";
    iris_state.request.relative_entity_name = "world";
    while(ros::ok())
    {  	
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        state_client.call(iris_state);

        geometry_msgs::Point p = iris_state.response.pose.position;
        Eigen::Vector2d vec;
        geometry_msgs::TwistStamped _vel;
        double q0,q1,q2,q3,current_yaw;;
        _vel.header.frame_id = "world";
        _vel.header.stamp = ros::Time::now();
        q0 = current_pos.pose.orientation.w;
        q1 = current_pos.pose.orientation.x;
        q2 = current_pos.pose.orientation.y;
        q3 = current_pos.pose.orientation.z;
        current_yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
        double kp=0.5;
        vec.x() = control_cmd.pose.position.x - p.x;
        vec.y() = control_cmd.pose.position.y - p.y;
        if(vec.norm() > 0.5){
            double angle = atan2(vec.y(), vec.x());
            control_cmd.pose.orientation.w = cos(angle / 2);
            control_cmd.pose.orientation.x = 0;
            control_cmd.pose.orientation.y = 0;
            control_cmd.pose.orientation.z = sin(angle / 2);

            double angle_ = ((atan2(2*(cos(angle / 2)*sin(angle / 2)),(1-(2*(sin(angle / 2)*sin(angle / 2)))))) - current_yaw);
            _vel.twist.angular.z =2*angle_;
            _vel.twist.linear.x = kp*(control_cmd.pose.position.x - current_pos.pose.position.x);
            _vel.twist.linear.y = kp*(control_cmd.pose.position.y - current_pos.pose.position.y);
            _vel.twist.linear.z = kp*(control_cmd.pose.position.z - current_pos.pose.position.z);
        }   // change EulerAngle to orientation
        
        //if(vec.norm() > 5){
        //    control_cmd.pose.position.x = p.x + vec.x() / vec.norm() * 5;
        //    control_cmd.pose.position.y = p.y + vec.y() / vec.norm() * 5;
            //control_cmd.pose.position.z = p.z + vec.z() / vec.norm() * 2;
        //}
       
        pose = control_cmd;
        // local_vel_pub.publish(_vel);   // velocity control
        local_pos_pub.publish(pose);  //  publish the goal pose, position control
        current_pos_pub.publish(current_pos);     // publish the current pose of UAV in world tf
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
