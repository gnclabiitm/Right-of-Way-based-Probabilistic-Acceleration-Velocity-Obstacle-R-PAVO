#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <move_robot/las_mes.h>
#include <iostream>
#include <list>
#include <move_robot/xycoord.h>
#include <sensor_msgs/LaserScan.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <string> 

using namespace std;
using namespace grid_map;

class OGM
{
public:
  OGM(const std::string& robot_namespace,ros::NodeHandle _nh)
 // OGM(const std::string& robot_namespace, ros::NodeHandle _nh)
  {
    nh = _nh;
    rb_ns = robot_namespace; 
    obstacle_sub = nh.subscribe(robot_namespace+"/obstacles", 10, &OGM::obstacleCallback, this);
    odom_sub = nh.subscribe(robot_namespace+"/odom", 10, &OGM::odomCallback, this);
    ogm_publisher = nh.advertise<grid_map_msgs::GridMap>(robot_namespace+"/ogm", 10, true);
    x2y2client = nh.serviceClient<move_robot::xycoord>(robot_namespace+"/occlu_xy");
  }

  // void Compute_OGM(const std::string& robot_namespace, const float&  initial_x, const float& initial_y, const float& initial_yaw)
  void Compute_OGM(const std::string& robot_namespace, const float& map_x_limit, const float& map_y_limit, const float& map_res)
  {
    // Create grid map.
  GridMap map({"occupancy"}); //layer to store the current positions
  map.setFrameId(rb_ns.substr(1,rb_ns.length())+"/odom");
//  map.setFrameId("odom");
//  map.setGeometry(Length(map_x_lim, map_y_lim), map_res);
  map.setGeometry(Length(map_x_limit, map_y_limit), map_res);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",map.getLength().x(), map.getLength().y(),map.getSize()(0), map.getSize()(1));

  double prev_time = 0.0;

  ros::Rate rate(30.0);
  ros::spinOnce();
  while (nh.ok())
   {
    // std::cout<<"Entered ogm while loop"<<"\n";
    ros::Time curr_time = ros::Time::now();
    map.move(Position(odom_x,odom_y));

    // Add data to grid map.
    ros::Time time = ros::Time::now();
    map.add("occupancy", Matrix::Zero(map.getSize()(0), map.getSize()(1)));
    map.add("velocity_x", Matrix::Zero(map.getSize()(0), map.getSize()(1)));
    map.add("velocity_y", Matrix::Zero(map.getSize()(0), map.getSize()(1)));
    map.add("position_x", Matrix::Zero(map.getSize()(0), map.getSize()(1)));
    map.add("position_y", Matrix::Zero(map.getSize()(0), map.getSize()(1)));


      if(isObstacleCallbackCalled)
      {
        // std::cout<<"laser is called"<<"\n";
          
          for (int i = 0; i < obs_pose.size(); ++i){
              Position centre(obs_pose[i].x,obs_pose[i].y);
              for (grid_map::CircleIterator iterator1(map, centre, obs_rad[i]); !iterator1.isPastEnd(); ++iterator1) 
                {
                    Position temp_position;
                    Index temp_index;
                    map.getPosition(*iterator1, temp_position);
                   // std::cout<<"temp_position "<<temp_position<<"\n";
                    double X1 = temp_position.x() - odom_x;
                    double Y1 = temp_position.y() - odom_y;
                   // std::cout<<"X1, Y1 "<< X1<<" "<<Y1<<"\n";
                    double occ_theta = atan2(Y1,X1);
                    double l = map.getLength().y()/2;
                    double w = map.getLength().x()/2;
                    //   std::cout<<"requesting for the service"<<"\n";
                    coord_srv.request.occ_theta = occ_theta;   
                    coord_srv.request.l = l-map_res/2;
                    coord_srv.request.w = w-map_res/2;
                    //   std::cout<<"got the service requested data"<<"\n";

                    if (map.isInside(temp_position))
                    {
                    //   std::cout<<"entered first if loop"<<"\n";
                    x2y2client.call(coord_srv);
                    double X2 =  coord_srv.response.x2;
                    double Y2 = coord_srv.response.y2;

                    Position occlu_temp_position;
                    occlu_temp_position[0] = X2+odom_x;
                    occlu_temp_position[1] = Y2+odom_y;
                    Index occlu_temp_index;
                    map.getIndex(temp_position,temp_index);

                    if(map.getIndex(occlu_temp_position, occlu_temp_index))
                    {
                        // std::cout<<"entered second if loop"<<"\n";
                        Index start(temp_index[0], temp_index[1]);
                        Index end(occlu_temp_index[0],occlu_temp_index[1]);
                        for (grid_map::LineIterator iterator2(map, start, end); !iterator2.isPastEnd(); ++iterator2) 
                        {   
                            if(map.at("occupancy", *iterator2) != P_occ)
                            {   Position occlu_position;
                                map.getPosition(*iterator2, occlu_position);
                                map.at("occupancy", *iterator2) = P_prior;
                                map.at("velocity_x", *iterator2) = obs_vel[i].x;
                                map.at("velocity_y", *iterator2) = obs_vel[i].y;
                                map.at("position_x", *iterator2) = occlu_position.x()-odom_x;
                                map.at("position_y", *iterator2) = occlu_position.y()-odom_y;
                            }
                            
                            // std::cout<<"map data for position is getting updated"<<"\n";
                        }
                    }

                    map.at("occupancy", temp_index) = P_occ;
                    map.at("velocity_x", temp_index) = obs_vel[i].x;
                    map.at("velocity_y", temp_index) = obs_vel[i].y;
                    map.at("position_x", temp_index) = temp_position.x()-odom_x;
                    map.at("position_y", temp_index) = temp_position.y()-odom_y;
                }            
            }

          }
  
        //   ros::spinOnce();
        }

    double elapsed_time = curr_time.toSec()-prev_time;
    prev_time = curr_time.toSec();
    double time_num = 1.0;
    // std::cout << "Final  positions of "<<robot_namespace<< "\n";
    // std::cout << map.get("position") << "\n";
    // std::cout << "difference in positions: \n"<< (map.get("position")-map.get("prev_position")) << "\n";
    // std::cout<< "elapsed time: "<< elapsed_time<<"\n";
    // map.add("velocities", ((map.get("position")-map.get("prev_position"))*(time_num/elapsed_time)) );
    // map.add("prev_position", map.get("position") + Matrix::Zero(map.getSize()(0), map.getSize()(1)));
    // std::cout << "previous positions after finding velocities should match with positions"<< map.get("prev_position") << "\n";
    // std::cout << "velocities: \n"<< map.get("velocities") << "\n";


    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    // GridMapRosConverter::toOccupancyGrid(map, "position", 0, 1, message);
    ogm_publisher.publish(message);
    // ROS_INFO_THROTTLE(10.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    
    ros::spinOnce();
    rate.sleep();

    if (! nh.hasParam(robot_namespace+"/ogm_created"))
    {
        nh.setParam(robot_namespace+"/ogm_created", true);
    }
    }

  }

private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber obstacle_sub;
  ros::Publisher ogm_publisher;
  ros::ServiceClient x2y2client;
  move_robot::xycoord coord_srv;
  
  std::string rb_ns;
  float initial_x;
  float initial_y;
  float initial_yaw;

  float map_x_limit;
  float map_y_limit;
  float map_res;
  
//  float map_x_lim = 0.7; //3.5 is the max_range of laser scan. sqrt(2)*max_range will make the grid map to be circumscribed in a circle of radius max_range
//  float map_y_lim = 0.7;
//  float map_res = 0.15;
  int laser_mes_len = 360;
  int n = 5;


  // Global variables
  float P_prior = 0.5;	// Prior occupancy probability
  float P_occ = 0.9;	// Probability that cell is occupied with total confidence
  float P_free = 0.1;	// Probability that cell is free with total confidence 

  nav_msgs::Odometry odom;
  double odom_x = 0.0;
  double odom_y = 0.0;
  double odom_theta = 0.0;

  std::vector<geometry_msgs::Point> obs_pose;
  std::vector<geometry_msgs::Vector3> obs_vel;
  std::vector<double> obs_rad;
  std::vector<double> obs_true_rad;
  bool isObstacleCallbackCalled = false;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {   
        // std::cout<<"odomcallback is called"<<"\n";
        odom = *msg;

        // Extract the x and y position from the odometry message
        odom_x = odom.pose.pose.position.x;//+initial_x;
        odom_y = odom.pose.pose.position.y;//+initial_y;

        // Extract the orientation quaternion from the odometry message
        geometry_msgs::Quaternion quat = odom.pose.pose.orientation;

        // Convert the quaternion to roll, pitch, and yaw angles (in radians)
        tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3 m(tf_quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        odom_theta =0.0;//yaw;//+initial_yaw;
    }



    void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
    {
        obstacle_detector::Obstacles obs = *msg;
        std::vector<geometry_msgs::Point> obs_pose_temp;
        std::vector<geometry_msgs::Vector3> obs_vel_temp;
        std::vector<double> obs_rad_temp;
        std::vector<double> obs_true_rad_temp;
        for (auto& it : obs.circles)
        {
            obs_pose_temp.push_back(it.center);
            obs_vel_temp.push_back(it.velocity);
            obs_rad_temp.push_back(it.radius);
            obs_true_rad_temp.push_back(it.true_radius);

        }
        
        obs_pose = obs_pose_temp;
        obs_vel = obs_vel_temp;
        obs_rad = obs_rad_temp;
        obs_true_rad = obs_true_rad_temp; 
        isObstacleCallbackCalled = true;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OGM_super_node");
  ros::NodeHandle nh("~"); // Use private node handle to access parameters
  std::string robot_namespace;
//   nh.getParam("robot_namespace", robot_namespace);
  robot_namespace = ros::this_node::getNamespace();

  sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(robot_namespace+"/scan", ros::Duration(10));
  ros::Rate pass_rate(0.5) ;
  pass_rate.sleep();
//  while (!nh.hasParam(robot_namespace+"/service_server_created")){
//    pass_rate.sleep();
//  }
  
  if (nh.hasParam(robot_namespace+"/service_server_created")){
    ROS_INFO("service_server_created has been set");
  }
  else{
    ROS_WARN("service_server_created is not set");
  }
 float map_x_limit, map_y_limit, map_res;
 nh.getParam("map_x_limit", map_x_limit);
 nh.getParam("map_y_limit", map_y_limit);
 nh.getParam("map_resolution", map_res);
 //float initial_x, initial_y, initial_yaw;
 // nh.getParam("initial_x",initial_x); 
 // nh.getParam("initial_y",initial_y);
 // nh.getParam("initial_yaw",initial_yaw);
 // std::cout<<"initial_x "<<initial_x<<"\n";
  OGM ogm_agent(robot_namespace, nh);

 // ogm_agent.Compute_OGM(robot_namespace);
 // ogm_agent.Compute_OGM(robot_namespace, initial_x,initial_y,initial_yaw);
  ogm_agent.Compute_OGM(robot_namespace,map_x_limit,map_y_limit,map_res);

  return 0;
}
