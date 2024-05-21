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
 
using namespace std;
using namespace grid_map;

class OGM
{
public:
  OGM(const std::string& robot_namespace, ros::NodeHandle _nh)
  {
    nh = _nh;
    odom_sub = nh.subscribe(robot_namespace+"/odom", 10, &OGM::odomCallback, this);
    las_mes_sub = nh.subscribe(robot_namespace+"/las_mes", 10, &OGM::laserCallback, this);
    ogm_publisher = nh.advertise<grid_map_msgs::GridMap>(robot_namespace+"/ogm", 10, true);
    x2y2client = nh.serviceClient<move_robot::xycoord>(robot_namespace+"/occlu_xy");
  }

  void Compute_OGM(const std::string& robot_namespace)
  {
    // Create grid map.
  GridMap map({"position"}); //layer to store the current positions
  map.setFrameId("odom");
  map.setGeometry(Length(map_x_lim, map_y_lim), map_res);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",map.getLength().x(), map.getLength().y(),map.getSize()(0), map.getSize()(1));
  map.add("prev_position", Matrix::Zero(map.getSize()(0), map.getSize()(1))); //layer to store the previous positions

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
    map.add("position", Matrix::Zero(map.getSize()(0), map.getSize()(1)));

      if(isLaserCallbackCalled)
      {
        // std::cout<<"laser is called"<<"\n";
          
          for (int i = 0; i < laser_mes_len; ++i){

              Position temp_position;
            //   std::cout<<dist_x[i]<< "\n"<< odom_x<<"\n";
              temp_position[0] = dist_x[i]+odom_x;
              temp_position[1] = dist_y[i]+odom_y;
              Index temp_index;

              double X1 = dist_x[i];
              double Y1 = dist_y[i];

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
                for (grid_map::LineIterator iterator(map, start, end); !iterator.isPastEnd(); ++iterator) 
                {
                    map.at("position", *iterator) = P_prior;
                    // std::cout<<"map data for position is getting updated"<<"\n";
                }
              }

              map.at("position", temp_index) = P_occ;
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
    map.add("velocities", ((map.get("position")-map.get("prev_position"))*(time_num/elapsed_time)) );
    map.add("prev_position", map.get("position") + Matrix::Zero(map.getSize()(0), map.getSize()(1)));
    // std::cout << "previous positions after finding velocities should match with positions"<< map.get("prev_position") << "\n";
    // std::cout << "velocities: \n"<< map.get("velocities") << "\n";


    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    // GridMapRosConverter::toOccupancyGrid(map, "position", 0, 1, message);
    ogm_publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    
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
  ros::Subscriber las_mes_sub;
  ros::Publisher ogm_publisher;
  ros::ServiceClient x2y2client;
  move_robot::xycoord coord_srv;

  float map_x_lim = sqrt(2)*3.5; //3.5 is the max_range of laser scan. sqrt(2)*max_range will make the grid map to be circumscribed in a circle of radius max_range
  float map_y_lim = sqrt(2)*3.5;
  float map_res = 0.3;
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

  std::vector<double> dist;
  std::vector<double> dist_x;
  std::vector<double> dist_y;
  std::vector<double> ls_angles;
  float max_las_range = 0.0;
  bool isLaserCallbackCalled = false;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {   
        // std::cout<<"odomcallback is called"<<"\n";
        odom = *msg;

        // Extract the x and y position from the odometry message
        odom_x = odom.pose.pose.position.x;
        odom_y = odom.pose.pose.position.y;

        // Extract the orientation quaternion from the odometry message
        geometry_msgs::Quaternion quat = odom.pose.pose.orientation;

        // Convert the quaternion to roll, pitch, and yaw angles (in radians)
        tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3 m(tf_quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        odom_theta = yaw;
    }

    void laserCallback(const move_robot::las_mes::ConstPtr& msg)
    {   
        // std::cout<<"lasercallback is called"<<"\n";
        move_robot::las_mes ls = *msg;

        dist = ls.las_dist;
        // std::cout<<typeid(ls).name()<<"--->"<<typeid(msg).name()<<"--->"<<typeid(dist).name()<<"--->"<<typeid(ls.las_dist).name()<<"\n";
        dist_x = ls.las_dist_x;
        dist_y = ls.las_dist_y;
        ls_angles = ls.las_angles;
        isLaserCallbackCalled = true;
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
  while (!nh.hasParam(robot_namespace+"/service_server_created")){
    pass_rate.sleep();
  }
  
  if (nh.hasParam(robot_namespace+"/service_server_created")){
    ROS_INFO("service_server_created has been set");
  }
  else{
    ROS_WARN("service_server_created is not set");
  }

  OGM ogm_agent(robot_namespace, nh);
  ogm_agent.Compute_OGM(robot_namespace);

  return 0;
}
