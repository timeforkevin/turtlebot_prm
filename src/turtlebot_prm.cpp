//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 3
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include <visualization_msgs/Marker.h>

#include "a_star.h"


#define TAGID 0
#define K_STEER 1
#define K_SOFT 1
#define DIST_THRESH 0.25
#define MAP_WIDTH 100
#define NUM_POINTS 100
#define RESOLUTION 0.1
#define FRAND_TO(X) (static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(X))))


typedef Eigen::Matrix<float, 3, 1> Vector3f;


ros::Publisher marker_pub;
visualization_msgs::Marker points;

volatile bool first_pose = false;
node pose;


node_t* nodes_arr[NUM_POINTS]; // 3 for the waypoints

void generate_prm(const nav_msgs::OccupancyGrid& msg) {
    // generate_points
    int count = 0;
    while (count < (NUM_POINTS-3)) {
        float rand_x = FRAND_TO(MAP_WIDTH*RESOLUTION);
        float rand_y = FRAND_TO(MAP_WIDTH*RESOLUTION);

        if(msg.data[round(rand_y/RESOLUTION)*MAP_WIDTH + round(rand_x/RESOLUTION)] != 100) {
            node_t *node = new node_t();
            node->x = rand_x;
            node->y = rand_y;
            node->yaw = 0;
            nodes_arr[count] = node;
            count += 1;
            ROS_INFO("x:%f, y:%f", rand_x, rand_y);
        }
    }

    // add waypoints
    node_t* waypoint1 = new node_t();
    waypoint1->x =  4;
    waypoint1->y = 0;
    waypoint1->yaw = 0;

    nodes_arr[97] = waypoint1;

    node_t* waypoint2 = new node_t();
    waypoint2->x =  8;
    waypoint2->y = 4;
    waypoint2->yaw = 3.14;

    nodes_arr[98] = waypoint2;

    node_t* waypoint3 = new node_t();
    waypoint3->x =  8;
    waypoint3->y = 0;
    waypoint3->yaw = -1.57;

    nodes_arr[99] = waypoint3;

    for (int i = 0; i < NUM_POINTS; i++) {
        geometry_msgs::Point p;
        p.x = nodes_arr[i]->x;
        p.y = nodes_arr[i]->y;
        points.points.push_back(p);
    }
       marker_pub.publish(points);




}

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  //This function is called when a new position message is received
  pose.x = msg.pose.pose.position.x; // Robot X psotition
  pose.y = msg.pose.pose.position.y; // Robot Y psotition
  pose.yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
  first_pose = true;
}

//Example of drawing a curve
void drawCurve(int k)
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p);

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);
   }

   //publish new curve
   marker_pub.publish(lines);

}


//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
    generate_prm(msg);
}

float steering_angle(node *start, node *end, node *curr_pose, float &v_f) {
  float path_ang = atan2(end->y - start->y,
                         end->x - start->x);
  float curr_ang = atan2(curr_pose->y - start->y,
                         curr_pose->x - start->x);
  float diff_ang = path_ang - curr_ang;
  float err_d = DISTANCE_NODES(start,curr_pose)*sin(diff_ang);// cross-track error
  float err_h = path_ang - curr_pose->yaw;
  v_f /= (1+10*fabs(err_d));
  return err_h + atan(K_STEER*err_d/(K_SOFT + v_f));
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    points.header.frame_id = "map";
    points.id = 0;
    points.ns = "particles";
    points.type = visualization_msgs::Marker::POINTS;
    points.action = 0;
    points.color.g = 1;
    points.color.a = 1;
    points.lifetime = ros::Duration(0);
    points.frame_locked = true;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.05;
    points.scale.y = 0.05;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    node nodes[4];
    nodes[0].x = 0;
    nodes[0].y = 0;
    nodes[1].x = 1;
    nodes[1].y = 0;
    nodes[2].x = 0;
    nodes[2].y = 1;
    nodes[3].x = 1;
    nodes[3].y = 2;
    SET_EDGE(&nodes[0],&nodes[1]);
    SET_EDGE(&nodes[0],&nodes[2]);
    // SET_EDGE(&nodes[0],&nodes[3]);
    SET_EDGE(&nodes[1],&nodes[2]);
    SET_EDGE(&nodes[1],&nodes[3]);
    SET_EDGE(&nodes[2],&nodes[3]);
    path out_path;
    bool found = a_star(&nodes[0], &nodes[3], out_path);
    for (node* n : out_path.nodes) {
      ROS_INFO("x: %f y: %f", n->x, n->y);
    }
    std::vector<node*>::iterator path_it = out_path.nodes.begin();
    node *prev_node = &pose;

    first_pose = false;
    ROS_INFO("Waiting for First Pose");
    while (ros::ok() && !first_pose) {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages
    }
    ROS_INFO("Got First Pose");

    while (ros::ok())
    {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages
      //Main loop code goes here:

      float v_f = 0.1;

      float ang = steering_angle(prev_node, *path_it, &pose, v_f);
      vel.linear.x = v_f; // set linear speed
      vel.angular.z = ang; // set angular speed

      velocity_publisher.publish(vel); // Publish the command velocity

      if (DISTANCE_NODES(&pose,*path_it) < DIST_THRESH) {
        prev_node = *path_it;
        path_it++;
        if (path_it == out_path.nodes.end()) {
          break;
        }
        ROS_INFO("FROM X:%f Y:%f", prev_node->x, prev_node->y);
        ROS_INFO("TO X:%f Y:%f", (*path_it)->x, (*path_it)->y);
      }
    }

    return 0;
}
