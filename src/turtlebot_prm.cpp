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
#define RADIUS_THRESHOLD 7
#define FRAND_TO(X) (static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(X))))


typedef Eigen::Matrix<float, 3, 1> Vector3f;


ros::Publisher marker_pub;

volatile bool first_pose = false;
node pose;
std::vector<node*> nodes_arr; // 3 for the waypoints

short sgn(int x ) { return x >=0 ? 1 : -1; }

void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
          d += inc2;
          if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

void generate_nodes(const nav_msgs::OccupancyGrid& msg) {
    int count = 0;
    while (count < NUM_POINTS) {
        float rand_x = FRAND_TO(MAP_WIDTH*RESOLUTION);
        float rand_y = FRAND_TO(MAP_WIDTH*RESOLUTION);

        if(msg.data[round(rand_y/RESOLUTION)*MAP_WIDTH + round(rand_x/RESOLUTION)] != 100) {
            node *new_node = new node();
            new_node->x = rand_x;
            new_node->y = rand_y;
            new_node->yaw = 0;
            nodes_arr.push_back(new_node);
            count += 1;
        }
    }
}

void generate_edges() {
    for(int i = 0; i < nodes_arr.size(); i++) {
        for(int j = 0; j < nodes_arr.size(); j++) {
            if (i==j) continue;

            if (DISTANCE_NODES(nodes_arr.at(i), nodes_arr.at(j)) < RADIUS_THRESHOLD) {
                SET_EDGE(nodes_arr.at(i), nodes_arr.at(j));
            }
        }
    }
}

bool collision_detected(node* node_1, node* node_2, const nav_msgs::OccupancyGrid& msg) {
    std::vector<int> line_x;
    std::vector<int> line_y;
    bresenham(node_1->x, node_1->y, node_2->x, node_2->y, line_x, line_y);
    while (!line_x.empty()) {
        int next_x = line_x.back();
        int next_y = line_y.back();
        line_x.pop_back();
        line_y.pop_back();

        if(msg.data[round(next_y/RESOLUTION)*MAP_WIDTH + round(next_x/RESOLUTION)] == 100) {
            return true;
        }
    }
    return false;
}

bool find_shortest_path(node* waypoint1, node* waypoint2, path& out_path, const nav_msgs::OccupancyGrid& msg) {
    bool valid_path_found = false;

    while (!valid_path_found) {
        bool found = a_star(waypoint1, waypoint2, out_path);
        if (!found) {
            break;
        } else {
            for(int i = 1; i < out_path.nodes.size(); i++) {
                if(collision_detected(out_path.nodes.at(i-1), out_path.nodes.at(i), msg)) {
                    REMOVE_EDGE(out_path.nodes.at(i-1), out_path.nodes.at(i))
                    continue;
                }
            }
            valid_path_found = true;

        }
    }

    return valid_path_found;
}

void publish_graph() {
    int id = 0;
    visualization_msgs::Marker milestone_msg;
    milestone_msg.id = id;
    milestone_msg.header.frame_id = "map";
    milestone_msg.header.stamp = ros::Time();
    milestone_msg.ns = "localization";
    milestone_msg.action = visualization_msgs::Marker::ADD;
    milestone_msg.type = visualization_msgs::Marker::POINTS;
    milestone_msg.scale.x = 0.2;
    milestone_msg.scale.y = 0.2;
    milestone_msg.scale.z = 0.2;
    milestone_msg.color.a = 1.0;
    milestone_msg.color.r = 0.0;
    milestone_msg.color.g = 1.0;
    milestone_msg.color.b = 0.0;

      // Add the milestones as markers and publish their edges
     for (node* curr_node : nodes_arr) {
       geometry_msgs::Point p;
       p.x = curr_node->x;
       p.y = curr_node->y;
       milestone_msg.points.push_back(p);

       //Pubish the edges
       for (node* node : curr_node->adj)
       {
            id++; // Increase the ID
            visualization_msgs::Marker edge_msg;
            edge_msg.id = id;
            edge_msg.header.frame_id = "map";
            edge_msg.header.stamp = ros::Time();
            edge_msg.ns = "localization";
            edge_msg.action = visualization_msgs::Marker::ADD;
            edge_msg.type = visualization_msgs::Marker::LINE_STRIP;
            edge_msg.scale.x = 0.02;
            edge_msg.scale.y = 0.02;
            edge_msg.scale.z = 0.02;
            edge_msg.color.a = 1.0;
            edge_msg.color.r = 0.0;
            edge_msg.color.g = 0.0;
            edge_msg.color.b = 1.0;

            geometry_msgs::Point m1;
            m1.x = curr_node->x;
            m1.y = curr_node->y;
            edge_msg.points.push_back(m1);

            geometry_msgs::Point m2;
            m2.x = node->x;
            m2.y = node->y;
            edge_msg.points.push_back(m2);

            marker_pub.publish(edge_msg);
        }
    }
    marker_pub.publish(milestone_msg);
}

void publish_shortest_path(path& out_path) {
    for(int i = 1; i < out_path.nodes.size(); i++) {

        id++; // Increase the ID
        visualization_msgs::Marker edge_msg;
        edge_msg.id = id;
        edge_msg.header.frame_id = "map";
        edge_msg.header.stamp = ros::Time();
        edge_msg.ns = "localization";
        edge_msg.action = visualization_msgs::Marker::ADD;
        edge_msg.type = visualization_msgs::Marker::LINE_STRIP;
        edge_msg.scale.x = 0.02;
        edge_msg.scale.y = 0.02;
        edge_msg.scale.z = 0.02;
        edge_msg.color.a = 1.0;
        edge_msg.color.r = 1.0;
        edge_msg.color.g = 0.0;
        edge_msg.color.b = 0.0;

        geometry_msgs::Point m1;
        m1.x = out_path.nodes.at(i-1)->x;
        m1.y = out_path.nodes.at(i-1)->y;
        edge_msg.points.push_back(m1);

        geometry_msgs::Point m2;
        m2.x = out_path.nodes.at(i)->x;
        m2.y = out_path.nodes.at(i-1)->y;
        edge_msg.points.push_back(m2);

        marker_pub.publish(edge_msg);
    }
}

void prm(node* waypoint1, node* waypoint2, const nav_msgs::OccupancyGrid& msg) {


    nodes_arr.push_back(waypoint1);
    nodes_arr.push_back(waypoint2);

    generate_nodes(msg);
    generate_edges();
    publish_graph();

    path out_path;
    bool found = find_shortest_path(waypoint1, waypoint2, out_path, msg);

    if (found) {
        publish_shortest_path();
    }
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
    node* waypoint1 = new node();
    waypoint1->x = 1;
    waypoint1->y = 1;
    waypoint1->yaw = 0;

    node* waypoint2 = new node();
    waypoint2->x = 7;
    waypoint2->y = 7;
    waypoint2->yaw = 0;

    prm(waypoint1, waypoint2, msg);
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
