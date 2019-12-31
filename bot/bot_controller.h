#ifndef BOT_CONTROLLER_H
#define BOT_CONTROLLER_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Plugin.hh>
#include <stdlib.h>
#include <utility>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <climits>
#include <iterator>
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <thread>
#include <mutex>
#include "../utilities/block_q/block_q.h"
#include "../utilities/flag/flag.h"
#include "../utilities/occupancy_grid/occupancy_grid.h"
#include "../utilities/logger/logger.h"

using namespace gazebo;
using namespace std;


namespace gazebo
{
	enum State {SENSING,MOVING,COMMUNICATING,DEACTIVATED};
	

	struct ZBotMSG
	{
		string msg_type="";  // teamup,handshake,grid_update,quit
		string sender_name="";
		int handshaked=false;
		int teamup_time=0;  // time when the team is found
		int teamend_time=0;  // time when the robot leave the team
		double sender_x0=0;
		double sender_y0=0;
		double sender_x=0;
		double sender_y=0;
		int sender_fr=0;
		int sender_fc=0;
		string teammates="";
		vector<double> data;
	};
	

	class DistCellComp{
	public:
		DistCellComp(){}
		~DistCellComp(){}
		bool operator()(vector<int>& lhs,vector<int>& rhs){
			return lhs[2]>rhs[2];
		}
	};
	

	/****************************************************************************
	* Three coordinate systems are used: 
		1. world coordinate: robot's actual local in the world
		2. local coordinate: relative location in robot's local frame
		3. grid coordiante: row and column indexes of the robot's occupancy grid
	* A robot's local coordination is defined in its own local frame  
	  using left-hand rule:
	  	- positive x: right
	  	- positive y: up
	  The robot's initial position is defined as <0,0> in world coordinate;
	  The <0,0> cell in the robot's occupancy map is defined to be the lower 
	  left corner. Below is the correspondce between occupancy grid and world 
	  coordinate: 
				***********<0,+>***********
				*            |            *
				*            |            *
				*            |            *
				*            |            *
				*            |            *
				*            |            *
			  <-,0>--------<0,0>--------<+,0>
				*            |            *
				*            |            *
				*            |            *
				*            |            *
				*            |            *
				*            |            *
			 <r0,c0>*******<0,->***********
	******************************************************************************/
	class ZBot:public ModelPlugin
	{
	private:
		physics::ModelPtr _model;
		event::ConnectionPtr _updateConnection;
		transport::NodePtr _world_node;
		transport::PublisherPtr _world_pub;

		Logger _logger_traj;
		Logger _logger_grid;
		Logger _logger_comm;
		double _log_trjc_len;
		Logger _logger_frnt_clls;
		int _log_collisions;
		bool _had_collision;

		string _name;
		State _state;

		int _time;
		int _teamup_time;
		int _max_idle_time;
		int _max_botcom_wait_time;
		int _max_teamup_wait_time;
		int _walk_time;
		int _idle_time;
		int _teamup_broadcast_gap;
		double _comm_loss_prob;
		mutex _time_lock;
		int _max_grid_broadcast_time;
		int _grid_broadcast_time;

		double _world_x0,_world_y0;
		double _min_local_x,_min_local_y;
		double _max_local_x,_max_local_y;

		double _tvel_scale;
		double _com_range;
		double _colid_dist;
		double _max_off_sub_goal_dist;
		double _turn_unit_angle;
		bool _turning_to_sub_goal;
		double _turn_dir_collavoid;

		pair<int,int> _goal;
		pair<int,int> _sub_goal;
		unordered_map<string,vector<double>> _neighbors;  // key:name, value:world position
		unordered_map<string,vector<double>> _teammates;

		double _sensor0_min_angle,_sensor0_max_angle;
		double _sensor0_ray_cnt;
		double _sensor0_max_range;
		double _sensor0_collision_angle;
		Flag<bool> _sensor0_mapenv;
		Flag<bool> _sensor0_avoid_collision;
		thread* _sensor0_worker;
		BlockQ<vector<double>> _sensor0_data;
		transport::NodePtr _sensor0_node;
		transport::SubscriberPtr _sensor0_sub;
		transport::PublisherPtr _sensor0_pub;

		transport::NodePtr _botcom_node;
		transport::PublisherPtr _botcom_pub;
		transport::SubscriberPtr _botcom_sub;
		Flag<bool> _botcom_handshaked;
		BlockQ<ZBotMSG> _botcom_msgs;
		unordered_map<string,vector<vector<bool>>> _botcom_hist;  // key: bot_name, value: cell has been sent
		unordered_map<string,vector<vector<bool>>> _botcom_hist_temp;
		unordered_map<string,int> _botcom_time;  // the last time when the robot communicate with the other robots
		thread* _botcom_worker;
		
		// entire occupancy grid will be locked, when 
		// _occupancy_grid_worker is merging/updating it  
		OccupancyGrid _occupancy_grid;
		mutex _occupancy_grid_lock;  // lock the entire occupancy grid
		
		int _frnt_assg_cnt;
		double _frnt_obst_penalty;
		double _frnt_bot_influence_radius;
		double _frnt_inaccessible_cost;
		vector<pair<int,int>> _frontiers;

		/*the grid representing the cost of moving from a paricular cell to 
		  destination cell, e.g. from cells to the assigned frontier.*/
		vector<vector<int>> _cost_grid;

		/*increase the _time while locking it. _time is read by the _botcom_worker 
	  	  for determine if teamup session expires*/
		void _time_increase_sync();

		/*Listen to the broadcast of the robot's neighbor*/
		void _listen(ConstGzStringPtr& msg);

		/*store the recieved laser sensor data*/
		void _store_laser_sensor_data(ConstGzStringPtr& sensor_data);
		
		/*handle all the data that is sent from the plugin of sensor0. executed by _sensor0_worker*/
		void _sensor0_data_handler();

		/*Map the sensed environment to the occupancy grid*/
		bool _sensor0_map_environment(vector<double>& sensor_data);

		/*detect collision caused by the obstacles that is  less than _colid_dist away in front of the robot.
		  only use the ray shot towards the front of the robot (a cone of d degrees) for the dection.*/
		void _sensor0_detect_collision(vector<double>& sensor_data);

		/*handling all the messages received from other robots*/
		void _botcom_msg_handler();

		/*serialize the information of this robot's occupancy grid*/
		bool _botcom_serializer(string& msg, string msg_type);
		bool _botcom_serializer_with_log(string& msg, string msg_type);

		/*parse the header of the message recieved from a neighbor robot*/
		bool _botcom_header_parser(string header,ZBotMSG& msg);

		/*parse the data of the message recieved from a neighbor robot
		  current data for*/
		bool _botcom_data_parser(string data, ZBotMSG& msg);

		/*update the robot's _botcom_hist which stores the cells that have been broadcasted to which robots*/
		void _set_botcom_hist();
		void _update_botcom_hist(string bot_name, int r, int c);

		void _reset_botcom_hist(int r, int c);

		/*update the lastest time that the robot communicated with the robot which sent the recieved msg*/
		void _botcom_time_update_sync(ZBotMSG& msg);

		/*merge the recieved robot observation with the robot's occupancy grid
		  assume that the robot knows its initial pose in the world coordinate system*/
		void _merge_occupancy_grid_known_init_pose(ZBotMSG& bot_msg);

		/*check if the cell(r,c) has been sent to ALL of the robot's neighbor*/
		bool _is_replicate_cell(int r, int c);

		/*the robot will not forward any messages until the teamup has expired*/
		int _teamup_expired_sync();

		/*the robot start a teamup process by recording the current time stamp*/
		void _teamup_started_sync();

		/*the robot will not re-communicated the same robot within a certain amount of time*/
		bool _teamup_qualified_sync(ZBotMSG& msg);
		
		/*store the sender of the msg as the neighbor of the robot*/
		void _neighbor_update_sync(ZBotMSG& msg);

		/*clean up all the neighbors of the robot*/
		void _neighbor_clear_sync();

		/*collect all the frontiers in the current local occupancy map*/
		void _load_frontiers();

		/*assign the frontier with the lowest cost to the robot. the function updates
		 _cost_grid after the assignment is done*/
		bool _assigned_closest_frontier();

		/*assign the frontier that is conditioned on the position of this robot's neighbors.
		  the utility of a frontier is penalized by its distance to all the neighrbors*/
		bool _assigned_neighbor_conditioned_frontier();

		/*with epsilon percent of change to assign a random frontier, otherwise assign the 
		  closest frontier to the robot*/
		bool _assigned_egreedy_closest_frontier();

		/*fire tournament among the frontiers. select the winner to the robot*/
		bool _assigned_tournament_closest_frontier();

		/*calculate the distance from each cell to the cell(r0,c0)
		  the function is used for frontier assignment, and online planning for reach the goal*/
		void _cal_cost_grid(vector<vector<double>>& conv_og, int r0, int c0);

		/*convolve the occupancy probability of input occupancy grid using a 2d gaussian filter*/
		vector<vector<double>> _convolve(OccupancyGrid& og);
		vector<vector<double>> _convolve_sync(OccupancyGrid& og);
		
		/*convolution filter based on 2d gaussian function*/
		double _gaussian_filter(OccupancyGrid& map,int r, int c, int filter_size);
		
		/*2d gaussian function*/
		double _gaussian_kernel(double x, double y, double mean, double dev);

		/*check if the robot has reached its goal (the previously assigned frontier)*/
		bool _reached(int r, int c);

		/*the robot is soft_reached if it's within rch_dist of the goal cell (r,c)*/
		bool _soft_reached(int r, int c, double rch_dist);

		/*update the sub_goal where the robot should move next*/
		void _update_sub_goal();

		/*the robot's current position is too far from the current sub_goal.
		  this can be caused by robot's colision avoidance behavior*/
		bool _off_sub_goal();

		/*check if the input cell has been explored or not*/
		bool _verified_sync(int r, int c);

		/*sample the input array based on low variance sampling method. input contains
		  the weights of all the elements to be sampled*/
		unordered_map<int,int> _low_variance_sampler(vector<int>& input);

		/*fire a tournament consisting of the number of rounds equal to the number of 
		  input elements. input contains all the weights of all the involved elements*/
		unordered_map<int,int> _tournament(vector<double>& input);

		/*find the position of the target in the input array by using binary search*/
		int _binary_search(vector<double>& sorted_input, int left, int right, double target);
		
		/*the number of the unexplored cells divided by the total number of cells 
		  in the region centered at input cell. the region is approaximated as a
		  bounding box, instead of a circle.*/
		double _information_gain(int r, int c);
		double _information_gain_sync(int r, int c);

		/*convert the x coordinate in the robot's local frame to the  
		  column index of the associated occupancy grid cells*/
		int _local_x_to_c(double x);

		/*invert procedure of _local_x_to_c*/
		double _c_to_local_x(int c);

		/*convert the y coordinate in the robot's local frame to the
		  row index of the associated occupancy grid cells*/
		int _local_y_to_r(double y);

		/*invert procedure of _local_y_to_r*/
		double _r_to_local_y(int r);
		
		/*round the x coordinate in robot's local frame to the x
		  coordinate of the center of the associated occupancy cell*/
		double _round_local_x(double x);
		
		/*round the y coordinate in robot's local frame to the y
		  coordinate of the center of the associated occupancy cell*/
		double _round_local_y(double y);

		/*cast ray on the occupancy grid based on bresenham's line draw 
		  algirthm. mark cells on the ray to be free in-place*/
		void _bresenham_ray_cast(double xs, double ys, double xe, double ye);

		/*check if the robot goes beyond the maximum area that can be 
		  covered by its occupancy grid*/
		bool _out_of_grid(double x, double y);
		bool _out_of_grid(int r, int c);

		/*the euclidian distance between two cells, based on the robot's local coordinate*/
		double _euclid_cell_dist(int r1, int c1, int r2, int c2);

	public:
		ZBot();
		
		~ZBot();

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

		void OnUpdate(const common::UpdateInfo & /*_info*/);

		/*navigate to the current goal by following the gradien of the _cost_grid*/
		bool MoveToGoal();

		/*Move along the current direction*/
		void Move();

		/*Move after turning*/
		void Move(double degree);

		/*Move toward the goal -- odometry model.*/
		void Move(gazebo::math::Vector3 goal);

		/*Move by following the path specified as vector of x and y coordinate.*/
		void Move(vector<double>& path);

		/*Turn certain degree from the current orientation*/
		void Turn(double dgree);

		/*Turn to a certain location*/
		bool Turnto(gazebo::math::Vector3 goal);

		/*Turn to a certain cell in occupancy grid*/
		bool Turnto(pair<int,int>& cell);

		/*Broadcast a message to the robot's neighbor*/
		void Broadcast(string msg);

	};
}
#endif
