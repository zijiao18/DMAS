#include "bot_controller.h"

GZ_REGISTER_MODEL_PLUGIN(gazebo::ZBot)

ZBot::ZBot():_sensor0_avoid_collision(false),_sensor0_mapenv(false),_botcom_handshaked(false),_occupancy_grid(300,300,1.0){
	_max_idle_time=300;
	_max_botcom_wait_time=10000;
	_max_teamup_wait_time=2000;
	_teamup_time=-_max_teamup_wait_time;
	_max_grid_broadcast_time=10;
	_grid_broadcast_time=0;
	_walk_time=0;
	_teamup_broadcast_gap=50000;//was 10000
	_comm_loss_prob=0.2;
	_time=rand()%_teamup_broadcast_gap;//to simulate the local clocks of robots
	
	_state=MOVING;
	_tvel_scale=2;
	_com_range=20.0;
	_colid_dist=0.5;
	_max_off_sub_goal_dist=5.0;
	_turn_unit_angle=0.001;
	_turn_dir_collavoid=1;
	_turning_to_sub_goal=false;
	_frnt_assg_cnt=0;
	_frnt_obst_penalty=500;
	_frnt_bot_influence_radius=20;
	_frnt_inaccessible_cost=500;

	_cost_grid=vector<vector<int>>(_occupancy_grid.NRow(), vector<int>(_occupancy_grid.NCol(), numeric_limits<int>::max()));
	
	// the occupancy grid must be sufficiently large. The robot's location 
	// will be invalid when it goes beyound the boundaries of the map.
	_min_local_x=-((_occupancy_grid.NCol()-1)/2)*_occupancy_grid.CellSize();  // center of the left most cell
	_min_local_y=-((_occupancy_grid.NRow()-1)/2)*_occupancy_grid.CellSize();  // center of the bottom most cell
	_max_local_x=((_occupancy_grid.NCol()-1)-(_occupancy_grid.NCol()-1)/2)*_occupancy_grid.CellSize();  // center of the right most cell
	_max_local_y=((_occupancy_grid.NRow()-1)-(_occupancy_grid.NRow()-1)/2)*_occupancy_grid.CellSize();  // center of the top most cell

	_log_collisions=0;
	_had_collision=false;

}

ZBot::~ZBot(){
	ZBotMSG botcom_quit;
	botcom_quit.msg_type="quit";
	_botcom_msgs.enq(botcom_quit);
	vector<double> sensor0_quit{-10};
	_sensor0_data.enq(sensor0_quit);
	this->_botcom_worker->join();
	this->_sensor0_worker->join();
	delete this->_botcom_worker;
	delete this->_sensor0_worker;
	
	this->_logger_grid.Log(_occupancy_grid.ToString());
	this->_logger_frnt_clls.Log("last line");
	cout<<_name<<" last line"<<endl;
	this->_logger_grid.Close();
	this->_logger_traj.Close();
	this->_logger_comm.Close();
	this->_logger_frnt_clls.Close();
}

void ZBot::Load(physics::ModelPtr _parent, sdf::ElementPtr ep){
	this->_model = _parent;
	// Listen to the update event. This event is broadcast every simulation iteration.
	this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&ZBot::OnUpdate, this, _1)
	);
	_world_node=transport::NodePtr(new gazebo::transport::Node()); 
	_world_node->Init();
	_world_pub=_world_node->Advertise<msgs::GzString>("~/world/bot");
	this->_name=this->_model->GetName();
	this->_world_x0=this->_model->GetWorldPose().pos.x;
	this->_world_y0=this->_model->GetWorldPose().pos.y;
	this->_goal=pair<int,int>(_local_y_to_r(0),_local_x_to_c(0));
	this->_sub_goal=this->_goal;

	//hard coded based on the configuration in the sensor sdf.
	this->_sensor0_min_angle=-3.141592654;
	this->_sensor0_max_angle=3.141592654;
	this->_sensor0_collision_angle=1.57;
	this->_sensor0_ray_cnt=360;
	this->_sensor0_max_range=10.0;

	_sensor0_node=transport::NodePtr(new gazebo::transport::Node()); _sensor0_node->Init();
	_sensor0_pub=_sensor0_node->Advertise<msgs::Int>("~/"+_model->GetName()+"::sensor0/req");
	_sensor0_sub=_sensor0_node->Subscribe("~/"+_model->GetName()+"::sensor0/data", &ZBot::_store_laser_sensor_data, this);
	_sensor0_worker=new thread(&ZBot::_sensor0_data_handler,this);

	_botcom_node=transport::NodePtr(new gazebo::transport::Node()); _botcom_node->Init();
	_botcom_pub=_botcom_node->Advertise<msgs::GzString>("~/botcom");
	_botcom_sub=_botcom_node->Subscribe("~/botcom",&ZBot::_listen,this);
	_botcom_worker=new thread(&ZBot::_botcom_msg_handler,this);

	this->_logger_traj.Open(_name+"_traj");//current dir is the dir containing the .world file
	this->_logger_grid.Open(_name+"_grid");
	this->_logger_comm.Open(_name+"_comm");
	this->_logger_frnt_clls.Open(_name+"_clls");
}

void ZBot::OnUpdate(const common::UpdateInfo & /*_info*/){	
	//the robot is always listening to its neighbors
	if (_state==DEACTIVATED)
	{
		return;
	}
	//Behaviors of the robot
	if (_state==SENSING)
	{
		if (_sensor0_mapenv==false)
		{
			if (_assigned_closest_frontier())
			{
				cout<<_name<<" assigned frontier "<<get<0>(_goal)<<" "<<get<1>(_goal)<<endl;
				double goal_x=_c_to_local_x(_goal.second)+_world_x0;
				double goal_y=_r_to_local_y(_goal.first)+_world_y0;
				_state=MOVING;
				_neighbor_clear_sync();
				cout<<_name<<" start moving"<<endl;
				_state=MOVING;

				_logger_frnt_clls.Log(
					to_string(_time)+","+
					to_string(goal_x)+","+
					to_string(goal_y)+","+
					to_string(_log_collisions)
				);
			}else{
				cout<<_name<<" is deactivated"<<endl;
				msgs::GzString msg;
				msg.set_data(_name+":deactivate");
				_world_pub->Publish(msg);
				_state=DEACTIVATED;
			}
		}
	}
	if (_state==MOVING){
		if(!(MoveToGoal())){
			_state=SENSING;
			_sensor0_mapenv=true;
			cout<<_name<<" start sensing"<<endl;
		}
		if (_botcom_handshaked==true)
		{
			_state=COMMUNICATING;
			_botcom_handshaked=false;
			_grid_broadcast_time=0;
			cout<<_name<<" start communicating"<<endl;
		}else if (_time%_teamup_broadcast_gap==0)
		{
			string msg="";
			if (_botcom_serializer_with_log(msg,"teamup"))
			{
				_teamup_started_sync();
				Broadcast(msg);
				cout<<_name<<" broadcasted teamup"<<endl;
			}
		}
	}
	if (_state==COMMUNICATING)
	{
		string msg="";
		if (_botcom_serializer_with_log(msg,"grid_update"))
		{
			Broadcast(msg);
			cout<<_name<<" broadcasted grid_update"<<endl;
		}else{
			if(!_assigned_neighbor_conditioned_frontier()){
				msgs::GzString msg;
				msg.set_data(_name+":deactivate");
				_world_pub->Publish(msg);
				_state=DEACTIVATED;
			}else{
				cout<<_name<<" re-assigned frontier ["<<_goal.first<<" "<<_goal.second<<"]"<<endl;
			
				double goal_x=_c_to_local_x(_goal.second)+_world_x0;
				double goal_y=_r_to_local_y(_goal.first)+_world_y0;
				_logger_frnt_clls.Log(to_string(_time)+","+
									to_string(goal_x)+","+
									to_string(goal_y)+","+
									to_string(_log_collisions));

			}
			_state=MOVING;
			_idle_time=0;
			_botcom_handshaked=false;
			_set_botcom_hist();
			_neighbor_clear_sync();
			cout<<_name<<" ends communicating, and start moving"<<endl;
		}
	}
	_logger_traj.Log(to_string(_model->GetWorldPose().pos.x)+","+to_string(_model->GetWorldPose().pos.y));
	_time_increase_sync();
}

void ZBot::Broadcast(string msg){
	if (!msg.empty())
	{
		msgs::GzString msg_out;
		msg_out.set_data(msg);
		_botcom_pub->Publish(msg_out);	
	}
}

bool ZBot::MoveToGoal(){
	if (_reached(_goal.first, _goal.second))
	{
		return false;
	}
	if(_sensor0_avoid_collision==true){
		if(!_had_collision){
			_log_collisions++;
			_had_collision=true;
			if(rand()%10<5){
				_turn_dir_collavoid *= -1.0;
			}
		}
		Turn(_turn_dir_collavoid*_turn_unit_angle);
		_turning_to_sub_goal=false; 
		return true;
	}
	_had_collision=false;
	if (_reached(_sub_goal.first,_sub_goal.second))
	{
		_update_sub_goal();
		_turning_to_sub_goal=true;
	}else if(_off_sub_goal()){
		_update_sub_goal();
		return false;
	}else{
		if (_turning_to_sub_goal)
		{
			if (Turnto(_sub_goal))
			{
				_turning_to_sub_goal=false;
			}
		}else{
			Move();
		}
	}
	return true;
}

bool ZBot::_verified_sync(int r, int c){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	if (_occupancy_grid.GetOccupancy(r,c)!=0.5)
	{
		return false;
	}
	return true;
}

void ZBot::_update_sub_goal(){
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int r=_local_y_to_r(world_y-_world_y0);
	int c=_local_x_to_c(world_x-_world_x0);
	double min_h=numeric_limits<double>::max();
	int sub_r=_sub_goal.first, sub_c=_sub_goal.second;
	for (int i = -1; i < 2; ++i)
	{
		for (int j = -1; j < 2; ++j)
		{
			double h=numeric_limits<double>::max();
			int cr=r+j;
			int cc=c+i;
			if (cr>=0 && cr<_occupancy_grid.NRow() && 
				cc>=0 && cc<_occupancy_grid.NCol())
			{
				h=_cost_grid[cr][cc];
			}
			if (h<min_h)
			{
				min_h=h;
				sub_r=cr;
				sub_c=cc;
			}
		}
	}
	_sub_goal.first=sub_r;
	_sub_goal.second=sub_c;
	cout<<_name<<" at ["<<
	r<<" "<<c<<"] moves to sub_goal ["<<
	_sub_goal.first<<" "<<_sub_goal.second<<"]:("<<
	_cost_grid[_sub_goal.first][_sub_goal.second]<<")->["<<
	_goal.first<<" "<<_goal.second<<"]"<<endl;
}

bool ZBot::_off_sub_goal(){
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int r=_local_y_to_r(world_y-_world_y0);
	int c=_local_x_to_c(world_x-_world_x0);
	double dist=_euclid_cell_dist(r, c, _sub_goal.first, _sub_goal.second);
	if (dist>=_max_off_sub_goal_dist)
	{
		cout<<_name<<" off sub_goal "<<dist<<endl;
		return true;
	}
	return false;
}

void ZBot::_cal_cost_grid(vector<vector<double>>& conv_og, int r0, int c0){
	//reset _cost_grid
	for (int i = 0; i < _cost_grid.size(); ++i)
	{
		for (int j = 0; j < _cost_grid[i].size(); ++j)
		{
			_cost_grid[i][j]=numeric_limits<int>::max();
		}
	}
	priority_queue<vector<int>,vector<vector<int>>,DistCellComp> pq;
	vector<int> s{r0,c0,0};
	pq.push(s);
	/************************************************************************
	  adding large penalty to the occupied cell, so that robot will not 
	  select paths penentrating obstacles. the penalty is scaled based on 
	  cell occupancy probability
	 *************************************************************************/
	while(!pq.empty()){
		vector<int> e=pq.top();
		pq.pop();
		_cost_grid[e[0]][e[1]]=e[2];

		for (int i = -1; i < 2; ++i)
		{
			for (int j = -1; j < 2; ++j)
			{
				int cr=e[0]+j, cc=e[1]+i;
				if (cr>=0 && cr<_occupancy_grid.NRow() && 
					cc>=0 && cc<_occupancy_grid.NCol() &&
					!(i==0 && j==0))
				{
					vector<int> neig{cr,cc,e[2]+int(1+_frnt_obst_penalty*conv_og[cr][cc])};
					if (_cost_grid[cr][cc]==numeric_limits<int>::max())
					{
						pq.push(neig);
					}
					_cost_grid[cr][cc]=min(_cost_grid[cr][cc],neig[2]);
				}
			}
		}
	}
	/*for (int i = 0; i < _cost_grid.size(); ++i)
	{
		for (int j=0;j<_cost_grid[i].size();++j){
			cout<<_cost_grid[i][j]<<" ";
		}
		cout<<endl;
	}
	cout<<endl;*/
}

double ZBot::_euclid_cell_dist(int r1, int c1, int r2, int c2){
	if (r1<0 || r1>=_occupancy_grid.NRow() || 
		c1<0 || c1>=_occupancy_grid.NCol() ||
		r2<0 || r2>=_occupancy_grid.NRow() || 
		c2<0 || c2>=_occupancy_grid.NCol())
	{
		return numeric_limits<double>::max();
	}
	return sqrt(pow(r1-r2,2.0)+pow(c1-c2,2.0))*_occupancy_grid.CellSize();
}

void ZBot::Move(){
	gazebo::math::Pose pose = _model->GetWorldPose();
	double yaw=pose.rot.GetYaw();
	gazebo::math::Vector3 vel_dir(cos(yaw),sin(yaw),0);
	this->_model->GetLink("chassis")->SetLinearVel(_tvel_scale*vel_dir);
}

void ZBot::Move(double degree){
	gazebo::math::Pose pose = _model->GetWorldPose();
	double yaw = pose.rot.GetYaw()+degree;
    	_model->SetWorldPose(
    		gazebo::math::Pose(
    			pose.pos,
    			gazebo::math::Quaternion(
    				0, 
    				0, 
    				yaw
    			)
    		)
    	);
	gazebo::math::Vector3 vel_dir(cos(yaw),sin(yaw),0);
	this->_model->GetLink("chassis")->SetLinearVel(_tvel_scale*vel_dir);
}

void ZBot::Move(gazebo::math::Vector3 goal){
	/***************************************************************** 	
		By default the robot face to the positive direction of x-axis, 
		yaw is rotated counter-clock wise, and it's in range [-pi,pi]
	******************************************************************/
	gazebo::math::Pose pose = _model->GetWorldPose();
	gazebo::math::Vector3 goal_dir=goal-pose.pos;
	//angle between positive x-axis and vec_robot_to_goal
	double goal_angle=atan2(
		goal_dir.y,
		goal_dir.x
	);
	gazebo::math::Pose pose_to_goal(
		pose.pos,
		gazebo::math::Quaternion(
			0, 
			0, 
			goal_angle
		)
	);
	_model->SetWorldPose(pose_to_goal);
	double yaw = pose.rot.GetYaw();
	gazebo::math::Vector3 vel_dir = gazebo::math::Vector3(
		cos(yaw),
		sin(yaw),
		0
	);
	this->_model->GetLink("chassis")->SetLinearVel(_tvel_scale*vel_dir);
}

void ZBot::Turn(double degree){
	gazebo::math::Pose pose = _model->GetWorldPose();
	double new_yaw=pose.rot.GetYaw()+degree;
    _model->SetWorldPose(
    	gazebo::math::Pose(
    		pose.pos,
  			gazebo::math::Quaternion(0, 0, new_yaw)
  		)
  	);
}

bool ZBot::Turnto(gazebo::math::Vector3 goal){
	gazebo::math::Pose pose = _model->GetWorldPose();
	gazebo::math::Vector3 vec_robot_to_goal=goal-pose.pos;
	double yaw=pose.rot.GetYaw();
	//angle between positive x-axis and vec_robot_to_goal
	double goal_angle=atan2(vec_robot_to_goal.y,vec_robot_to_goal.x);
	if (abs(goal_angle-yaw)>=_turn_unit_angle)
	{
		double degree=(goal_angle-yaw)/abs(goal_angle-yaw)*_turn_unit_angle;
		Turn(degree);
		return false;
	}
	return true;
}

bool ZBot::Turnto(pair<int,int>& cell){
	gazebo::math::Pose pose = _model->GetWorldPose();
	double cell_world_x = _c_to_local_x(cell.second)+_world_x0;
	double cell_world_y = _r_to_local_y(cell.first)+_world_y0;
	double yaw = pose.rot.GetYaw();
	gazebo::math::Vector3 goal(cell_world_x,cell_world_y,0);
	gazebo::math::Vector3 curpos(pose.pos.x, pose.pos.y, pose.pos.z);
	gazebo::math::Vector3 goal_dir=goal-curpos;
	//angle between positive x-axis and vec_robot_to_goal
	double goal_angle=atan2(goal_dir.y, goal_dir.x);
	double diff_angle=goal_angle-yaw;
	if (abs(diff_angle)>=_turn_unit_angle)
	{
		double degree=diff_angle/abs(diff_angle)*_turn_unit_angle;
		Turn(degree);
		return false;
	}
	return true;
}

void ZBot::_listen(ConstGzStringPtr& msg){
	string content=msg->data();
	int header_offset=content.find(' ');
	ZBotMSG rcv_msg;
	string header = content.substr(0,header_offset);
	if (_botcom_header_parser(header,rcv_msg))
	{
		string data = content.substr(header_offset+1);
		_botcom_data_parser(data,rcv_msg);
		_botcom_msgs.enq(rcv_msg);
	}
}

void ZBot::_store_laser_sensor_data(ConstGzStringPtr& sensor_data){
	vector<double> ranges;
	stringstream data(sensor_data->data());
	string range_str;
	while(getline(data,range_str,' ')){
		ranges.push_back(stod(range_str));
	}
	_sensor0_data.enq(ranges);

}

void ZBot::_sensor0_data_handler(){
	while(true){
		vector<double> sensor_data=_sensor0_data.deq();
		if (sensor_data.size()==1)
		{
			cout<<_name<<" _sensor0_worker quit"<<endl;
			return;
		}
		if(_sensor0_mapenv==true){
			_sensor0_map_environment(sensor_data);
			_sensor0_mapenv=false;
		}else{
			_sensor0_detect_collision(sensor_data);
		}
	}
}

void ZBot::_sensor0_detect_collision(vector<double>& sensor_data){
	gazebo::math::Pose pose = _model->GetWorldPose();
	double bot_yaw = pose.rot.GetYaw();
	double bot_x = pose.pos.x-_world_x0;
	double bot_y = pose.pos.y-_world_y0;
	//ray angle increase in counter-clock wise direction
	double ray_angle_unit = (
		(_sensor0_max_angle-_sensor0_min_angle)
		/ double(_sensor0_ray_cnt-1)
	);
	double ray_angle = bot_yaw-_sensor0_max_angle;
	for(int i=0; i<sensor_data.size(); i++, ray_angle+=ray_angle_unit)
	{
		//index of ray increases in counter-clock wise
		if(sensor_data[i] != -1 && 
			abs(ray_angle-bot_yaw) < _sensor0_collision_angle && 
			sensor_data[i] < _colid_dist){
			_sensor0_avoid_collision=true;
			return;
		}
	}
	_sensor0_avoid_collision=false;
}

bool ZBot::_sensor0_map_environment(vector<double>& sensor_data)
{
	lock_guard<mutex> lock(_occupancy_grid_lock);	
	gazebo::math::Pose pose = _model->GetWorldPose();
	double bot_yaw=pose.rot.GetYaw();
	double bot_local_x=pose.pos.x-_world_x0;
	double bot_local_y=pose.pos.y-_world_y0;
	if (_out_of_grid(bot_local_x,bot_local_y))
	{
		return true;
	}
	//ray angle increase in counter-clock wise direction
	double ray_angle_unit=(
		(_sensor0_max_angle-_sensor0_min_angle)
		/ double(_sensor0_ray_cnt-1)
	);
	double ray_angle=bot_yaw-_sensor0_max_angle;
	for(int i=0; i<sensor_data.size(); i++, ray_angle+=ray_angle_unit){
		//index of ray increases in counter-clock wise
		double end_beam_x = _sensor0_max_range*cos(ray_angle)+bot_local_x;
		double end_beam_y = _sensor0_max_range*sin(ray_angle)+bot_local_y;
		if(sensor_data[i]!=-1)
		{
			end_beam_x=sensor_data[i]*cos(ray_angle)+bot_local_x;
			end_beam_y=sensor_data[i]*sin(ray_angle)+bot_local_y;
			_occupancy_grid.SetOccupancy(
				_local_y_to_r(end_beam_y),
				_local_x_to_c(end_beam_x),
				1.0
			);
			_occupancy_grid.SetGrid(
				_local_y_to_r(end_beam_y),
				_local_x_to_c(end_beam_x),
				true
			);
		}
		_bresenham_ray_cast(bot_local_x,bot_local_y,end_beam_x,end_beam_y);
	}
	cout<<_name<<endl;
	_occupancy_grid.ShowGrid(
		_local_y_to_r(bot_local_y),
		_local_x_to_c(bot_local_x)
	);
	return true;
}

void ZBot::_bresenham_ray_cast(double xs, double ys, double xe, double ye){
	int rs=_local_y_to_r(ys);
	int cs=_local_x_to_c(xs);
	int re=_local_y_to_r(ye);
	int ce=_local_x_to_c(xe);
	int dr=re-rs;
	int dc=ce-cs;
	int c=cs;
	int r=rs;
	/***************************************************
	 decision variable of k_th cell p(k)=dx(d_low-d_up)
	 p(k+1)=p(k)+2|dy|(x(k+1)-x(k))-2|dx|(y(k+1)-y(k))
	****************************************************/
	if (abs(dr)<=abs(dc))// -1<=slope<=1
	{
		int p=2*abs(dr)-abs(dc);  // decision value for next plot
		while (c!=ce){
			if (_out_of_grid(r,c))  // stop early when the ray goes out of the occupancy grid
			{
				break;
			}
			if (_occupancy_grid.GetOccupancy(r,c)==0.5)
			{
				_occupancy_grid.SetOccupancy(r,c,0);
				_occupancy_grid.SetGrid(r,c,false);
			}
			if (p>0){
				if (dr>0)
				{
					r++;
				}else{
					r--;
				}
				p-=(2*abs(dc));
			}
			p+=(2*abs(dr));
			if (dc>0){
				c++;
			}else{
				c--;
			}
		}
	}else if(abs(dr)>abs(dc)) // slope>1 or slope<-1
	{
		int p=2*abs(dc)-abs(dr);  // decision value for next plot
		while (r!=re){
			if (_out_of_grid(r,c))  // stop early when the ray goes out of the occupancy grid
			{
				break;
			}
			if (_occupancy_grid.GetOccupancy(r,c)==0.5)
			{
				_occupancy_grid.SetOccupancy(r,c,0);
				_occupancy_grid.SetGrid(r,c,false);
			}
			if(p>0){
				if (dc>0)
				{
					c++;
				}else{
					c--;
				}
				p-=(2*abs(dr));
			}
			p+=(2*abs(dc));
			if(dr>0){
				r++;
			}else{
				r--;
			}
		}
	}
}

bool ZBot::_botcom_serializer(string& msg, string msg_type){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	msg += (msg_type + "," + _name + ","
		+ to_string(_model->GetWorldPose().pos.x) + ","
		+ to_string(_model->GetWorldPose().pos.y) + ","
		+ to_string(_world_x0) + ","
		+ to_string(_world_y0) + "," + " ");
	if (msg_type.compare("grid_update")==0)
	{
		int data_size=0;
		for (int r = 0; r < _occupancy_grid.NRow(); ++r)
		{
			for (int c =0; c<_occupancy_grid.NCol(); ++c){
				if (_occupancy_grid.GetOccupancy(r,c)!=0.5 && !_is_replicate_cell(r,c))
				{
					msg+=(to_string(r)+","+
						to_string(c)+","+
						to_string(_occupancy_grid.GetOccupancy(r,c))+",");
					++data_size;
				}
			}
		}
		if (data_size==0)
		{
			return false;
		}
	}
	return true;
}

bool ZBot::_botcom_serializer_with_log(string& msg, string msg_type){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	msg+=(msg_type+","+_name+","+
		  to_string(_model->GetWorldPose().pos.x)+","+
		  to_string(_model->GetWorldPose().pos.y)+","+
		  to_string(_world_x0)+","+to_string(_world_y0)+",");
	msg+=" ";
	if (msg_type.compare("grid_update")==0)
	{
		int data_size=0;
		int known_size=0;
		for (int r = 0; r < _occupancy_grid.NRow(); ++r)
		{
			for (int c =0; c<_occupancy_grid.NCol(); ++c){
				if (_occupancy_grid.GetOccupancy(r,c)!=0.5)
				{
					if (!_is_replicate_cell(r,c))
					{
						msg+=(to_string(r) + "," + to_string(c) + "," + to_string(_occupancy_grid.GetOccupancy(r,c))+",");
						++data_size;
					}
					++known_size;
				}
			}
		}
		_logger_comm.Log(to_string(data_size)+","+to_string(known_size));
		this->_grid_broadcast_time++;
		if (this->_grid_broadcast_time > this->_max_grid_broadcast_time)
		{
			return false;
		}
		cout << _name << " has broadcast env map, known cells : " \
		<< known_size << " broadcast cells : " << data_size<<endl;
	}
	return true;
}

bool ZBot::_botcom_header_parser(string header, ZBotMSG& msg){
	//header format: name,x,y,teamup_time,x0,y0
	stringstream hss(header);
	string token="";
	getline(hss,token,',');
	string type=token;
	getline(hss,token,',');
	string name=token;
	getline(hss,token,',');
	double x=stod(token);
	getline(hss,token,',');
	double y=stod(token);
	double dist=sqrt(
		pow(x-_model->GetWorldPose().pos.x, 2.0)
		+ pow(y-_model->GetWorldPose().pos.y, 2.0)
	);
	if (name.compare(_name)!=0 && dist<_com_range)
	{
		msg.msg_type=type;
		msg.sender_name=name;
		msg.sender_x=x;
		msg.sender_y=y;
		getline(hss,token,',');
		msg.sender_x0=stod(token);
		getline(hss,token,',');
		msg.sender_y0=stod(token);
		return true;
	}
	return false;
}

bool ZBot::_botcom_data_parser(string data,ZBotMSG& msg){
	//data: cell1_r,cell1_c,cell1_occu,cell1_prob,cell2_r,cell2_c,cell2_occu,cell2_prob...
	if (data.empty())
	{
		return false;
	}
	stringstream dss(data);
	string token="";
	int cnt=0;
	while(getline(dss,token,',')){
		msg.data.push_back(stod(token));
		cnt++;
	}
	if (cnt>0 && cnt%3==0)
	{
		return true;
	}
	msg.data.clear();
	return false;
}

void ZBot::_botcom_msg_handler(){
	/* While communicating with other robots, the robot will wait to maximmum t time unit. 
	The robot will stop communicating with other robot, regardless if its occupancy map is
	converged or not. In addition, the robot will also stop teaming up with new robot when 
	the robot's waiting time approaches its end. */
	while(true){
		ZBotMSG bot_msg=_botcom_msgs.deq();

		if (bot_msg.msg_type.compare("quit")==0){
			cout<<_name<<" _botcom_worker quit"<<endl;
			return;
		}

		if (bot_msg.msg_type.compare("teamup")==0 /*&& _teamup_qualified_sync(bot_msg)*/)//the robot should not re-communicate with the other robots who it recently communicated
		{
			/*Once the robot recieves a teamup message, it will start its teaming up process.
			  during the teaming up process, the robot should wait until all its neighbors'
			  teamup message arrives. The robot should not broadcast message, until the end 
			  of its teamup process*/
			string msg="";
			if (_teamup_expired_sync()>0 && _botcom_serializer_with_log(msg,"teamup"))
			{
				_teamup_started_sync(); //without this, broadcasting loop can be trigered
										// when the robot is forward teamup messages
				Broadcast(msg);
				cout<<_name<<" _botcom_worker replied "<<bot_msg.sender_name<<endl;
			}
			_botcom_handshaked=true;
			_neighbor_update_sync(bot_msg);
			cout<<_name<<" _botcom_worker handshaked with "<<bot_msg.sender_name<<endl;
		}

		if (bot_msg.msg_type.compare("grid_update")==0 && 
			double(rand())/double(numeric_limits<int>::max()) < (1-this->_comm_loss_prob))
		{
			_merge_occupancy_grid_known_init_pose(bot_msg);
			cout<<_name<<" _botcom_worker merged occupancy grid from "<<bot_msg.sender_name<<endl;
		}
	}
}

void ZBot::_neighbor_update_sync(ZBotMSG& msg){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	_neighbors[msg.sender_name]=vector<double>{msg.sender_x,msg.sender_y};
}

void ZBot::_neighbor_clear_sync(){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	_neighbors.clear();
}

void ZBot::_merge_occupancy_grid_known_init_pose(ZBotMSG &bot_msg){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	double ext_world_x0=bot_msg.sender_x0, ext_world_y0=bot_msg.sender_y0;
	double cell_size=_occupancy_grid.CellSize();
	for (int i = 0; i < bot_msg.data.size(); i+=3)
	{
		double ext_world_y=_min_local_y+bot_msg.data[i]*cell_size+ext_world_y0;  // convert r to y in world coord
		double ext_world_x=_min_local_x+bot_msg.data[i+1]*cell_size+ext_world_x0;  // convert c to x in world coord
		double ext_prob=bot_msg.data[i+2];
		int r=_local_y_to_r(ext_world_y-_world_y0);  // convert ext_y to this robot's local frame and then to r
		int c=_local_x_to_c(ext_world_x-_world_x0);  // convert ext_x to this robot's local frame and then to c

		if (r<0 || c<0)
		{
			continue;
		}
		double prob=_occupancy_grid.GetOccupancy(r,c);
		if (prob!=0.5)
		{
			// set the cell to be free when conflict happens
			// the conflict can happen when other robots are detected as obstacles
			//double new_prob=min(_occupancy_grid.GetOccupancy(r,c),ext_prob);
			double new_prob=max(_occupancy_grid.GetOccupancy(r,c),ext_prob);
			_occupancy_grid.SetOccupancy(r,c,new_prob);
			_occupancy_grid.SetGrid(r,c,new_prob==1);
		}else{
			_occupancy_grid.SetOccupancy(r,c,ext_prob);
			_occupancy_grid.SetGrid(r,c,ext_prob==1);
		}
		_update_botcom_hist(bot_msg.sender_name, r, c);
	}
}

void ZBot::_load_frontiers(){
	//frontier is the UNKNOWN cell on the boundery of explored and unexplored areas.
	int nr=_occupancy_grid.NRow();
	int nc=_occupancy_grid.NCol();
	_frontiers.clear();
	for(int r=0; r<nr; r++){
		for(int c=0; c<nc; c++){
			if (_occupancy_grid.GetOccupancy(r,c)==0.0){
				int is_frontier=false;
				for(int i=-1; i<2; i++){
					for (int j=-1; j<2; j++){
						double occ = _occupancy_grid.GetOccupancy(r+i,c+j);
						if(occ==0.5 && !_out_of_grid(r+i,c+j)){
							is_frontier=true;	
							_frontiers.push_back(pair<int,int>(r,c));	
							break;			
						}
					}
					if(is_frontier){
						break;
					}
				}
			}
		}
	}
}

bool ZBot::_assigned_closest_frontier(){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int r=_local_y_to_r(world_y-_world_y0);
	int c=_local_x_to_c(world_x-_world_x0);
	_load_frontiers();
	vector<vector<double>> conv_og=_convolve(_occupancy_grid);
	if (!_frontiers.empty())
	{
		_cal_cost_grid(conv_og, r,c);
		int best_fr=0,best_fc=0;
		double min_cost=numeric_limits<double>::max();
		for(auto f : _frontiers){
			int fr=get<0>(f);
			int fc=get<1>(f);
			double cost=_cost_grid[fr][fc];
			if (cost<_frnt_inaccessible_cost && cost<min_cost && cost>0)
			{
				min_cost=cost;
				best_fr=fr;
				best_fc=fc;
			}
		}
		if (min_cost<numeric_limits<double>::max())
		{
			_goal.first=best_fr;
			_goal.second=best_fc;
			_cal_cost_grid(conv_og,_goal.first,_goal.second);
			return true;
		}
		
	}
	return false;
}

bool ZBot::_assigned_neighbor_conditioned_frontier(){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int r=_local_y_to_r(world_y-_world_y0);
	int c=_local_x_to_c(world_x-_world_x0);
	_load_frontiers();
	vector<vector<double>> conv_og=_convolve(_occupancy_grid);
	if (!_frontiers.empty())
	{
		/*cout<<_name<<" neighbors: ";
		for (auto n : _neighbors)
		{
			cout<<n.first<<"["<<n.second[0]<<" "<<n.second[1]<<"]";
		}
		cout<<endl;*/
		_cal_cost_grid(conv_og,r,c);
		int best_fr=0,best_fc=0;
		double min_cost=numeric_limits<double>::max();
		for(auto f : _frontiers){
			int fr=get<0>(f);
			int fc=get<1>(f);
			double cost=_cost_grid[fr][fc];
			if (cost<_frnt_inaccessible_cost)
			{
				double fy_world=_r_to_local_y(fr)+_world_y0;
				double fx_world=_c_to_local_x(fc)+_world_x0;
				for (auto n : _neighbors)
				{
					double dist = sqrt(pow(n.second[0]-fx_world,2.0)+pow(n.second[1]-fy_world,2.0));
					double penalty=max(0.0,(_frnt_bot_influence_radius-dist));
					cost+=penalty;
				}
				if (cost<min_cost && cost>0)
				{
					min_cost=cost;
					best_fr=fr;
					best_fc=fc;
				}
			}
			
		}
		if (min_cost<numeric_limits<double>::max())
		{
			_goal.first=best_fr;
			_goal.second=best_fc;
			_cal_cost_grid(conv_og,_goal.first,_goal.second);
			return true;
		}
	}
	return false;
}

bool ZBot::_assigned_egreedy_closest_frontier(){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int r=_local_y_to_r(world_y-_world_y0);
	int c=_local_x_to_c(world_x-_world_x0);
	_load_frontiers();
	vector<vector<double>> conv_og=_convolve(_occupancy_grid);
	if (!_frontiers.empty())
	{
		double p = double(rand())/double(RAND_MAX);
		double epsilon = max(0.1,pow(0.9,_frnt_assg_cnt));
		if (p<epsilon)
		{
			int fi=rand()%_frontiers.size();
			_goal=_frontiers[fi];
			cout<<_name<<" assigned randome frontier, epsilon: "<<epsilon<<endl;
		}else{
			_cal_cost_grid(conv_og, r,c);
			int best_fr=0,best_fc=0;
			double min_dist=numeric_limits<double>::max();
			for(auto f : _frontiers){
				int fr=get<0>(f);
				int fc=get<1>(f);
				double dist=_cost_grid[fr][fc];
				if (dist<min_dist && dist>0)
				{
					min_dist=dist;
					best_fr=fr;
					best_fc=fc;
				}
			}
			_goal.first=best_fr;
			_goal.second=best_fc;
		}
		_cal_cost_grid(conv_og,_goal.first,_goal.second);
		_frnt_assg_cnt++;
		return true;
	}
	return false;
}

bool ZBot::_assigned_tournament_closest_frontier(){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int r=_local_y_to_r(world_y-_world_y0);
	int c=_local_x_to_c(world_x-_world_x0);
	_load_frontiers();
	vector<vector<double>> conv_og=_convolve(_occupancy_grid);
	if (!_frontiers.empty())
	{
		_cal_cost_grid(conv_og,r,c);
		vector<double> f_cost;
		for(auto f : _frontiers){
			int fr=get<0>(f);
			int fc=get<1>(f);
			double cost=_cost_grid[fr][fc];
			f_cost.push_back(1.0/cost);
		}
		unordered_map<int,int> f_sample=_tournament(f_cost);
		int max_freq=0,best_fi=0;
		for (auto s:f_sample)
		{
			if (s.second>max_freq)
			{
				max_freq=s.second;
				best_fi=s.first;
			}
		}
		_goal=_frontiers[best_fi];
		_cal_cost_grid(conv_og, _goal.first,_goal.second);
		return true;
	}
	return false;
}

int ZBot::_binary_search(vector<double>& sorted_input, int left, int right, double target){
	if(left==right){
		return -1;
	}
	int mid=(left+right)/2;
	if(sorted_input[mid+1]<=target){
		return _binary_search(sorted_input,mid+1,right,target);
	}else if (sorted_input[mid]>target)
	{
		return _binary_search(sorted_input,left,mid,target);
	}else{
		return mid;
	}
}

unordered_map<int,int> ZBot::_tournament(vector<double>& input){
	unordered_map<int,int> samples;
	int n=input.size();
	double sum=0;
	vector<double> wheel{sum};
	for (int i = 0; i < n; ++i)
	{
		sum+=input[i];
		wheel.push_back(sum);
		samples[i]=0;
	}
	for (int round = 0; round < n; ++round)  // fire the trounament for n round
	{
		double r=double(sum)*double(rand()%RAND_MAX)/double(RAND_MAX);
		int i=0;
		while(!(r>=wheel[i]&&r<wheel[i+1]) && i<=n){
			i++;
		}
		if (samples.find(i)!=samples.end())
		{
			samples[i]++;
		}
	}
	return samples;
}

unordered_map<int,int> ZBot::_low_variance_sampler(vector<int>& input){
	unordered_map<int,int> samples;
	int n=input.size();
	int sum=0;
	vector<int> wheel{sum};
	for (int i = 0; i < n; ++i)
	{
		sum+=input[i];
		wheel.push_back(sum);
		samples[i]=0;
	}
	double step=double(sum)/double(n);
	double r=double(sum)*double(rand()%RAND_MAX)/double(RAND_MAX);
	int r_i=0;
	while(!(r>=wheel[r_i] && r<wheel[r_i+1])){
		r_i++;
	}
	samples[r_i]=1;
	int cnt=0;
	int ws=wheel.size();
	while(cnt<n){
		r=(r+step<sum)?(r+step):(r+step-sum);
		if (r>=wheel[(r_i+1)%ws])
		{
			r_i=(r_i+1)%ws;
		}
		samples[r_i]++;
		cnt++;
	}
	return samples;
}

bool ZBot::_is_replicate_cell(int r, int c){
	for(auto n : _neighbors){
		if (_botcom_hist.find(n.first)==_botcom_hist.end() || 
			!_botcom_hist[n.first][r][c])
		{
			return false;
		}
	}
	return true;
}

void ZBot::_set_botcom_hist(){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	this->_botcom_hist.clear();
	for(auto iter=_botcom_hist_temp.begin(); iter!=_botcom_hist_temp.end(); iter++){
		_botcom_hist[iter->first] = iter->second;
	}
}

void ZBot::_update_botcom_hist(string bot_name, int r, int c){
	if (_botcom_hist_temp.find(bot_name)==_botcom_hist_temp.end())
	{
		_botcom_hist_temp[bot_name]=vector<vector<bool>>(
						_occupancy_grid.NRow(),
						vector<bool>(_occupancy_grid.NCol(),false));
	}
	_botcom_hist_temp[bot_name][r][c]=true;
}

void ZBot::_reset_botcom_hist(int r, int c){
	for(auto h : _botcom_hist){
		h.second[r][c]=false;
	}
}

bool ZBot::_reached(int r, int c){
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int bot_r=_local_y_to_r(world_y-_world_y0);
	int bot_c=_local_x_to_c(world_x-_world_x0);
	if (bot_r==r && bot_c==c)
	{
		cout<<_name<<" reached ["<<r<<" "<<c<<"]"<<endl;
		return true;
	}
	return false;
}

bool ZBot::_soft_reached(int r, int c, double rch_dist){
	double world_x=_model->GetWorldPose().pos.x;
	double world_y=_model->GetWorldPose().pos.y;
	int bot_r=_local_y_to_r(world_y-_world_y0);
	int bot_c=_local_x_to_c(world_x-_world_x0);
	return (_cost_grid[bot_r][bot_c]<=rch_dist);
}

void ZBot::_teamup_started_sync(){
	lock_guard<mutex> lock(_time_lock);
	_teamup_time=_time;
}

int ZBot::_teamup_expired_sync(){
	lock_guard<mutex> lock(_time_lock);
	if ((_time-_teamup_time)>_max_teamup_wait_time)
	{
		return 1;
	}else if ((_time-_teamup_time)==_max_teamup_wait_time)
	{
		return 0;
	}else{
		return -1;
	}
}

void ZBot::_time_increase_sync(){
	lock_guard<mutex> lock(_time_lock);
	_time++;
}

vector<vector<double>> ZBot::_convolve(OccupancyGrid& og){
	vector<vector<double>> conv_og(og.NRow(),vector<double>(og.NCol(),0));
	for (int r=0;r<og.NRow();r++){
		for (int c=0;c<og.NCol();c++){
			conv_og[r][c]=_gaussian_filter(og,r,c,2);  // 3x3 gaussian filter
		}
	}
	return conv_og;
}

vector<vector<double>> ZBot::_convolve_sync(OccupancyGrid& og){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	return _convolve(og);
}

double ZBot::_gaussian_filter(OccupancyGrid& og, int r, int c, int filter_size){
	int offset=filter_size/2;
	double conv_res=0;
	double sum=0;
	for(int i=-offset; i<=offset; i++){
		for (int j=-offset; j<=offset; j++)
		{
			if ((r+i)>=0 && (r+i)<og.NRow() && 
				(c+j)>=0 && (c+j)<og.NCol() && 
				(i!=0 || j!=0)){
				double g=_gaussian_kernel(i,j,0.0,1.0);
				sum+=g;
				conv_res+=g*(og.GetOccupancy(r+i,c+j));	
			}
		}
	}
	return conv_res/sum;
}

double ZBot::_gaussian_kernel(double x, double y, double mean, double dev){
	double n=-(pow(x-mean,2.0)+pow(y-mean,2.0))/(2.0*pow(dev,2.0));
	double out = exp(n)/sqrt(dev*2.0*M_PI);
	return out;
}

int ZBot::_local_x_to_c(double x){
	//column 0 is defined to be the left most column
	double dist_to_left_boundary=x-(_min_local_x-_occupancy_grid.CellSize()/2.0);
	return int(dist_to_left_boundary/_occupancy_grid.CellSize());
}

double ZBot::_c_to_local_x(int c){
	return _min_local_x+c*_occupancy_grid.CellSize();
}

int ZBot::_local_y_to_r(double y){
	//row 0 is defined to be the bottom most row
	double dist_to_bottom_boundary=y-(_min_local_y-_occupancy_grid.CellSize()/2.0);
	return int(dist_to_bottom_boundary/_occupancy_grid.CellSize());
}

double ZBot::_r_to_local_y(int r){
	return _min_local_y+r*_occupancy_grid.CellSize();
}

double ZBot::_round_local_x(double x){
	double cell_size=_occupancy_grid.CellSize();
	return int((x-_min_local_x-cell_size/2.0)/cell_size)*cell_size;
}

double ZBot::_round_local_y(double y){
	double cell_size=_occupancy_grid.CellSize();
	return int((y-_min_local_y-cell_size/2.0)/cell_size)*cell_size;
}


bool ZBot::_out_of_grid(double x, double y){
	if (x<=_min_local_x-_occupancy_grid.CellSize()/2.0 || 
		x>=_max_local_x+_occupancy_grid.CellSize()/2.0 ||
		y<=_min_local_y-_occupancy_grid.CellSize()/2.0 || 
		y>=_max_local_y+_occupancy_grid.CellSize()/2.0)
	{
		return true;
	}
	return false;
}

bool ZBot::_out_of_grid(int r, int c){
	if (r<0 || r>=_occupancy_grid.NRow() || 
		c<0 || c>=_occupancy_grid.NCol())
	{
		return true;
	}
	return false;
}

bool ZBot::_teamup_qualified_sync(ZBotMSG& msg){
	lock_guard<mutex> lock(_time_lock);
	return (_time-_botcom_time[msg.sender_name])>_max_botcom_wait_time;
}

void ZBot::_botcom_time_update_sync(ZBotMSG& msg){
	lock_guard<mutex> lock(_time_lock);
	_botcom_time[msg.sender_name]=_time;
}

double ZBot::_information_gain(int r, int c){
	int cell_offset=int(_sensor0_max_range/_occupancy_grid.CellSize())-1;
	int info_cnt=0;
	for (int i = -cell_offset; i < cell_offset+1; ++i)
	{
		for (int j = -cell_offset; j < cell_offset+1; ++j)
		{
			if ( (!_out_of_grid(r+i,c+j)) && _occupancy_grid.GetOccupancy(r+i,c+j)==0.5)
			{
				info_cnt++;
			}
		}
	}
	return double(info_cnt)/pow(cell_offset*2+1,2.0);
}

double ZBot::_information_gain_sync(int r, int c){
	lock_guard<mutex> lock(_occupancy_grid_lock);
	return _information_gain(r,c);
}
