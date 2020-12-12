#include "bot_factory.h"

GZ_REGISTER_WORLD_PLUGIN(gazebo::BotFactory)

gazebo::BotFactory::BotFactory()
{
	std::srand(6570);
	this->time_ = 1;
	this->bot_gap_ = 3.0;
	this->n_inserted_bot_ = 0;
	this->n_initial_bot_ = 10;
	this->bot_sx_ = this->bot_gap_;
	this->bot_sy_ = this->bot_gap_;
	this->n_active_bot_ = this->n_initial_bot_;
}

gazebo::BotFactory::~BotFactory()
{
	int quit = -1;
	this->bot_insertion_requests_.enq(quit);
	this->logger_.Close();
	this->bot_insertion_worker_->join();
	delete this->bot_insertion_worker_;
}

void gazebo::BotFactory::Load(physics::WorldPtr _parent, sdf::ElementPtr _ep)
{
	std::cout << "loading bot factory" << std::endl;
	this->world_ = _parent;
	this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&gazebo::BotFactory::OnUpdate, this, _1));
	this->bot_node_ = transport::NodePtr(new gazebo::transport::Node());
	this->bot_node_->Init();
	this->bot_sub_ = this->bot_node_->Subscribe("~/world/bot", &gazebo::BotFactory::BotMsgHandler, this);
	this->bot_pub_ = this->bot_node_->Advertise<msgs::Factory>("~/factory");
	this->bot_insertion_worker_ = new std::thread(&gazebo::BotFactory::InsertBots, this);
	this->server_control_node_ = transport::NodePtr(new gazebo::transport::Node());
	this->server_control_node_->Init();
	this->server_control_pub_ = server_control_node_->Advertise<msgs::ServerControl>("/gazebo/server/control"); 
	this->world_->SetPaused(true);
	this->bot_insertion_requests_.enq(this->n_initial_bot_);
	this->logger_.Open("world");
	std::cout << "bot factory is loaded" << std::endl;
}

void gazebo::BotFactory::OnUpdate(const common::UpdateInfo &_info)
{
	if (this->n_active_bot_==0)
	{
		this->logger_.Log(
			"exploration end time: "
			+ this->world_->SimTime().FormattedString(
				gazebo::common::Time::MINUTES,
				gazebo::common::Time::SECONDS
			)
		);
		msgs::ServerControl sc_msg;
		sc_msg.set_stop(true);
		this->server_control_pub_->Publish(sc_msg);
		this->n_active_bot_--;
	}
}

void gazebo::BotFactory::BotMsgHandler(ConstGzStringPtr &_msg)
{
	std::string d = _msg->data();
	int p = d.find(":");
	std::string bot_name = d.substr(0, p);
	std::string bot_req = d.substr(p+1);
	if (bot_req.compare("deactivate") == 0)
	{		
		//this->n_active_bot_--;
		this->n_active_bot_=0;
	}
}

void gazebo::BotFactory::InsertBots()
{
	while(true){
		int n_bot = this->bot_insertion_requests_.deq();
		if (n_bot < 0)
		{
			return;
		}
		this->world_->SetPaused(true);
		std::cout << "inserting robots" << std::endl;
		for (int i = 0; i < n_bot; ++i)
		{
			float x = this->bot_sx_+this->bot_gap_*(i%3);
			float y = this->bot_sy_+this->bot_gap_*(i/3);
			InsertOneBot(x, y);
			this->n_inserted_bot_++;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		this->logger_.Log(
			"exploration start time: "
			+ this->world_->SimTime().FormattedString(
				gazebo::common::Time::MINUTES,
				gazebo::common::Time::SECONDS
			)
		);
		this->world_->SetPaused(false);
	}
}

void gazebo::BotFactory::InsertOneBot(double _x, double _y)
{
	//robots should be inserted one by one, and they all should start from [0,0]
	sdf::SDF bot_sdf;
	bot_sdf.SetFromString(
		"<?xml version='1.0'?>\
		<sdf version='1.6'>\
		  <model name='bot"+std::to_string(this->n_inserted_bot_)+"'>\
		    <static>false</static>\
		    <self_collide>false</self_collide>\
		    <pose>"+std::to_string(_x)+" "+std::to_string(_y)+" 0 0 0 0</pose>\
		    <link name='chassis'>\
		      <inertial>\
		        <mass>40.0</mass>\
		      </inertial>\
		      <pose>0 0 .1 0 0 0</pose>\
		      <collision name='collision'>\
		        <geometry>\
		          <cylinder>\
					<radius>0.2</radius>\
					<length>0.1</length>\
				  </cylinder>\
		        </geometry>\
		      </collision>\
		      <visual name='visual'>\
		        <geometry>\
		          <cylinder>\
					<radius>0.2</radius>\
					<length>0.1</length>\
				  </cylinder>\
		        </geometry>\
		        <material>\
		            <ambient>0 1 0 1</ambient>\
		            <diffuse>0 0.5 0.5 1</diffuse>\
		        </material>\
		      </visual>\
		      <visual name='svisual'>\
		        <pose>0 0 0.15 0 0 0</pose>\
		        <geometry>\
		          <cylinder>\
					<radius>0.1</radius>\
					<length>0.2</length>\
				  </cylinder>\
		        </geometry>\
		      </visual>\
		    </link>\
		    <link name='sensor0'>\
		      <sensor name='laser' type='ray'>\
		        <pose>0 0 0.3 0 0 0</pose>\
		        <ray>\
		          <scan>\
		            <horizontal>\
		              <samples>360</samples>\
		              <resolution>1</resolution>\
		              <min_angle>-3.141592654</min_angle>\
		              <max_angle>3.141592654</max_angle>\
		            </horizontal>\
		          </scan>\
		          <range>\
		            <min>0.2</min>\
		            <max>5</max>\
		            <resolution>0.01</resolution>\
		          </range>\
		          <noise>\
		            <type>gaussian</type>\
		            <mean>0</mean>\
		            <stddev>0.001</stddev>\
		          </noise>\
		        </ray>\
		        <plugin name='sensor0_plgin' filename='./build/liblaser_sensor.so' />\
		        <always_on>true</always_on>\
		        <update_rate>30</update_rate>\
		        <visualize>true</visualize>\
		      </sensor>\
		    </link>\
		    <link name='sensor0_body'>\
		      <inertial>\
		        <mass>1.0</mass>\
		      </inertial>\
		      <pose>0 0 .25 0 0 0</pose>\
		      <collision name='collision'>\
		        <geometry>\
		          <cylinder>\
					<radius>0.1</radius>\
					<length>0.2</length>\
				  </cylinder>\
		        </geometry>\
		      </collision>\
		      <visual name='visual'>\
		        <geometry>\
		          <cylinder>\
					<radius>0.1</radius>\
					<length>0.2</length>\
				  </cylinder>\
		        </geometry>\
		      </visual>\
		    </link>\
		    <joint name='sensor0_joint' type='fixed'>\
		      <child>sensor0</child>\
		      <parent>chassis</parent>\
		    </joint>\
		    <joint name='sensor0_body_joint' type='fixed'>\
		      <child>sensor0_body</child>\
		      <parent>chassis</parent>\
		    </joint>\
		    <plugin name='bot_controller' filename='./build/libbot_controller.so'/>\
		  </model>\
		</sdf>"
	);
	this->world_->InsertModelSDF(bot_sdf);
}




