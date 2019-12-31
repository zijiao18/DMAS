#ifndef BOT_FACTORY_H
#define BOT_FACTORY_H
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <thread>
#include "../utilities/block_q/block_q.h"
#include "../utilities/logger/logger.h"

namespace gazebo
{
	class BotFactory : public WorldPlugin
	{
		public: 
			BotFactory();
			~BotFactory();
		  	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
		  	void OnUpdate(const common::UpdateInfo &_info);  	
		  	void BotMsgHandler(ConstGzStringPtr &_msg);
			void InsertOneBot(double _x, double _y);
			void InsertBots();
		private:
			physics::WorldPtr world_;
			event::ConnectionPtr updateConnection_;
			Logger logger_;
			int time_;
			double bot_gap_;
			double bot_sx_;
			double bot_sy_;
			int n_inserted_bot_;
			int n_initial_bot_;
			// exploration is done, if n_exp_bot==0
			int n_active_bot_;
			BlockQ<int> bot_insertion_requests_;
			std::thread *bot_insertion_worker_;
			transport::NodePtr bot_node_;
			transport::PublisherPtr bot_pub_;
			transport::SubscriberPtr bot_sub_;
			transport::NodePtr server_control_node_;
			transport::PublisherPtr server_control_pub_;
	};
}

#endif
