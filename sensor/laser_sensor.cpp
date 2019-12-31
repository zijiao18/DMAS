#include "laser_sensor.h"
GZ_REGISTER_SENSOR_PLUGIN(gazebo::LaserSensor)

gazebo::LaserSensor::LaserSensor(){}

gazebo::LaserSensor::~LaserSensor(){}

void gazebo::LaserSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
	this->_parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
	if (!this->_parentSensor)
	{
		std::cout << "LaserSensorPlugin not attached to a Laser sensor\n";
		return;
	}
	this->_width = this->_parentSensor->RangeCount();
	this->_height = this->_parentSensor->VerticalRangeCount();

    this->_updateConnection = this->_parentSensor->ConnectUpdated(
    	std::bind(&LaserSensor::OnUpdate, this)
    );
    this->_parentSensor->SetActive(true);
	_bot_node=transport::NodePtr(
		new gazebo::transport::Node()
	); 
	_bot_node->Init();
	_bot_pub=_bot_node->Advertise<msgs::GzString>(
		"~/"
		+ _parentSensor->ParentName()
		+ "/data"
	);
}

void gazebo::LaserSensor::OnUpdate(){
    std::vector<double> data;
    std::string data_str="";
	_parentSensor->Ranges(data);
	for(int i=0;i<data.size();i++)
	{
		if (std::isinf(data[i]))
		{
			data_str+="-1 ";
		}else{
			data_str+=(std::to_string(data[i])+" ");
		}
    }
    msgs::GzString data_msg;
    data_msg.set_data(data_str);
    _bot_pub->Publish(data_msg);
}




