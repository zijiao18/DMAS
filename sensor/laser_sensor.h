#ifndef LASER_SENSOR_H
#define LASER_SENSOR_H

#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <climits>
#include <cmath>
#include "../utilities/flag/flag.h"

namespace gazebo {
    class LaserSensor: public SensorPlugin{
    private:
        transport::NodePtr _bot_node;
        transport::PublisherPtr _bot_pub;
        sensors::RaySensorPtr _parentSensor;
        event::ConnectionPtr _updateConnection;
        double _width, _height;
    public:
        LaserSensor();
        virtual ~LaserSensor();
        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
        virtual void OnUpdate();
    };
}

#endif
