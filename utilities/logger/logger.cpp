#include "logger.h"

gazebo::Logger::Logger(){}

gazebo::Logger::Logger(std::string file_name){
	Open(file_name);
}

gazebo::Logger::~Logger(){
	Close();
}

void gazebo::Logger::Open(std::string file_name){
	if (_file_out.is_open()){
		_file_out.close();
	}
	std::ifstream file_exist("./log/"+file_name);
	if (file_exist){
		file_exist.close();
		std::cout<<"previous "<<file_name<<" is replaced"<<std::endl;
		std::remove(("./log/"+file_name).c_str());
		
	}
	_file_out.open(("./log/"+file_name), std::ios_base::out | std::ios_base::app);	
	std::cout<<file_name<<" is open"<<std::endl;
}

void gazebo::Logger::Log(std::string line){
	if (_file_out.is_open()){
		_file_out << line <<std::endl;
	}else{
		std::cout<<"logger error: write the file which is not open"<<std::endl;
		exit(1);
	}
	
}

void gazebo::Logger::Close(){
	if (_file_out.is_open()){
		_file_out.close();
	}
}
