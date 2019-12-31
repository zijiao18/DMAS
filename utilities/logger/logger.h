#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <stdio.h>

namespace gazebo{

	/*Each logger is designed to 
	manipulate only one file*/
	class Logger{
		public:
			Logger();
			Logger(std::string file_name);
			~Logger();
			void Open(std::string file_name);
			void Close();
			void Log(std::string line);
		private:
			std::ofstream _file_out;
			std::string _file_name;
			
	};
}

#endif
