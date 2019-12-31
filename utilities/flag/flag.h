#ifndef FLAG_H
#define FLAG_H
#include <mutex>
namespace gazebo{
	template<typename T>
	class Flag{
		public:
			Flag();
			Flag(T value);
			~Flag();
			T get();
			void set_asyn(T value);
			bool equals_asyn(T value);
			bool operator==(const T& value);
			gazebo::Flag<T>& operator=(const T& value);
		private:
			T _flag;
			std::mutex _mutex;
	};
}

template<typename T>
gazebo::Flag<T>::Flag(){
	_flag=T();
}

template<typename T>
gazebo::Flag<T>::Flag(T value){
	_flag=value;
}

template<typename T>
gazebo::Flag<T>::~Flag(){}

template<typename T>
T gazebo::Flag<T>::get(){
	std::unique_lock<std::mutex> lock(_mutex);
	return _flag;
}

template<typename T>
void gazebo::Flag<T>::set_asyn(T value){
	_flag=value;
}

template<typename T>
bool gazebo::Flag<T>::equals_asyn(T value){
	return _flag==value;
}

template<typename T>
bool gazebo::Flag<T>::operator==(const T& v){
	std::unique_lock<std::mutex> lock(_mutex);
	return _flag==v;
}

template<typename T>
gazebo::Flag<T>& gazebo::Flag<T>::operator=(const T& v){
	std::unique_lock<std::mutex> lock(_mutex);
	_flag=v;
	return *this;
}

#endif
