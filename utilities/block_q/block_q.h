#ifndef BLOCK_Q_H
#define BLOCK_Q_H
#include <queue>
#include <mutex>
#include <condition_variable>
namespace gazebo{
	template<typename T>
	class BlockQ{
		public:
			BlockQ();
			~BlockQ();
			void enq(T& e);
			T deq();
			int size();
			void clear();
		private:
			std::queue<T> _q;
			std::mutex _mutex;
			std::condition_variable _condition;
	};
}

template<typename T>
gazebo::BlockQ<T>::BlockQ(){}

template<typename T>
gazebo::BlockQ<T>::~BlockQ(){}

template<typename T>
T gazebo::BlockQ<T>::deq(){
	std::unique_lock<std::mutex> lock(_mutex);
	while(_q.empty()){
		_condition.wait(lock);  // the lock will be released. thread sleeps on _condition
	}
	T head=_q.front();
	_q.pop();
	return head;
}

template<typename T>
void gazebo::BlockQ<T>::enq(T& e){
	std::unique_lock<std::mutex> lock(_mutex);
	_q.push(e);
	lock.unlock();
	_condition.notify_one();  // wake up one thread sleeping on the _condition
}

template<typename T>
int gazebo::BlockQ<T>::size(){
	std::unique_lock<std::mutex> lock(_mutex);
	return _q.size();
}

template<typename T>
void gazebo::BlockQ<T>::clear(){
	std::unique_lock<std::mutex> lock(_mutex);
	while(!_q.empty()){
		_q.pop();
	}
}

#endif
