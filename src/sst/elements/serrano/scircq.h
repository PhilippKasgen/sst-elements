
#ifndef _H_SERRANO_CIRC_Q
#define _H_SERRANO_CIRC_Q

#include <cstdint>

namespace SST {
namespace Serrano {

template<class T>
class SerranoCircularQueue {
public:
	SerranoCircularQueue( const size_t size ) :
		max_capacity(size) {

		front = 0;
		back = 0;

		count = 0;
		data = new T[size];
	}

	~SerranoCircularQueue() {
		delete[] data;
	}

	bool empty() {
		return (front == back);
	}

	bool full() {
		return ( safe_inc(back) == front );
	}

	void push(T item) {
		data[back] = item;
		back = safe_inc(back);
		count++;
	}

	T peek() {
		return data[front];
	}

	T peek( const size_t index ) {
		return data[ (front+index) % max_capacity ];
	}

	T pop() {
		T temp = data[front];
		front = safe_inc(front);
		count--;
		return temp;
	}

	size_t size() const {
		return count;
	}

	size_t capacity() const {
		return max_capacity;
	}

	void clear() {
		front = 0;
		back  = 0;
	}

private:
	size_t safe_inc(size_t v) {
		return (v+1) % max_capacity;
	}

	size_t front;
	size_t back;
	size_t count;
	const size_t max_capacity;
	T* data;

};

}
}

#endif