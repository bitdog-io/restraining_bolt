// Queue.h

#ifndef _QUEUE_h
#define _QUEUE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "ArduinoLog.h"

// define default capacity of the queue
constexpr auto QUEUE_SIZE = 20;

// Class for Queue
class Queue
{
	const char** _arr;		// array to store queue elements
	int _capacity;	// maximum capacity of the queue
	int _front;		// front points to front element in the queue (if any)
	int _rear;		// rear points to last element in the queue
	int _count;		// current size of the queue

public:
	Queue( int size = QUEUE_SIZE );		// constructor
	~Queue();					// destructor

	const char* dequeue();
	void enqueue( const char* item );
	const char* peek();
	int size();
	bool isEmpty();
	bool isFull();
};


#endif

