// 
// 
// 

#include "Queue.h"
#include <cstdlib>

// Constructor to initialize queue
Queue::Queue( int size )
{
	_arr = new const char*[size];
	_capacity = size;
	_front = 0;
	_rear = -1;
	_count = 0;
}

// Destructor to free memory allocated to the queue
Queue::~Queue()
{
	delete _arr;
}

// Utility function to remove front element from the queue
const char * Queue::dequeue()
{
	const char* item;
	// check for queue underflow
	if ( isEmpty() )
	{
		Log.error( "Queue was empty when dequeue attempted" );
		return nullptr;
	}

	item = _arr[_front];

	_front = (_front + 1) % _capacity;
	_count--;

	return item;
}

// Utility function to add an item to the queue
void Queue::enqueue(const char* item )
{
	// check for queue overflow
	if ( isFull() )
	{
		Log.error( "Queue was full when enqueue attempted" );
	}

	_rear = (_rear + 1) % _capacity;
	_arr[_rear] = item;
	_count++;
}

// Utility function to return front element in the queue
const char*  Queue::peek()
{
	if ( isEmpty() )
	{
	}
	return _arr[_front];
}

// Utility function to return the size of the queue
int Queue::size()
{
	return _count;
}

// Utility function to check if the queue is empty or not
bool Queue::isEmpty()
{
	return (size() == 0);
}

// Utility function to check if the queue is full or not
bool Queue::isFull()
{
	return (size() >= _capacity);
}
