// 
// 
// 

#include "Queue.h"
#include <cstdlib>

// Constructor to initialize queue
Queue::Queue( int size )
{
	arr = new char*[size];
	capacity = size;
	front = 0;
	rear = -1;
	count = 0;
}

// Destructor to free memory allocated to the queue
Queue::~Queue()
{
	delete arr;
}

// Utility function to remove front element from the queue
char * Queue::dequeue()
{
	char* item;
	// check for queue underflow
	if ( isEmpty() )
	{
		Log.error( "Queue was empty when dequeue attempted" );
		return nullptr;
	}

	item = arr[front];

	front = (front + 1) % capacity;
	count--;

	return item;
}

// Utility function to add an item to the queue
void Queue::enqueue( char* item )
{
	// check for queue overflow
	if ( isFull() )
	{
		Log.error( "Queue was full when enqueue attempted" );
	}

	rear = (rear + 1) % capacity;
	arr[rear] = item;
	count++;
}

// Utility function to return front element in the queue
char*  Queue::peek()
{
	if ( isEmpty() )
	{
	}
	return arr[front];
}

// Utility function to return the size of the queue
int Queue::size()
{
	return count;
}

// Utility function to check if the queue is empty or not
bool Queue::isEmpty()
{
	return (size() == 0);
}

// Utility function to check if the queue is full or not
bool Queue::isFull()
{
	return (size() >= capacity);
}
