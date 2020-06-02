// 
// 
// 

#include "LogHelper.h"


/**
 * @brief Log timestamp generator function
 * @param _logOutput The stream to write on
*/
void printTimestamp( Print* _logOutput )
{
	char c[12];
	sprintf( c, "%10lu ", millis() );
	_logOutput->print( c );
}

/**
 * @brief Adds return line feed to the end of log records
 * @param _logOutput The stream to wrtie on
*/
void printNewline( Print* _logOutput )
{
	_logOutput->print( "\r\n" );
	_logOutput->flush();
}
