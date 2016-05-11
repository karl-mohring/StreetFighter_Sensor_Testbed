#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "StraightBuffer.h"

/**
* Initialise the buffer with the specified size
* @param maxSize Size of the buffer in bytes
*/
StraightBuffer::StraightBuffer(byte *buffer, int maxSize){
	_buffer = buffer;
	_maxCapacity = maxSize;

	reset();
}

StraightBuffer::StraightBuffer(int maxSize){
	_buffer = (byte*)malloc(sizeof(byte)*maxSize);
	_maxCapacity = maxSize;

	reset();
}


/**
* Clear the data in the buffer
*/
void StraightBuffer::reset(){
	_putter = 0;
	_getter = 0;

	// Zero the buffer
	for (int i = 0; i < _maxCapacity; i++){
		_buffer[i] = 0;
	}
}


/**
* Get the position of the buffer writer
*
* Returns:
*	Current putter position
*/
int StraightBuffer::getWritePosition(){
	return _putter;
}


/**
* Get the position of the buffer reader
* @return Current position of the getter
*/
int StraightBuffer::getReadPosition(){
	return _getter;
}


/**
* Get the number of remaining byte in the buffer
* @return Number of bytes left in the buffer
*/
int StraightBuffer::getNumRemaining(){
	return _putter - _getter;
}


/**
* Set the read position of the buffer
* @param index: New index of the buffer reader
*/
void StraightBuffer::setReadPosition(int index){
	index = constrain(index, 0, _maxCapacity);
	_getter = index;
}



/**
* Set the write position of the buffer
* @param index: New index of the buffer writer
*/
void StraightBuffer::setWritePosition(int index){
	index = constrain(index, 0, _maxCapacity);
	_putter = index;
}


/**
* Get the starting address of the buffer in memory
*
* Returns:
*	The byte address of the buffer
*/
byte* StraightBuffer::getBufferAddress(){
	return _buffer;
}

bool StraightBuffer::available(){
	bool available = false;

	if (_getter < _putter){
		available = true;
	}
}


//////////////////////////////////////////////////////////////////////////
// Putters

/**
* Put a byte in the buffer
* Will do nothing if the buffer is already full
*
* @param in The byte to put into the buffer
*/
void StraightBuffer::write(byte in){
	if (_putter < _maxCapacity){
		_buffer[_putter] = in;
		_putter++;
	}
}


/**
* Put an integer in the buffer
* Will do nothing if the buffer is already full
*
*@param in Integer to put into the buffer
*/
void StraightBuffer::writeInt(int in){
	byte *pointer = (byte *)&in;
	write(pointer[1]);
	write(pointer[0]);
}


/**
*
*/
void StraightBuffer::writeLong(long in){
	byte *pointer = (byte *)&in;
	write(pointer[3]);
	write(pointer[2]);
	write(pointer[1]);
	write(pointer[0]);
}


//////////////////////////////////////////////////////////////////////////
// Getters

/**
* Get a byte from the buffer
* Returns a null character if the buffer has reached the end
* @return Byte from the buffer; Null if at end of the buffer
*/
byte StraightBuffer::read(){
	byte out;

	if (_getter < _putter){
		out = _buffer[_getter];
		_getter++;
	}

	else{
		out = 0;
	}

	return out;
}


/**
* Get an integer out of the buffer
* @return Integer from the buffer
*/
int StraightBuffer::readInt(){
	int out;
	byte *pointer = (byte *)&out;
	pointer[1] = read();
	pointer[0] = read();
}


/**
* Get a long from the buffer
* @return Long from the buffer
*/
long StraightBuffer::readLong(){
	long out;
	byte *pointer = (byte *)&out;
	pointer[3] = read();
	pointer[2] = read();
	pointer[1] = read();
	pointer[0] = read();
}


/**
* Get the next character from the buffer without taking it out
* @return Next byte from the buffer
*/
byte StraightBuffer::peek(){
	return _buffer[_getter];
}

void StraightBuffer::destroy(){
	free(_buffer);
}
