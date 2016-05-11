
#ifndef StraightBuffer_h
#define StraightBuffer_h


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class StraightBuffer{
	public:

	// Constructor
	StraightBuffer(byte *, int);
	StraightBuffer(int);

	// General I/O
	void reset();
	int getWritePosition();
	int getReadPosition();
	int getNumRemaining();
	void setWritePosition(int index);
	void setReadPosition(int index);
	byte* getBufferAddress();
	bool available();
	void destroy();

	// Writers
	void write(byte in);
	void writeInt(int in);
	void writeLong(long in);

	// Readers
	byte read();
	int	readInt();
	long readLong();
	byte peek();

	private:
	byte* _buffer;
	int _putter;
	int _getter;
	int _maxCapacity;

};

#endif
