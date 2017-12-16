#ifndef SERIAL_PROCESSOR_H
#define SERIAL_PROCESSOR_H

#include <Arduino.h>
#include "limited_string.h"
#include "callback.h"

/**
 * Helper class for safe reading and writing from/to a stream
 * Reads data bytewise and provides functionality to read full lines etc.
 */
template<size_t ReadbufSize, size_t WritebufSize>
class StringStreamProcessor
{
public:
	StringStreamProcessor(Stream& stream, char newline = '\n', char trimline = '\r');

public:
	// to be called regularly, e.g. from main loop
	void process();

public:
	inline void forceWrite(bool bForce) { this->m_bForceWrite = bForce; }

	/**
	 * Nonblocking read of a character only if it's available
	 * @return true if char has been read
	 */
	bool peekChar(char& chr);

	/**
	 * Reads a character and calls callback when character has been read.
	 * @param cb Signature: void(char chr, int error)
	 */
	template<typename CB>
	void readChar(CB&& cb, bool bAllowImmedCb = false, int iTimeoutMs = 30000);

	/**
	 * Reads a full line and calls callback when finished
	 * @param cb Signature: void(const ConstString& line, int error)
	 */
	template<typename CB>
	void readLine(CB&& cb, bool bAllowImmedCb = false, int iTimeoutMs = 30000);

	/**
	 * @return error if unsuccessful
	 */
	int write_forget(const char* buf, int len);

	/**
	 * @param cb Signature: void(int err)
	 * @return error if starting write op was unsuccessful
	 */
	template<typename CB>
	int write(CB&& cb, const char* buf, int len, bool bAllowImmedCb = false);

	/**
	 * Special purpose function: Breaks any peanding read operations (if there are any)
	 * with the given result
	 */
	void breakPendingRead(char chrRead, const ConstString& strRead, int err);

private:
	void flushSend();
	void tryRead();
	void processRead();

	void setReadTimeout(int iTimeoutMs);

	void callCharCallback(char chr, int err = 0);
	void callLineCallback(const ConstString& line, int err = 0);
	void callSendCallback(int err = 0);

private:
	Stream& m_stream;

	LimitedString<ReadbufSize> m_readbuf;
	LimitedString<WritebufSize> m_writebuf;

	uint32_t m_toRead = 0; // this is the millis() value where the current read operation will time out
	Callback<void(char, int)>* m_pcbChar = nullptr;
	Callback<void(const ConstString&, int)>* m_pcbLine = nullptr;
	Callback<void(int)>* m_pcbSend = nullptr;

	const char m_newline;
	const char m_trimline;

	bool m_bForceWrite = false;
};

template<size_t ReadbufSize, size_t WritebufSize>
StringStreamProcessor<ReadbufSize, WritebufSize>::StringStreamProcessor(Stream& stream,
	char newline, char trimline)
	: m_stream(stream),
	m_newline(newline),
	m_trimline(trimline)
{
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::process()
{
	this->flushSend();
	this->tryRead();
	this->processRead();
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::setReadTimeout(int msTimeout)
{
	this->m_toRead = (msTimeout < 0 ? 0 : millis() + (long)msTimeout);
}

template<size_t ReadbufSize, size_t WritebufSize>
bool StringStreamProcessor<ReadbufSize, WritebufSize>::peekChar(char& chr)
{
	if (this->m_stream.available())
	{
		chr = this->m_stream.read();
		return true;
	}
	return false;
}

template<size_t ReadbufSize, size_t WritebufSize>
template<typename CB>
void StringStreamProcessor<ReadbufSize, WritebufSize>::readChar(CB&& cb, bool bAllowImmedCb, int iTimeoutMs)
{
	this->setReadTimeout(iTimeoutMs);

	this->m_pcbChar = new Callback_Impl<void(char, int), CB>(move(cb));
	if (bAllowImmedCb)
		this->processRead();
}

template<size_t ReadbufSize, size_t WritebufSize>
template<typename CB>
void StringStreamProcessor<ReadbufSize, WritebufSize>::readLine(CB&& cb, bool bAllowImmedCb, int iTimeoutMs)
{
	this->setReadTimeout(iTimeoutMs);

	this->m_pcbLine = new Callback_Impl<void(const ConstString&, int), CB>(move(cb));
	if (bAllowImmedCb)
		this->processRead();
}

template<size_t ReadbufSize, size_t WritebufSize>
int StringStreamProcessor<ReadbufSize, WritebufSize>::write_forget(const char* buf, int len)
{
	if (this->m_writebuf.length() + len > this->m_writebuf.capacity())
		return 1;
	this->m_writebuf.append(buf, len);

	this->flushSend();

	return 0;
}

template<size_t ReadbufSize, size_t WritebufSize>
template<typename CB>
int StringStreamProcessor<ReadbufSize, WritebufSize>::write(CB&& cb, const char* buf, int len, bool bAllowImmedCb)
{
	if (this->m_writebuf.length() + len > this->m_writebuf.capacity())
		return 1;
	this->m_writebuf.append(buf, len);

	this->m_pcbWrite = new Callback_Impl<void(int), CB>(move(cb));
	if (bAllowImmedCb)
		this->flushSend();

	return 0;
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::flushSend()
{
	if ((this->m_bForceWrite || this->m_stream.availableForWrite()) && this->m_writebuf.length() > 0)
	{
		this->m_stream.write(this->m_writebuf.c_str()[0]);
		this->m_writebuf.remove(0, 1);

		if (this->m_writebuf.length() == 0 && this->m_pcbSend != nullptr)
		{
			this->callSendCallback();
		}
	}
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::tryRead()
{
	if (this->m_stream.available() && 
		(this->m_pcbChar != nullptr || this->m_pcbLine != nullptr))
	{
		char chr = (char)this->m_stream.read();

		if (this->m_readbuf.length() >= this->m_readbuf.capacity())
		{
			// buffer overflow!
			if (this->m_pcbChar != nullptr)
				this->callCharCallback(0, 1);
			else if (this->m_pcbLine != nullptr)
				this->callLineCallback(this->m_readbuf, 1);
		}
		else
			this->m_readbuf.append(&chr, 1);
	}
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::processRead()
{
	bool bCallbackCalled = false;
	if (this->m_readbuf.length() > 0)
	{
		if (this->m_pcbChar != nullptr)
		{
			auto chr = this->m_readbuf.c_str()[0];
			this->m_readbuf.remove(0, 1);
			this->callCharCallback(chr);
			bCallbackCalled = true;
		}
		else if (this->m_pcbLine != nullptr)
		{
			if (this->m_readbuf.endsWith(this->m_newline))
			{
				this->m_readbuf.remove(this->m_readbuf.length() - 1);
				if (this->m_readbuf.endsWith(this->m_trimline))
				{
					this->m_readbuf.remove(this->m_readbuf.length() - 1);
				}

				Serial.println(this->m_readbuf.c_str());

				this->callLineCallback(this->m_readbuf);
				this->m_readbuf.clear();
				bCallbackCalled = true;
			}
		}
	}

	if (!bCallbackCalled && this->m_toRead > 0 && millis() > this->m_toRead)
	{
		// we do have a read timeout
		if (this->m_pcbChar != nullptr)
			this->callCharCallback(0, 2);
		else if (this->m_pcbLine != nullptr)
			this->callLineCallback(0, 2);
	}
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::breakPendingRead(char chrRead, const ConstString& strRead, int err)
{
	if (this->m_pcbChar != nullptr)
		this->callCharCallback(chrRead, err);
	else if (this->m_pcbLine != nullptr)
		this->callLineCallback(strRead, err);
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::callCharCallback(char chr, int err)
{
	auto cb = this->m_pcbChar;
	this->m_pcbChar = nullptr;
	(*cb)(chr, err);
	delete cb;
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::callLineCallback(const ConstString& line, int err)
{
	auto cb = this->m_pcbLine;
	this->m_pcbLine = nullptr;
	(*cb)(line, err);
	delete cb;
}

template<size_t ReadbufSize, size_t WritebufSize>
void StringStreamProcessor<ReadbufSize, WritebufSize>::callSendCallback(int err)
{
	auto cb = this->m_pcbSend;
	this->m_pcbSend = nullptr;
	(*cb)(err);
	delete cb;
}


#endif


