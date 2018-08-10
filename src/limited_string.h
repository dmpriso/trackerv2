#ifndef LIMITED_STRING_H
#define LIMITED_STRING_H

#include "const_string.h"

/**
 * String with limited capacity
 */
template<int Capacity>
class LimitedString : public ConstString
{
public:
	LimitedString()
	{
		this->m_cbuf = this->m_buf;
	}

	inline LimitedString(const char* str, int len = -1)
	{
		this->m_cbuf = this->m_buf;

		this->append(str, len);
	}

	inline LimitedString(const __FlashStringHelper* str, int len = -1)
		: ConstString(m_buf, len)
	{
		this->m_cbuf = this->m_buf;

		this->append(str, len);
	}

	template<int S>
	inline LimitedString(const LimitedString<S>& cpy)
		: LimitedString<Capacity>(cpy.c_str(), cpy.length())
	{
	}

	template<int S>
	inline LimitedString& operator=(const LimitedString<S>& assign)
	{
		this->clear();
		this->append(assign.c_str(), assign.length());
		return *this;
	}

	inline LimitedString& operator=(const char* str) { this->append(str); return *this; }
	inline LimitedString& operator=(const __FlashStringHelper* str) { this->append(str); return *this; }

public:
	void clear();
	void append(const char* str, int len = -1);
	void append(const __FlashStringHelper* fstr, int len = -1);
	inline LimitedString<Capacity>& operator+= (const char* str) { this->append(str); return *this; }
	inline LimitedString<Capacity>& operator+= (const __FlashStringHelper* str) { this->append(str); return *this; }
	template<int S>
	inline LimitedString<Capacity>& operator+= (const LimitedString<S>& str) { this->append(str.c_str(), str.length()); return *this; };

	void remove(int idx, int cnt = -1);

	template<int S = Capacity>
	LimitedString<S> substring(int idx, int cnt) const;

	template<int S = Capacity>
	inline LimitedString<S> left(int cnt) const
	{
		return this->substring<S>(0, min(cnt, this->length()));
	}

	template<int S = Capacity>
	inline LimitedString<S> right(int cnt) const
	{
		cnt = min(cnt, this->length());
		return this->substring<S>(this->length() - cnt, cnt);
	}

	using ConstString::compareTo;
	template<int S>
	inline int compareTo(const LimitedString<S>& str) const { return this->compareTo(str.c_str(), str.length()); }

	using ConstString::operator==;
	template<int S>
	inline bool operator==(const LimitedString<S>& str) const { return 0 == this->compareTo(str); }
	using ConstString::operator!=;
	template<int S>
	inline bool operator!=(const LimitedString<S>& str) const { return 0 != this->compareTo(str); }

public:
	inline static int capacity() { return Capacity; }

private:
	inline void terminate() { this->m_buf[this->m_length] = '\0'; }

private:
	char m_buf[Capacity + 1];

};

template<int Capacity>
void LimitedString<Capacity>::clear()
{
	this->m_length = 0;
	this->terminate();
}

template<int Capacity>
void LimitedString<Capacity>::append(const char* str, int len)
{
	if (-1 == len)
		len = strlen(str);

	int newlen = min(this->m_length + len, Capacity);
	for (int i = 0; i < newlen - this->m_length; i++)
	{
		this->m_buf[i + this->m_length] = str[i];
	}

	this->m_length = newlen;
	this->terminate();
}

template<int Capacity>
void LimitedString<Capacity>::append(const __FlashStringHelper* fstr, int len)
{
	auto str = (PGM_P)fstr;
	while (this->m_length < Capacity && (len > 0 || len == -1))
	{
		auto chr = pgm_read_byte(str++);
		if (len == -1 && chr == 0) 
			break;
		if (len > 0) 
			--len;

		this->m_buf[this->m_length++] = chr;
	}
	this->terminate();
}

template<int Capacity>
void LimitedString<Capacity>::remove(int idx, int cnt)
{
	if (-1 == cnt) cnt = this->length() - idx;

	for (int i = idx; i < this->length() - cnt; i++)
	{
		this->m_buf[i] = this->m_buf[i + cnt];
	}
	this->m_length -= cnt;
	this->terminate();
}

template<int Capacity>
template<int S>
LimitedString<S> LimitedString<Capacity>::substring(int idx, int cnt) const
{
	return LimitedString<S>(this->m_buf + idx, cnt);
}



class LimitedString_TestDriver
{
public:
	static int test();
};

#endif



