#ifndef CONST_STRING_H
#define CONST_STRING_H

#include <Arduino.h>

class StringHelper
{
public:
	static int compareString(const char* str1, const char* str2, int len1 = -1, int len2 = -1);
	static int compareString(const char* str1, const __FlashStringHelper* str2, int len1 = -1, int len2 = -1);
};

class ConstString
{
public:
	ConstString() = default;
	ConstString(const char* buf, int len = -1);
	
	ConstString(ConstString& cpy) = default;
	ConstString& operator=(ConstString& cpy) = default;

public:
	int indexOf(char chr, int idStart = 0) const;

	inline int compareTo(const char* str, int len = -1) const { return StringHelper::compareString(this->c_str(), str, this->length(), len); }
	inline int compareTo(const __FlashStringHelper* str, int len = -1) const { return StringHelper::compareString(this->c_str(), str, this->length(), len); }

	inline bool operator==(const char* str) const { return 0 == this->compareTo(str); }
	inline bool operator==(const __FlashStringHelper* str) const { return 0 == this->compareTo(str); }
	inline bool operator!=(const char* str) const { return 0 != this->compareTo(str); }
	inline bool operator!=(const __FlashStringHelper* str) const { return 0 != this->compareTo(str); }

	bool startsWith(const char* str, int len = -1) const;
	bool startsWith(const __FlashStringHelper* str, int len = -1) const;
	bool startsWith(char chr) const;

	bool endsWith(const char* str, int len = -1) const;
	bool endsWith(const __FlashStringHelper* str, int len = -1) const;
	bool endsWith(char chr) const;


public:
	inline const char* c_str() const { return this->m_cbuf; };
	inline int length() const { return this->m_length; }

protected:
	const char* m_cbuf = nullptr;
	int m_length = 0;

};


#endif


