#include "const_string.h"

ConstString::ConstString(const char* buf, int len)
	: m_cbuf(buf),
	m_length(len == -1 ? strlen(buf) : len)
{
}

int ConstString::indexOf(char chr, int idStart) const
{
	for (int i = idStart; i < this->length(); i++)
		if (this->m_cbuf[i] == chr)
			return i;
	return -1;
}


bool ConstString::startsWith(const char* str, int len) const
{
	if (-1 == len)
		len = strlen(str);
	if (len > this->length())
		return false;
	auto cmplen = min(this->length(), len);
	return 0 == StringHelper::compareString(this->c_str(), str, cmplen, cmplen);
}


bool ConstString::startsWith(const __FlashStringHelper* str, int len) const
{
	if (-1 == len)
		len = strlen_P((PGM_P)str);
	if (len > this->length())
		return false;
	auto cmplen = min(this->length(), len);
	return 0 == StringHelper::compareString(this->c_str(), str, cmplen, cmplen);
}


bool ConstString::startsWith(char chr) const
{
	return (this->length() >= 1 && this->c_str()[0] == chr);
}


bool ConstString::endsWith(const char* str, int len) const
{
	if (-1 == len)
		len = strlen(str);
	if (len > this->length())
		return false;
	auto cmplen = min(this->length(), len);
	return 0 == StringHelper::compareString(this->c_str() + this->length() - len, str, cmplen, cmplen);
}


bool ConstString::endsWith(const __FlashStringHelper* str, int len) const
{
	if (-1 == len)
		len = strlen_P((PGM_P)str);
	if (len > this->length())
		return false;
	auto cmplen = min(this->length(), len);
	return 0 == StringHelper::compareString(this->c_str() + this->length() - len, str, cmplen, cmplen);
}


bool ConstString::endsWith(char chr) const
{
	return (this->length() >= 1 && this->c_str()[this->length() - 1] == chr);
}

