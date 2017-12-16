#include "limited_string.h"

int StringHelper::compareString(const char* str1, const char* str2, int len1, int len2)
{
	if (-1 == len1)
		len1 = strlen(str1);
	if (-1 == len2)
		len2 = strlen(str2);

	for (int i = 0; i < min(len1, len2); i++)
	{
		auto chr1 = str1[i];
		auto chr2 = str2[i];

		if (chr1 < chr2)
			return -1;
		else if (chr2 < chr1)
			return 1;
	}
	if (len1 < len2)
		return -1;
	else if (len2 < len1)
		return 1;
	return 0;
}

int StringHelper::compareString(const char* str1, const __FlashStringHelper* fstr2, int len1, int len2)
{
	auto str2 = (PGM_P)fstr2;
	if (-1 == len1)
		len1 = strlen(str1);
	if (-1 == len2)
		len2 = strlen_P(str2);

	for (int i = 0; i < min(len1, len2); i++)
	{
		auto chr1 = str1[i];
		auto chr2 = pgm_read_byte(&str2[i]);

		if (chr1 < chr2)
			return -1;
		else if (chr2 < chr1)
			return 1;
	}
	if (len1 < len2)
		return -1;
	else if (len2 < len1)
		return 1;
	return 0;
}


/*
int LimitedString_TestDriver::test()
{
	LimitedString<16> s1;
	if (s1.length() != 0)
		return 1;
	s1 = "This is a test";
	if (s1.length() != 14)
	{
		Serial.println(s1.length());
		Serial.println(s1.c_str());
		return 2;
	}
	if (s1.length() != strlen(s1.c_str()))
	{
		Serial.println(s1.length());
		Serial.println(strlen(s1.c_str()));

		Serial.println(s1.c_str() + 2);
		return 3;
	}
	if (s1 != "This is a test")
		return 4;
	if (s1 != F("This is a test"))
		return 5;
	if (s1 != LimitedString<20>("This is a test"))
		return 6;
	if (s1 != LimitedString<15>("This is a test"))
		return 7;
	if (s1 == "This is some other string")
		return 8;
	if (s1 == F("This is some other string"))
		return 9;
	if (!s1.startsWith("This"))
		return 10;
	if (s1.startsWith("That"))
		return 11;
	if (!s1.endsWith("test"))
		return 12;
	if (s1.endsWith("blah"))
		return 13;
	if (s1.left(4) != "This")
		return 14;
	if (s1.right(4) != "test")
		return 15;
	if (2 != s1.indexOf('i'))
		return 16;
	if (-1 != s1.indexOf('x'))
		return 17;
	s1.remove(4, 3);
	if (s1 != "This a test")
	{
		Serial.println(s1.c_str());
		return 18;
	}
	s1.clear();
	if (s1 != "")
		return 19;

	return 0;
}
*/

