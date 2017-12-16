#ifndef CALLBACK_H
#define CALLBACK_H

#include <Arduino.h>
#include "tools.h"

template<typename T>
class Callback;

template<typename Result, typename... Args>
class Callback<Result(Args...)>
{
public:
	virtual ~Callback() = default;
	virtual Result operator()(Args ... args) = 0;
};

template<typename T, typename CB>
class Callback_Impl;

template<typename Result, typename...Args, typename CB>
class Callback_Impl<Result(Args...), CB> : public Callback<Result(Args...)>
{
public:
	inline Callback_Impl(CB&& cb) : cb(move(cb)) {}
	virtual ~Callback_Impl() = default;

	inline Result operator()(Args ... args) override
	{
		return this->cb(args...);
	}
private:
	CB cb;
};

#endif


