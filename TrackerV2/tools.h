#ifndef TOOLS_H
#define TOOLS_H

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

float normalizeAngle360(float val);
float normalizeAngle(float val);

float rad2deg(float rad);
float deg2rad(float deg);


template<class T> struct remove_reference { typedef T type; };
template<class T> struct remove_reference<T&> { typedef T type; };
template<class T> struct remove_reference<T&&> { typedef T type; };

template <typename T>
typename remove_reference<T>::type&& move(T&& arg)
{
	return static_cast<typename remove_reference<T>::type&&>(arg);
}

template<typename T>
struct identity {
	typedef T type;
};

template <typename T>
T&& forward(typename remove_reference<T>::type& param)
{
	return static_cast<T&&>(param);
}


#endif
