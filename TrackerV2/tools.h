#ifndef TOOLS_H
#define TOOLS_H

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

float normalizeAngle360(float val);
float normalizeAngle(float val);

float rad2deg(float rad);
float deg2rad(float deg);

#endif
