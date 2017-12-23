#ifndef POS_AVG_H
#define POS_AVG_H

/**
 * Our home GS jumps around quite a bit.
 * To compensate for that, we use this averager helper class
 */
template<int Capacity>
class PositionAverager
{
public:
	void put(float lat, float lon);
	bool get(float& lat, float& lon) const;

	void clear();
private:
	float lat[Capacity];
	float lon[Capacity];
	int pos = 0;
	int num = 0;
};

template<int Capacity>
void PositionAverager<Capacity>::put(float lat, float lon)
{
	this->lat[pos] = lat;
	this->lon[pos] = lon;
	if (++this->pos >= Capacity) this->pos = 0;
	if (this->num < Capacity) ++this->num;	
}

template<int Capacity>
bool PositionAverager<Capacity>::get(float& lat, float& lon) const
{
	if (0 == num)
		return false;

	lat = lon = 0.f;
	for (int i = 0; i < this->num; i++)
	{
		lat += this->lat[i];
		lon += this->lon[i];
	}
	lat /= (float)this->num;
	lon /= (float)this->num;

	return true;
}

template<int Capacity>
void PositionAverager<Capacity>::clear()
{
	this->num = this->pos = 0;
}

#endif


