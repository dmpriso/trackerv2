#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

template<typename T>
class StateMachine
{
public:
	StateMachine(T initstate)
		: m_state(initstate) {}

	bool changeState(T oldstate, T newstate)
	{
		if (this->m_state == oldstate)
		{
			this->m_state = newstate;
			return true;
		}
		return false;
	}

	T setState(T newstate)
	{
		T tmp = this->m_state;
		this->m_state = newstate;
		return tmp;
	}

	T getState() const { return this->m_state; }


private:
	T m_state;

};

#endif


