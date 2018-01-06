#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Stream.h>
#include "stream_processor.h"
#include "state_machine.h"

class Bluetooth
{
private:
	enum class State
	{
		Initializing = 0,
		QueryingInformation = 1,
		Connecting = 2,
		FindingPeers = 3,
		Connected = 4,
		Error = 5
	};

public:
	// should display a status on the screen
	typedef void(*displayStatusFn)(const __FlashStringHelper* status);

	// should display an error on the screen
	typedef void(*displayErrorFn)(const __FlashStringHelper* err);

	// connect change notification. 1 = connected, 2 = searching for peer (binding)
	typedef void(*connectChangeFn)(int connected);

	// should display a possible peer on the screen and return true if peer is to be used
	typedef bool(*selectPeerFn)(const char* devname);

	typedef void(*dataReceivedFn)(char chr);

public:
	Bluetooth(Stream& serial,
		uint8_t connectAnalogPin,
		uint8_t bindSwitchPin,
		bool bRedirectToSerial,
		displayStatusFn fnDisplayStatus,
		displayErrorFn fnDisplayError,
		connectChangeFn fnConnectChange,
		selectPeerFn fnSelectPeer,
		dataReceivedFn fnDataReceived);

private:
	void queryInfo();
	void connect();

public:
	void debugPassthrough();

	void onLoop();

	void setPeer(const ConstString& address);

private:
	void onPeersAvailable(const ConstString& listPeers);

private:
	// selects a peer and sets it with SR command
	void selectPeer();

	// during an inquiry, this reads the next peer
	void readNextPeer();

	void writeCmd(const char* cmd, bool bIncludeLF = true, int len = -1);

	template<typename CB>
	inline void writeCmdAndReadReply(const char* cmd, CB&& cb, bool bIncludeLF = true, long msTimeout = 30000)
	{
		this->writeCmd(cmd, bIncludeLF);
		this->m_processor.readLine(forward<CB>(cb), false, msTimeout);
	}
	template<typename CB>
	inline void writeCmdAndReadReply(const ConstString& cmd, CB&& cb, bool bIncludeLF = true, long msTimeout = 30000)
	{
		this->writeCmd(cmd.c_str(), bIncludeLF, cmd.length());
		this->m_processor.readLine(forward<CB>(cb), false, msTimeout);
	}


	inline static bool stringStartsWith(const char* str, const char* startsWith)
	{
		auto strlen1 = strlen(str);
		auto strlen2 = strlen(startsWith);

		if (strlen2 >= strlen1)
			return false;
		for (size_t i = 0; i < strlen2; i++)
			if (str[i] != startsWith[i])
			{
				return false;
			}
		return true;
	}

	void displayStatus(const __FlashStringHelper* status);
	void displayError(const __FlashStringHelper* err);
	void onError(const __FlashStringHelper* err, int recoverTime = 3000);

private:
	Stream& m_sserial;
	const uint8_t m_connectPin;
	const uint8_t m_bindSwitchPin;

	StateMachine<State> m_state = State::Initializing;

	StringStreamProcessor<64, 64> m_processor;
	LimitedString<256> m_strPeerList;

	uint32_t m_tsErrorRecover = 0;

	const bool m_bRedirectToSerial;
	
	const displayStatusFn m_fnDisplayStatus;
	const displayErrorFn m_fnDisplayError;
	const connectChangeFn m_fnConnectChange;
	const selectPeerFn m_fnSelectPeer;
	const dataReceivedFn m_fnDataReceived;
};



#endif


