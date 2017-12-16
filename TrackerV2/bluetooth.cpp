#include "bluetooth.h"
#include <mavlink.h>

Bluetooth::Bluetooth(HardwareSerial& serial,
	uint8_t connectPin,
	uint8_t bindSwitchPin,
	bool bRedirectToSerial,
	displayStatusFn fnDisplayStatus,
	displayErrorFn fnDisplayError,
	connectChangeFn fnConnectChange,
	selectPeerFn fnSelectPeer,
	dataReceivedFn fnDataReceived)
	: m_sserial(serial),
	m_connectPin(connectPin),
	m_bindSwitchPin(bindSwitchPin),
	m_processor(m_sserial),
	m_bRedirectToSerial(bRedirectToSerial),
	m_fnDisplayStatus(fnDisplayStatus),
	m_fnDisplayError(fnDisplayError),
	m_fnConnectChange(fnConnectChange),
	m_fnSelectPeer(fnSelectPeer),
	m_fnDataReceived(fnDataReceived)
{
	serial.begin(9600);
	pinMode(connectPin, INPUT);
	pinMode(bindSwitchPin, INPUT_PULLUP);

	// the software serial never reports availableForWrite()
	this->m_processor.forceWrite(true);
}

void Bluetooth::onLoop()
{
	bool bConnected = digitalRead(this->m_connectPin);

	if (!bConnected)
	{
		this->m_processor.process();

		// try various state transitions
		if (this->m_state.changeState(State::Initializing, State::QueryingInformation))
		{
			Serial.println(F("First try..."));
			this->queryInfo();
		}
		else if (this->m_state.changeState(State::Connected, State::Error))
		{
			this->m_fnConnectChange(0);
			this->onError(F("Connection lost"), 2000);
		}
		else if (millis() > this->m_tsErrorRecover && this->m_state.changeState(State::Error, State::QueryingInformation))
		{
			Serial.println(F("Retry..."));
			this->queryInfo();
		}
	}
	else 
	{
		if (this->m_state.setState(State::Connected) != State::Connected)
		{
			// break pending read ops, if there are any
			this->m_processor.breakPendingRead(0, ConstString(), -1); // the error code will tell the connect function 
				// that connection has been established;

			this->displayStatus(F("Connected"));

			// connect notification
			this->m_fnConnectChange(1);
		}

		// we are connected -> simply process chars
		char chr;
		if (this->m_processor.peekChar(chr))
		{
			this->m_fnDataReceived(chr);

			//if (this->m_bRedirectToSerial)
			//	Serial.write(&chr, 1);
		}
	}
}

void Bluetooth::onError(const __FlashStringHelper* err, int recoverTime)
{
	if (nullptr != err)
		this->displayError(err);

	this->m_fnConnectChange(0);

	this->m_state.setState(State::Error);
	this->m_tsErrorRecover = millis() + (long)recoverTime;
}

void Bluetooth::displayStatus(const __FlashStringHelper* status)
{
	Serial.print(F("STATUS: "));
	Serial.println(status);
	this->m_fnDisplayStatus(status);
}

void Bluetooth::displayError(const __FlashStringHelper* err)
{
	Serial.print(F("ERROR: "));
	Serial.println(err);
	this->m_fnDisplayError(err);
}

void Bluetooth::readNextPeer()
{
	this->m_processor.readLine([this](const ConstString& str, int err)
	{
		if (!err && str.length() > 14)
		{
			// found a peer
			// parse it's name and address
			int id1 = str.indexOf(',');
			int id2 = str.indexOf(',', id1 + 1);
			if (-1 != id2)
			{
				// add to peer list
				auto pstr = str.c_str();
				this->m_strPeerList.append(pstr, id2);
				this->m_strPeerList.append(F("\n"));
			}

			this->readNextPeer();
		}
		else if (this->m_strPeerList.length() > 0)
		{
			this->onPeersAvailable(this->m_strPeerList);
		}
		else
		{
			this->onError(F("Error reading peers"));
		}
	});
}

void Bluetooth::onPeersAvailable(const ConstString& listPeers)
{
	if (!this->m_fnSelectPeer(listPeers.c_str()))
	{
		// retry scan
		this->selectPeer();
	}
}

void Bluetooth::setPeer(const ConstString& address)
{
	LimitedString<24> cmd;
	cmd.append("SR,");
	cmd.append(address.c_str(), address.length());
	this->writeCmdAndReadReply(cmd.c_str(), [this](const ConstString& repl, int err)
	{
		if (err != 0 || repl != "AOK")
		{
			this->onError(F("Could not set remote peer"));
		}
		else
		{
			if (this->m_state.changeState(State::FindingPeers, State::Connecting))
			{
				this->connect();
			}
		}
	});
}

void Bluetooth::selectPeer()
{
	this->m_fnConnectChange(1);

	this->displayStatus(F("Searching for peers..."));

	this->m_strPeerList.clear();

	// start inquiry
	this->writeCmdAndReadReply("I", [this](const ConstString& str, int err)
	{
		if (err || !str.startsWith("Inquiry"))
		{
			this->onError(F("Failed to start inquiry"));
		}
		// wait for a "found" reply
		this->m_processor.readLine([this](const ConstString& str, int err)
		{
			if (str == F("No Devices Found"))
			{
				this->displayStatus(F("No peer"));
				this->selectPeer();
			}
			else if (!str.startsWith(F("Found")))
			{
				this->onError(F("Inquiry failed"));
			}
			else
			{
				this->readNextPeer();
			}
		});
	});
}

void Bluetooth::debugPassthrough()
{
	while (true)
	{
		if (this->m_sserial.available()) {
			Serial.write(this->m_sserial.read());
		}
		if (Serial.available()) {
			this->m_sserial.write(Serial.read());
		}
	}
}

void Bluetooth::connect()
{
	this->displayStatus(F("Connecting..."));
	this->writeCmdAndReadReply("C", [this](const ConstString& repl, int err)
	{
		if (err != 0 || repl != "TRYING")
		{
			this->onError(F("Could not start connect operation"));
		}
		else
		{
			// try to read another line
			this->m_processor.readLine([this](const ConstString& line, int err)
			{
				// error code -1 signals successful connect (see onLoop() function
				if (-1 != err)
				{
					this->onError(F("Connect failed"));
				}
			});
		}
	});
}

void Bluetooth::queryInfo()
{
	bool bReSetPeer = LOW == digitalRead(this->m_bindSwitchPin);

	this->displayStatus(F("Enter CMD mode..."));

	// enter command mode
	this->writeCmdAndReadReply("$$$", [this, bReSetPeer](const ConstString& repl, int err)
	{
		// define continue logic, first
		auto ldContinue = [this, bReSetPeer](const ConstString& repl, int err)
		{
			if (err != 0 || (repl != "CMD" && repl != "?")) // "?" would mean already in cmd mode
			{
				this->onError(F("Could not enter command mode"));
			}
			else
			{
				this->displayStatus(F("Determine peer..."));

				// check if we do have a peer already
				this->writeCmdAndReadReply("GR", [this, bReSetPeer](const ConstString& repl, int err)
				{
					bool bTmp = bReSetPeer;
					if (err != 0 || repl.length() != 12)
					{
						this->displayStatus(F("No peer set"));
						// we don't have a peer so we need to re-set it
						bTmp = true;
					}

					if (bTmp)
					{
						if (this->m_state.changeState(State::QueryingInformation, State::FindingPeers))
						{
							this->selectPeer();
						}
						else
						{
							// unknown error. may be connected already?
							this->onError(F("Could not change state to FindingPeers"));

						}
					}
					else if (this->m_state.changeState(State::QueryingInformation, State::Connecting))
					{
						this->connect();
					}

				});
			}
		};

		if (err == 2)
		{
			// we may already be in cmd mode, so let's send a LF
			this->writeCmdAndReadReply("", move(ldContinue));
		}
		else
			ldContinue(repl, err);
	}, false, 1000);
}

void Bluetooth::writeCmd(const char* cmd, bool bIncludeLF, int len)
{
	if (-1 == len) len = strlen(cmd);

	this->m_processor.write_forget(cmd, len);

	if (this->m_bRedirectToSerial)
		Serial.write(cmd);
	if (bIncludeLF)
	{
		this->m_processor.write_forget("\n", 1);

		if (this->m_bRedirectToSerial)
			Serial.write("\n");
	}
}

