#ifndef _SERIALCOMM
#define _SERIALCOMM

#include "serialport.h"

constexpr auto RETURN_SUCCESS = 1;
constexpr auto RETURN_FAIL = 0;


class CSerialComm
{
public:
	CSerialComm();
	~CSerialComm();

	CSerialPort	serial;
	int		connect(char* _portNum);
	int		sendCommand(char pos);
	void		disconnect();
};

#endif