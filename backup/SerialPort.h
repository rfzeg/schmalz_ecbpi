#ifndef __SERIALPORT_H_
#define __SERIALPORT_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>
#include <vector>
#include <cstdint>


struct PortAttributes {
	PortAttributes() : 
		device_name("/dev/ttyS0"),
		baud_rate(9600),
		parity(0),
		byte_size(CS8),
		stop_bit(1),
		time_out(1),
		hardware_ctrl(false),
		software_ctrl(false),
		raw_input(true),
		raw_output(true) {}

	// char*		 	DeviceName;
	std::string		device_name;
	int				baud_rate;		//9600 - 19200 - 38400 - 576000 - 76800 - 115200
	int				parity;			//0: None - 1: Odd - 2: Even
	int				byte_size;		//CS5 - CS6 - CS7 - CS8
	int				stop_bit;		//1 - 2
	int				time_out;		//unit: 100ms	- Is ignored if canonical input is used
	bool 			hardware_ctrl;	//Hardware Control
	bool 			software_ctrl;	//Software Control
	bool 			raw_input;		//Raw input if true / Canonical input if false
	bool			raw_output;		//Raw output if true/ PostProcessed if false
};



class SerialPort {
public:
	SerialPort();
	~SerialPort();

	void setPortAttributes(const PortAttributes& pa);
	// void SetPortAttributes(char* devName, const int& baudRate, const int& parity, const int& byteSize, const int& stopBit, const float& timeOut, const bool& hwCtrl, const bool& swCtrl, const bool& rawInput, const bool& rawOutput);
	void setPortAttributes(std::string devName, const int& baudRate, const int& parity, const int& byteSize, const int& stopBit, const float& timeOut, const bool& hwCtrl, const bool& swCtrl, const bool& rawInput, const bool& rawOutput);
	PortAttributes getPortAttributes();

	int connect();
	int disconnect();

	// int WriteByte(const unsigned char& b);
	// unsigned char ReadByte();

	int writeVector(std::vector<unsigned char>& buffer, int len);
	int readVector(std::vector<unsigned char>& buffer);
	int readVector(std::vector<unsigned char>& buffer, int len);

	//TODO
	int setDeviceName(char* devName);
	int setBaudRate(const int& br);
	int setParity(const int& p);
	int setByteSize(const int& b);
	int setStopBits(const int& s);
	int setTimeOut(const int& t);
	int enableHardwareFlowControl(bool state);
	int enableSoftwareFlowControl(bool state);
	int setRawInput(bool state);
	int setRawOutput(bool state);

	int resetInput();
	int resetOutput();
	int resetIO();

private:
	PortAttributes pa_;
	struct termios tty_;
	int fd_;
	int config();
};

#endif 