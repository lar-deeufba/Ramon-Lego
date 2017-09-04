#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <cstring>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

class SerialPort {

public:
	//SerialPort();
	//~SerialPort();
	int getFd();
	void setPort(char *port);
	char* getPort();
	bool isConnected();

	bool connectSerial();
	bool disconnectSerial();
	void configureSerial();
	void clearSerialData(int index=0);
	void updateSerial();

    int sendSerialData(char* data, int nBytes);
    int readSerialData(char* data, int nBytes);

	static SerialPort* getInstance();

	void changeBaudrate(int index);
	void changeParityBits(int index);
	void changeDataBits(int index);
	void changeStopBit(int index);
	
	double retunABS(double value);

private:
	char *port;
	int fd;
	struct termios tty_options_old, tty_options;
	bool connected;

	static SerialPort *mInstance;

	SerialPort();
	~SerialPort();
	void connect(bool c);
	void setFd(int fd);


};
#endif // SERIAL_PORT_HPP
