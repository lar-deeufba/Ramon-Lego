#include "serial_port.hpp"

#define PORT_LEN 24

using namespace std;

SerialPort::SerialPort(){
    port = NULL;
    fd = -11;
    connect(false);
}

SerialPort::~SerialPort(){
    delete port;
}

//sets and gets methods
void SerialPort::setFd(int fd){
    this->fd = fd;
}

int SerialPort::getFd(){
    return fd;
}

void SerialPort::setPort(char *port){
    if(this->isConnected()){
        close(fd);
        connect(false);
    }

    this->port = new char[PORT_LEN];

    strcpy(this->port, port);
}

char* SerialPort::getPort(){
    return port;
}

void SerialPort::connect(bool c){
    connected = c;
}

bool SerialPort::isConnected(){
    return connected;
}

//public functions
bool SerialPort::connectSerial(){
    fd = open(port, O_RDWR | O_NOCTTY);// | O_NDELAY);
 
    if(fd == -1) {
        cout << "Invalid port value. Couldn't open device\n";
        return false;
    }

    cout << "Connected to the device\n";
    connect(true);
    configureSerial();

    return true;
}

bool SerialPort::disconnectSerial(){
    if(fd == -1)
        return false;

    tcsetattr(this->fd, TCSANOW, &tty_options_old);
    close(fd);
    connect(false);

    return true;
}

int SerialPort::sendSerialData(char* data, int nBytes){
	return write(this->fd, data, nBytes);
}

int SerialPort::readSerialData(char* data, int nBytes){
    return read(this->fd, data, nBytes);
}

void SerialPort::updateSerial(){
    if(fd != -1){
        close(fd);
    }
    port = NULL;
    fd = -11;
    connect(false);
}

SerialPort* SerialPort::mInstance = NULL;

SerialPort *SerialPort::getInstance()
{
  if(mInstance == NULL){
    mInstance = new SerialPort();
  }
  return mInstance;
}

void SerialPort::changeBaudrate(int index)
{
  cout << "changeBaudrate(int index) " << index << endl;

  switch (index) {
  case 0:
    break;
  case 1:
    break;
  case 2:
    break;
  case 3:
    break;
  case 4:
    break;
  case 5:
    break;
  case 6:
    break;
  case 7:
    break;
  case 8:
    break;
  default:
    break;
  }
}

void SerialPort::changeParityBits(int index)
{
  cout << "changeParityBits(int index) " << index << endl;
}

void SerialPort::changeDataBits(int index)
{
  cout << "changeDataBits(int index) " << index << endl;
}


void SerialPort::changeStopBit(int index)
{
  cout << "cahngeStopBit(int index) " << index << endl;
}

void SerialPort::configureSerial(){

    //TESTANDO
    //Get the current options for the port...
    tcgetattr(fd, &tty_options_old);

    bzero(&tty_options, sizeof(tty_options));
    tty_options.c_cflag = B57600 | CRTSCTS | CS8 | CLOCAL | CREAD;
    tty_options.c_iflag = IGNPAR;
    tty_options.c_oflag = 0;

    //set input mode (non-canonical, no echo..)
    tty_options.c_lflag = 0;
    tty_options.c_cc[VTIME] = 0;
    tty_options.c_cc[VMIN] = 11;

    //TESTANDO
    
    /* Flush Port, then applies attributes */
    tcflush( fd, TCIFLUSH );
    
    //Set the new options for the port...
    if (tcsetattr(fd, TCSANOW, &tty_options) != 0)
    {
        cout << "Error2 " << errno << " from tcsetattr" << endl;
    }
}


void SerialPort::clearSerialData(int index){
    int qs;
    switch(index){
    case 0:
          qs = TCIFLUSH;
          break;
    case 1:
          qs = TCOFLUSH;
          break;
    case 2:
          qs = TCIOFLUSH;
          break;
    default:
          qs = TCIFLUSH;
          break;
    };

    /* Flush Port, then applies attributes */
    tcflush( fd, qs );
    //std::cout << "index = " << index;
}

double SerialPort::retunABS(double value) {
      if (value >=  0)
	return value;
    
      return (-1)*value;
}
