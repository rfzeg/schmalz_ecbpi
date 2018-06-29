#include "SerialPort.h"

SerialPort::SerialPort(){

}

SerialPort::~SerialPort(){
	//if not closed, close
	if(isatty(this->fd_))
		this->disconnect();
}

void SerialPort::setPortAttributes(const PortAttributes& pa){
	this->pa_ = pa;
	// this->Config();
}

void SerialPort::setPortAttributes(std::string devName, const int& baudRate, const int& parity, const int& byteSize, const int& stopBit, const float& timeOut,const bool& hwCtrl, const bool& swCtrl, const bool& rawInput, const bool& rawOutput){
	this->pa_.device_name	= devName;
	this->pa_.baud_rate 	= baudRate;
	this->pa_.parity 		= parity;
	this->pa_.byte_size 	= byteSize;
	this->pa_.stop_bit 		= stopBit;
	this->pa_.time_out 		= timeOut;
	this->pa_.hardware_ctrl	= hwCtrl;
	this->pa_.software_ctrl	= swCtrl;
	this->pa_.raw_input 	= rawInput;
	this->pa_.raw_output 	= rawOutput;
	// this->Config();
}

PortAttributes SerialPort::getPortAttributes(){
	return this->pa_;
}

int SerialPort::config(){
	struct termios tty;
	if(!isatty(this->fd_)){
		this->connect();
		return 1;
	}

	if(tcgetattr(this->fd_, &tty) < 0){
		std::cout<<"Configuration Failed. Could not get attributes."<<std::endl;
		return -1;
	}

	//Setting the baudrate for output and input
	cfsetospeed(&tty, (speed_t) this->pa_.baud_rate);
	cfsetispeed(&tty, (speed_t) this->pa_.baud_rate);
	
	//Ignore modem control
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= this->pa_.byte_size;

	//Set parity
	switch(this->pa_.parity){						//No parity
		case 0: {
					tty.c_cflag &= ~PARENB; 
					tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP);
					break;		
				}
		case 1: {									//Odd parity
					tty.c_cflag |= PARENB;
					tty.c_cflag |= PARODD;
					tty.c_iflag |= (BRKINT | PARMRK | ISTRIP);
					break;
				}
		case 2:	{									//Even parity
					tty.c_cflag |= PARENB;
					tty.c_cflag &= ~PARODD;
					tty.c_iflag |= (BRKINT | PARMRK | ISTRIP);					
					break;
				}
		default:{	tty.c_cflag &= ~PARENB;	
					tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP);
					break;	
				}
	}

	//Set Stopbits
	if(this->pa_.stop_bit==1)
		tty.c_cflag &= ~CSTOPB;		//1 stop bit
	else
		tty.c_cflag |= CSTOPB;		//2 stop bits

	//Set Byte Size
	tty.c_cflag |= this->pa_.byte_size;

	//Set Timeout for Read 
	tty.c_cc[VMIN] = 0;		//only block for the first byte
	tty.c_cc[VTIME] = this->pa_.time_out;

	//Enable/Disable Hardware Flow Control
	if(this->pa_.hardware_ctrl)
		tty.c_cflag |= CRTSCTS;
	else
		tty.c_cflag &= ~CRTSCTS;

	//Enable/Disable Flow Control
	if(this->pa_.software_ctrl)
		tty.c_cflag |= (IXON | IXOFF | IXANY);
	else
		tty.c_cflag &= ~(IXON | IXOFF | IXANY);

	//Enable/Disable Raw Input
	if(this->pa_.raw_input){
		tty.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);
		tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON);
	}
	else{
		tty.c_lflag |= (ICANON | ECHO | ECHONL | ISIG | IEXTEN);
		tty.c_iflag |= (INLCR | IGNCR | ICRNL | IXON);
	}

	//Enable/Disable Raw Output
	if(this->pa_.raw_output)
		tty.c_oflag &= ~OPOST;
	else
		tty.c_oflag |= OPOST;


	if(tcsetattr(this->fd_, TCSANOW, &tty) != 0){
		std::cout<<"Failed to set port attributes"<<std::endl;
		return -1;
	}
	return 0;
}

int SerialPort::connect(){
	this->fd_ = open(this->pa_.device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if(this->fd_ == -1){
		std::cout<<"Failed to open port "<<this->pa_.device_name<<std::endl;
		return -1;
	}
	fcntl(this->fd_, F_SETFL, 0);	//force non-blocking when read from input buffer

	return this->config();
}

int SerialPort::disconnect(){
	return close(this->fd_);
}

int SerialPort::writeVector(std::vector<unsigned char>& buffer, int len){
	int wlen = write(this->fd_, &buffer[0], len);
	while(tcdrain(this->fd_) !=0){};
	
	if(wlen<0){
		std::cout<<"Write Error"<<std::endl;
		return -1;
	}
	else if(wlen<len){
		std::cout<<"Write Incomplete"<<std::endl;
		return -1;
	}
	return 0;
}

int SerialPort::readVector(std::vector<unsigned char>& buffer){
	buffer.resize(1);

	if(read(this->fd_, &buffer[0],1)>0){
		buffer.resize(buffer[0]);
		read(this->fd_, &buffer[1], buffer[0]-1);
	}
	return 0;
}

int SerialPort::readVector(std::vector<unsigned char>& buffer, int len){
	buffer.resize(len);
	return read(this->fd_, &buffer[0], len);
}


//TODO
int SerialPort::setDeviceName(char* devName){
	this->pa_.device_name = std::string(devName);
	return 0;
}
int SerialPort::setBaudRate(const int& br){
	switch(br){
		case 9600	: this->pa_.baud_rate = B9600;	break;
		case 19200	: this->pa_.baud_rate = B19200;	break;
		case 38400	: this->pa_.baud_rate = B38400;	break;
		case 57600	: this->pa_.baud_rate = B57600;	break;
	//	case 76800	: baud_rate = B76800;	break;
		case 115200	: this->pa_.baud_rate = B115200;	break;
		default		: std::cout<<"Invalid Baudrate"<<std::endl;return -1;
	}							
	return 0;
}

int SerialPort::setParity(const int& p){
	if((p<0) || (p>3))
		return -1;
	this->pa_.parity = p;
	return 0;
}

int SerialPort::setByteSize(const int& b){
	switch(b){
		case 5:	this->pa_.byte_size = CS5;break;
		case 6: this->pa_.byte_size = CS6;break;
		case 7: this->pa_.byte_size = CS7;break;
		case 8: this->pa_.byte_size = CS8;break;
		default: std::cout<<"Invalid Byte Size"<<std::endl;return -1;
	}
	return 0;
}

int SerialPort::setStopBits(const int& s){
	if((s!=1) && (s!=2))
		return -1;
	this->pa_.stop_bit = s;
	return 0;
}

int SerialPort::setTimeOut(const int& t){
	if(t<0)
		return -1;
	
	this->pa_.time_out = t;
	return 0;
}

int SerialPort::enableHardwareFlowControl(bool state){
	this->pa_.hardware_ctrl	= state;
	return 0;
}

int SerialPort::enableSoftwareFlowControl(bool state){
	this->pa_.software_ctrl	= state;
	return 0;
}
int SerialPort::setRawInput(bool state){
	this->pa_.raw_input 	= state;
	return 0;
}

int SerialPort::setRawOutput(bool state){
	this->pa_.raw_output 	= state;
	return 0;
}

int SerialPort::resetInput(){
	return tcflush(this->fd_, TCIFLUSH);
}

int SerialPort::resetOutput(){
	return tcflush(this->fd_, TCOFLUSH);
}

int SerialPort::resetIO(){
	return tcflush(this->fd_, TCIOFLUSH);
}
// //TEST CASE
// int main(int argc, char** argv){
// 	PortAttributes pa;
// 	if(argc>1){
// 		for(int i=1;i<argc;i++){
// 			if(argv[i][0]=='-'){
// 				switch(argv[i][1]){
// 					case 'H':	pa.hardware_ctrl = true;break;
// 					case 'S':	pa.software_ctrl = true;break;
// 					case 'I':	pa.raw_input = false;break;
// 					case 'O':	pa.raw_output = false;break;
// 					case 'D':	{
// 									std::string str(argv[i+1]);
// 									pa.device_name = str;i++;
// 									break;
// 								}
// 					case 'B':	{
// 									int br, baud_rate;
// 									sscanf(argv[i+1], "%d", &br);
// 									i++;
// 									switch(br){
// 										case 9600	: baud_rate = B9600;		break;
// 										case 19200	: baud_rate = B19200;	break;
// 										case 38400	: baud_rate = B38400;	break;
// 										case 57600	: baud_rate = B57600;	break;
// 										// case 76800	: baud_rate = B76800;	break;
// 										case 115200	: baud_rate = B115200;	break;
// 										default		: std::cout<<"Invalid Baudrate"<<std::endl;return -1;
// 									}
// 									pa.baud_rate = baud_rate;
// 									break;
// 								}
// 					case 'P':	{
// 									int p;
// 									sscanf(argv[i+1],"%d",&p);
// 									i++;
// 									if((p<0)||(p>3)){
// 										std::cout<<"Invalid parity Type"<<std::endl;return -1;
// 									}
// 									pa.parity = p;
// 									break;
// 								}
// 					case 'b':	{
// 									int b;
// 									sscanf(argv[i+1],"%d",&b);
// 									i++;
// 									switch(b){
// 										case 5:	pa.byte_size = CS5;break;
// 										case 6: pa.byte_size = CS6;break;
// 										case 7: pa.byte_size = CS7;break;
// 										case 8: pa.byte_size = CS8;break;
// 										default: std::cout<<"Invalid Byte Size"<<std::endl;return -1;
// 									}
// 									break;
// 								}
// 					case 's':	{
// 									int sb;
// 									sscanf(argv[i+1],"%d",&sb);
// 									i++;
// 									if((sb<1)||(sb>2)){
// 										std::cout<<"Invalid Stop Bit"<<std::endl;return -1;
// 									}
// 									pa.stop_bit = sb;
// 									break;
// 								}
// 					case 't':	{
// 									int t;
// 									sscanf(argv[i+1],"%d",&t);
// 									i++;
// 									if(t<0){
// 										std::cout<<"Invalid Timeout"<<std::endl;return -1;
// 									}
// 									pa.time_out = t;
// 									break;
// 								}
// 					default:	std::cout << "Invalid Option"<<std::endl;return -1;break;
// 				}
// 			}
// 			else{
// 				std::cout<<"Invalid arguments"<<std::endl;return -1;
// 			}
// 		}
// 	}
// 	else{
// 		std::cout<<"No attributes"<<std::endl;
// 		return -1;
// 	}

// 	std::vector<unsigned char> activate = {0x24, 0x00, 0x02, 0x0F, 0x00, 0x00, 0x0B, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20};
// 	std::vector<unsigned char> deactivate = {0x24, 0x00, 0x02, 0x0F, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20};
// 	std::vector<unsigned char> start_pump = {0x07, 0x00, 0x02, 0x0B, 0x00, 0x01, 0x00};
// 	std::vector<unsigned char> stop_pump = {0x07, 0x00, 0x02, 0x0B, 0x00, 0x00, 0x00};
// 	std::vector<unsigned char> check = {0x08, 0x00, 0x02, 0x02, 0x00, 0x64, 0x00, 0x00};
// 	// unsigned char activate[36] = {0x24, 0x00, 0x02, 0x0F, 0x00, 0x00, 0x0B, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20};
// 	// unsigned char deactivate[36] = {0x24, 0x00, 0x02, 0x0F, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20};
// 	// unsigned char start_pump[7] = {0x07, 0x00, 0x02, 0x0B, 0x00, 0x01, 0x00};
// 	// unsigned char stop_pump[7] = {0x07, 0x00, 0x02, 0x0B, 0x00, 0x00, 0x00};
// 	// unsigned char buffer[256];
// 	std::vector<unsigned char> buffer;
// 	//test
// 	SerialPort myport;
// 	myport.SetPortAttributes(pa);

// 	while(1){
// 		char cmd;
// 		std::cout<<"Enter Command: ";
// 		std::cin >> cmd;
// 		switch(cmd){
// 			//connect
// 			case 'C':{
// 				myport.Connect();
// 				break;
// 			}
// 			//disconnect
// 			case 'D':{
// 				myport.Disconnect();
// 				break;
// 			}
// 			//activate
// 			case 'a':{
// 				myport.Write(activate,36);
// 				// myport.ResetInput();
// 				myport.Read(buffer);
// 				break;
// 			}

// 			//deactivate
// 			case 'd':{
// 				myport.Write(deactivate,36);
// 				myport.Read(buffer);
// 				break;
// 			}

// 			//start pump
// 			case 's':{
// 				myport.Write(start_pump,7);
// 				myport.Read(buffer);
// 				break;
// 			}
// 			//stop pump
// 			case 'e':{
// 				myport.Write(stop_pump,7);
// 				myport.Read(buffer);
// 				break;
// 			}
// 			case 'c':{
// 				myport.Write(check, 8);
// 				myport.Read(buffer);
// 				break;
// 			}
// 			//quit program
// 			case 'q':
// 				return 0;
// 			default: break;
// 		}	
// 	}
	

// 	return 0;
// }