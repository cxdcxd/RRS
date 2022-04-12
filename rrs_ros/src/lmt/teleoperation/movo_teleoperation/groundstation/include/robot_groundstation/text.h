#include <iostream>
#include <fstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>  

class WriteParameter
{

public:

	WriteParameter();
	void Write(std::time_t time, double p1, double p2, double p3, double p4, double p5, double p6, double p7, double value, double send_flag, double network_data_rate);

	void OpenFile();
	void CloseFile();

	std::fstream f;



private:


};

