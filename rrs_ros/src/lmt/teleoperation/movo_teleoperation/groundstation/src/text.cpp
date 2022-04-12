#include <robot_groundstation/text.h>

WriteParameter::WriteParameter()
{

}

void WriteParameter::Write(std::time_t time, double p1, double p2, double p3, double p4, double p5, double p6, double p7, double value, double send_flag, double network_data_rate)
{
	f << time << " " << p1 << " " << p2 << " " << p3 << " " << p4 << " " << p5 << " " << p6 << " " << p7 << " " << value << " " << send_flag << " " << network_data_rate << std::endl;
}
void WriteParameter::OpenFile()
{
	f.open("/home/movo3/file.txt", std::fstream::app);
}
void WriteParameter::CloseFile()
{
	f.close();
}
