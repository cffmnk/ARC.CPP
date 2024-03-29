#include "lidar.h"

Lidar::Lidar()
	: shutting_down_(false)
{	
	uart_.name = LDS_PORT;
	uart_.defaultRM = 0;
	uart_.session = 0;
	
	status_ = Uart_Open(&uart_,
		BaudRate,
		DataBit,
		Uart_StopBits1_0,
		Uart_ParityNone);
	uint8_t writeData = 'b';
	status_ = Uart_Write(&uart_, &writeData, 1);
	ranges = std::vector<float>(360);
	points = std::vector<std::pair<float, float>>(360);
}

Lidar::~Lidar()
{
	uint8_t writeData = 'e';
	status_ = Uart_Write(&uart_, &writeData, 1);
	status_ = Uart_Clear(&uart_);
	status_ = Uart_Close(&uart_);
}

void Lidar::clear()
{
	Uart_Clear(&uart_);
}

void Lidar::poll()
{
	uint8_t temp_char;
	uint8_t start_count = 0;
	bool got_scan = false;
	std::vector<uint8_t> raw_bytes(2520);
	uint8_t good_sets = 0;
	uint32_t motor_speed = 0;
	rpms = 0;
	int index;
	auto t = std::chrono::high_resolution_clock::now();
	//freopen("scan.txt", "w", stdout);
	while (!shutting_down_ && !got_scan)
	{
		if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t).count() > 3000)
		{
			uint8_t writeData = 'b';
			Uart_Write(&uart_, &writeData, 1);
			t = std::chrono::high_resolution_clock::now();
		}
		// Wait until first data sync of frame: 0xFA, 0xA0
		status_ = Uart_Read(&uart_, &raw_bytes[start_count], 1);

		if (start_count == 0)
		{
			if (raw_bytes[start_count] == 0xFA)
			{
				start_count = 1;
			}
		}
		else if (start_count == 1)
		{
			if (raw_bytes[start_count] == 0xA0)
			{
				start_count = 0;

				// Now that entire start sequence has been found, read in the rest of the message
				got_scan = true;

				for (int i = 0; i < 2518; ++i)
				{
					status_ = Uart_Read(&uart_, &raw_bytes[2 + i], 1);
				}
				// scan->angle_min = 0.0;
				// scan->angle_max = 2.0*M_PI;
				// scan->angle_increment = (2.0*M_PI/360.0);
				// scan->range_min = 0.12;
				// scan->range_max = 3.5;
				// scan->ranges.resize(360);
				// scan->intensities.resize(360);

				//read data in sets of 6
				for(uint16_t i = 0 ; i < raw_bytes.size() ; i = i + 42)
				{
					if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42)) //&& CRC check
						{
							good_sets++;
							motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];   //accumulate count for avg. time increment
							rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10;

							for (uint16_t j = i + 4; j < i + 40; j = j + 6)
							{
								index = 6*(i / 42) + (j - 4 - i) / 6;

								// Four bytes per reading
								uint8_t byte0 = raw_bytes[j];
								uint8_t byte1 = raw_bytes[j + 1];
								uint8_t byte2 = raw_bytes[j + 2];
								uint8_t byte3 = raw_bytes[j + 3];

								// Remaining bits are the range in mm
								uint16_t intensity = (byte1 << 8) + byte0;

								// Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
								// uint16_t intensity = (byte3 << 8) + byte2;
								uint16_t range = (byte3 << 8) + byte2;

								// scan->ranges[359-index] = range / 1000.0;
								// scan->intensities[359-index] = intensity;
								if(range < 140)
									range = 0;
								//printf("%f\n", range / 10.0);
								float r = range / 10.;
								ranges[index] = r;
								double ang = index * M_PI / 180.;
								points[index].first = r * cos(ang);
								points[index].second = r * sin(ang);
							}
						}
				}

				// scan->time_increment = motor_speed/good_sets/1e8;
			}
			else
			{
				start_count = 0;
			}
		}
	}
	//fclose(stdout);
}

int Lidar::medianInRange(int left, int right)
{
//	for (int i = left; i <= right; ++i)
//		std::cout << ranges[i] << " ";
//	std::cout << "\n";
		
	std::vector<double> dsa = {200};
	for (int i = left; i != right + 1; i = (i + 1) % 360)
		if (ranges[i] != 0)
			dsa.push_back(ranges[i]);
		
	std::sort(dsa.begin(), dsa.end());
	if (dsa.size() == 1)
		return 0;
	return dsa[((int)dsa.size() - 1) / 2];
}



