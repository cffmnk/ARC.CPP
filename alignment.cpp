#include "alignment.h"

void alignment(MyRio_I2c* i2c, Lidar & l1, MotorController & mc1, MotorController & mc2)
{
	double dy = 20;
    while (std::abs(dy) > 5)
    {
        l1.poll();
	    //for (int i = 0; i < 360; ++i)
		  //  std::cout << " " << l1.ranges[i] << " ";
		//std::cout << std::endl;
        std::vector<int> asd;
        for (int i = 0; i < 3; ++i)
        {
            asd.push_back(l1.ranges[i]);
            asd.push_back(l1.ranges[359 - i]);
	        //std::cout << " " << l1.ranges[i] << " " <<  l1.ranges[359 - i];
        }
	    std::cout << std::endl;
	    std::sort(asd.begin(), asd.end());
        
	    dy = std::max(std::min((asd[3] - 230) * 1.0, (double)40), (double)-40);
		//dy = (asd[3] - 230) * 2;//0.95;
	    
	//    std::cout << "DY " << asd[3] << " | " << dy << "\n";
	    move(i2c, mc1, mc2, 0, dy, 0, false);
    }
    mc1.setMotorsSpeed(0, 0);
    mc2.setMotorsSpeed(0, 0);
	///*
    
    ///*
    int mid = 24;
    int dx = mid;
    while (std::abs(dx) > 5)
    {
        l1.poll();
        int al = mid;
        for (int i = 10; i < 40; ++i)
        {
            if (l1.ranges[i] > 400 || l1.ranges[i] == 0)
            {
                al = i;
                break;
            }       
        }
       // std::cout <<"al " << al << "\n";

        dx = (mid - al) * 3;
        move(i2c, mc1, mc2, dx, 0, 0, false);
	    
	   // Position pos(0, 0, 0);
       // pos = moveShift(pos, i2c, mc1, mc2, dx, 0, 15, 1);
    }
    mc1.setMotorsSpeed(0, 0);
    mc2.setMotorsSpeed(0, 0);
    
    //*/
	
	///*
	double t = 1;
	while (std::fabs(t) > 0.003)
	{
		l1.poll();
		int i1 = 1;
		int i2 = 358;
		//for (int i = 0; i < 360; ++i)
		//	std::cout << "I " << i << ": " << l1.ranges[i] << "\n"; 
    
		while ((l1.ranges[i1 + 1] != 0) && (l1.ranges[i1 + 1] < 700) && i1 < 90)
			++i1;
    
		while ((l1.ranges[i2 - 1] != 0) && (l1.ranges[i2 - 1] < 700) && i2 > 2)
			--i2;
        
		--i1;
		++i2;
    
		//std::cout << i1 << " " << i2 << "\n";
		//std::cout << l1.ranges[i1] << " " << l1.ranges[i2] << "\n";
    
		double a2_x = l1.ranges[i2] * cos(i2 * M_PI / 180);
		double a2_y = l1.ranges[i2] * sin(i2 * M_PI / 180);

		double a1_x = l1.ranges[i1] * cos(i1 * M_PI / 180);
		double a1_y = l1.ranges[i1] * sin(i1 * M_PI / 180);
    
		double A_X = (a2_x - a1_x);
		double A_Y = (a2_y - a1_y);
		//std::cout << A_X << " " << A_Y << std::endl;
        
		if((A_X) * (A_X) + (A_Y) * (A_Y) == 0 || std::fabs(((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) > 1))
		    t = 0;
		else
		    t = std::acos((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) - M_PI / 2 - 0.07;
		t = std::max(std::min(t, 0.7), -0.7);
        
		std::cout << "angle " << t * 57.3 << std::endl << std::endl;
		move(i2c, mc1, mc2, 0, 0, -1 * t, false);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	//*/
	
	///*
	dy = 20;
	while (std::abs(dy) > 5)
	{
		l1.poll();
		std::vector<int> asd;
		//for (int i = 0; i < 360; ++i)
		  //  std::cout << " " << l1.ranges[i] << " ";
	    //std::cout << std::endl;
        for(int i = 0 ; i < 3 ; ++i)
		{
			asd.push_back(l1.ranges[i]);
			asd.push_back(l1.ranges[359 - i]);
			//std::cout << " " << l1.ranges[i] << " " <<  l1.ranges[359 - i];
		}
	//	std::cout << std::endl;
		std::sort(asd.begin(), asd.end());
        
		dy = std::max(std::min((asd[3] - 230) * 1.0, (double)40), (double) - 40);
		//dy = (asd[3] - 230) * 2;//0.95;
	    
	   // std::cout << "DY " << asd[3] << " | " << dy << "\n";
		move(i2c, mc1, mc2, 0, dy, 0, false);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	//*/
    
    ///*
    mid = 24;
	dx = mid;
	while (std::abs(dx) > 5)
	{
		l1.poll();
		int al = mid;
		for (int i = 10; i < 40; ++i)
		{
			if (l1.ranges[i] > 400 || l1.ranges[i] == 0)
			{
				al = i;
				break;
			}       
		}
		//std::cout << "al " << al << "\n";

		dx = (mid - al) * 3;
		move(i2c, mc1, mc2, dx, 0, 0, false);
	    
		// Position pos(0, 0, 0);
		// pos = moveShift(pos, i2c, mc1, mc2, dx, 0, 15, 1);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
    //*/
	
}