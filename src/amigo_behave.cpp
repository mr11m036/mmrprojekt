/*
 * amigo_behave.cpp
 *
 *  Created on: 28.11.2012
 *      Author: amigo
 */

#include "libROBOT.h"


int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();
  ros::Rate pub_rate(20);
  ArSonarDevice sonar;//Creation of a sonar device

  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    printf( "setup failed... \n" );
    return -1;
  }


node->iBeQueueLength = node->getBeQueueLenght();
 //   node->iBeQueueLength = node->getBeQueueLenght();//Length of the behavior queue is determined
	// END OF INIT PHASE

	int* iBeQueID = NULL;//Behavior Queue is created dynamically
	int* iBeQuePr = NULL;//iBeQue[behaviorID][behaviorPriority]
	//double* iBeQueMe = NULL;//Mean
	//double* iBeQueSi = NULL;//Sigma

	iBeQueID = new int[node->iBeQueueLength];
	iBeQuePr = new int[node->iBeQueueLength];
	//iBeQueMe = new double[node->iBeQueueLength];
	//iBeQueSi = new double[node->iBeQueueLength];

	int iBeBuf[2];  //Buffer for the sorting algorithm
	//double dBeBuf[2];  //Buffer for the sorting algorithm

	iBeBuf[0]=0;
	iBeBuf[1]=0;
	//dBeBuf[0]=0.0;
	//dBeBuf[1]=0.0;

	for(int i = 0;i < node->iBeQueueLength;i++)//All fields of the behavior are initialized with 0
	{
		iBeQueID[i]	=	0;
		iBeQuePr[i]	=	0;
		//iBeQueMe[i]	=	0.0;
		//iBeQueSi[i]	= 	0.0;
	}

	//Reading Behavior Queue
	for(int i=0;i<node->iBeQueueLength;i++)
	{
		iBeQueID[i]	=	(int)node->readBehav(i,0);
		iBeQuePr[i]	=	(int)node->readBehav(i,1);
		//iBeQueMe[i]	=	node->readBehav(i,2);
		//iBeQueSi[i]	= 	node->readBehav(i,3);
	}

	//START OF SORTING

	//Sorting behavior queue by priority via a bubblesort algorithm
	bool bSwitched = 1;

	while(bSwitched)
	{
		bSwitched = 0;

		for(int i = 0;i < node->iBeQueueLength;i++)
		{
			if(iBeQuePr[i] < iBeQuePr[i+1])
			{
				iBeBuf[0]=iBeQueID[i];
				iBeBuf[1]=iBeQuePr[i];
			//	dBeBuf[0]=iBeQueMe[i];
			//	dBeBuf[1]=iBeQueSi[i];
				iBeQueID[i]=iBeQueID[i+1];
				iBeQuePr[i]=iBeQuePr[i+1];
			//	iBeQueMe[i]=iBeQueMe[i+1];
			//	iBeQueSi[i]=iBeQueSi[i+1];
				iBeQueID[i+1]=iBeBuf[0];
				iBeQuePr[i+1]=iBeBuf[1];
			//	iBeQueMe[i+1]=dBeBuf[0];
			//	iBeQueSi[i+1]=dBeBuf[1];
				bSwitched = 1;
			}
		}
	}

	//END OF SORTING


	//
	//========================START OF MAIN LOOP============================
	//

	for(;;)
	{
		//node->spin();
		ros::spinOnce();
		pub_rate.sleep();

		node->dVelLeft = 0;//Control variable for the left motor
		node->dVelRight = 0;//Control variable for the right motor
		node->iBeCount = 0;//Counter for the number of behaviors triggered

		/*Execute top behavior
		Check if there are further behaviors with the same priority
		If one of the behaviors was triggered, break
		If none of the behaviors were triggered, try the next behavior in the queue
		*/

		int iBuf = 0;//Buffer for the Priority of the Behavior that is currently being examined


		for(int i = 0;i<node->iBeQueueLength;i++)
		{
			//Buffer for priority of first behavior is set to 0
			iBuf = iBeQuePr[i];//Priority is stored in the buffer
			node->iBeCount += node->beLib(iBeQueID[i]);//Behavior is applied, if the conditions were met,
			//the buffer for the amount of behaviors with similar priority triggered
			//is increased by the return value of the behavior
			if(node->iBeCount)
			{
				for(int i2 = i+1;i2<node->iBeQueueLength;i2++)//Check if other behaviors have the same priority
				{
					if(iBeQuePr[i2]==iBuf)
						node->iBeCount += node->beLib(iBeQueID[i2]);//If applicable, add the return value to the buffer again
				}
				i=node->iBeQueueLength;//Loop is terminated if one or more behaviors were triggered
			}

		}

		//END OF behavior call

		if(node->iBeCount>0)//Median motor values are passed on to the robot
		{
			node->robot->setVel2(node->dVelLeft/node->iBeCount,node->dVelRight/node->iBeCount);
		}
		else//If no behaviors were triggered, the motor values are set to zero
		{
			node->robot->setVel2(0,0);
		}

	}

	//
	//==============================END OF MAIN LOOP================================
	//



  delete node;
  delete[] iBeQueID;
  delete[] iBeQuePr;
  // delete[] iBeQueMe;
  // delete[] iBeQueSi;
	Aria::shutdown();
  printf( "\nQuitting... \n" );
  return 0;

}
