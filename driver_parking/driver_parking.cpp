/***************************************************************************

	file                 : user3.cpp
	author            : Xuangui Huang
	email              : stslxg@gmail.com
	description    :  user module for CyberParking

 ***************************************************************************/

 /*
	  WARNING !

	  DO NOT MODIFY CODES BELOW!
 */

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);
double constrain(double lowerBoundary, double upperBoundary, double input);
// Module Entry Point
extern "C" int driver_parking(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	printf("OK!\n");
	return 0;
}

/*
	 WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

/*
	define your variables here.
	following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;
bool goahead = false;
bool accreverse = true;
bool initial = true;
double initialx = 0;
double expectedSpeed = 0;
double serr;
double curSpeedErr;
double speedErrSum;
double tmps;
double kp_s = 0.02;//0.02
double ki_s = 0;//0.02
double kd_s = 0.02;//0.02
double dist2 = 1000;
int sgn = 0;
int cnt = 0;
double targetangle = 0;
bool brakeflag = true;
double lastd = 100;
double d = 0;
double yerrsum = 0;
double ydiff = 0;
double derrsum = 0;
double ddiff = 0;
double td = 0;
double a = 0;
static double getAngleErr(double x, double y)
{
	double e1, e2, e3;
	e1 = x - y;
	e2 = e1 + 2 * PI;
	e3 = e1 - 2 * PI;
	if (abs(e1) <= abs(e2) && abs(e1) <= abs(e3))
		return e1;
	else if( abs(e2) <= abs(e3))
		return e2;
	else return e3;
}
static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
	//printf("lotX %.6f  lotY %.6f", _lotX, _lotY);
}

static int flag = 0;
static float k, b, dist;
static int flagt = 0;

static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	if (abs(_lotAngle) > (PI / 2 - 0.05) && abs(_lotAngle) < (PI / 2 + 0.05))                       //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
	{
		dist = abs(_carX - _lotX);
		if (_carX - _lotX > 0) sgn = 1;
		else sgn = -1;
	}

	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);
		dist = abs(k * _carX - _carY + b) / sqrt(k * k + 1);
		if (k * _carX - _carY + b > 0) sgn = 1;
		else sgn = -1;
	}
	if (sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) < 70)
	{
		if ((_caryaw - _lotAngle < 0.01 && _caryaw - _lotAngle >-0.01) || (_caryaw - _lotAngle < 0.01 && _caryaw - _lotAngle -2*PI> -0.01) || (_caryaw +2*PI- _lotAngle < 0.01 && _caryaw - _lotAngle>-0.01))
		{
			dist2 = dist / 0.01;
		}
		else
		{
			dist2 = dist / fabs(sin(_caryaw - _lotAngle));
		}
	}
	if (_lotAngle > 0) targetangle = _lotAngle - PI;
	else targetangle = _lotAngle + PI;

	if (flagt == 1) {
		//*cmdAcc = 1;
		//*cmdBrake = 0;
		//*cmdGear = 1;
		//*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/3.14;
		flag = 6;
	}
	else
	{
	
		if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 2.5) {    //用车速判断是否完成泊车
			yerrsum += _lotAngle - _caryaw;
			double AngleErr = getAngleErr(_lotAngle, _caryaw);
			*cmdSteer = constrain(-1, 1, -20 * AngleErr / 3.14);// -0.2 * yerrsum);
			d = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));

			if (abs(_speed) < 0.2) {
				*bFinished = true;
				flagt = 1;
			}
			else
			{
				*cmdBrake = 0.1;*cmdGear = -1;*cmdAcc = 0;
			}
			if (abs(_speed) < 1.0)
			{
				*cmdBrake = 0;
				*cmdGear = -1;
				*cmdAcc = 0.1;
			}
			if (d > lastd)
			{
				*cmdBrake = 1;
				*cmdAcc = 0;
			}
			lastd = d;
			flag = 1;
		}
		/*else if (sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) < 20 && flagt == 2 && _speed>-10&&accreverse)
		{
			flag = 6;
			*cmdSteer = constrain(-1, 1, -20 * (_lotAngle - _caryaw) / 3.14);
			*cmdGear = -1;
			*cmdAcc = 1;
			*cmdBrake = 0;
		}*/
		else if (flagt == 2)
		{ //接近停车位时，倒车入库
			if (dist < 0.5)
				derrsum += sgn * dist;
			ddiff = sgn * dist - td;
			td = sgn * dist;
			double AngleErr = getAngleErr(_lotAngle, _caryaw);
			*cmdSteer = constrain(-1, 1, -20 * AngleErr / 3.14 - sgn * dist * 1.2);// -derrsum * 0.15 - 10 * ddiff);
			
			//*cmdGear = -1;
			//*cmdAcc = 1;
			//*cmdBrake = 0;

			if (_speed > -10 && accreverse)
			{
				*cmdGear = -1;
				*cmdAcc = 1;
				*cmdBrake = 0;
			}
			else
			{


				accreverse = false;
				if (abs(_speed) > 10) { *cmdBrake = 0.1; *cmdGear = -1; *cmdAcc = 0; }
				else if (abs(_speed) > 9) { *cmdBrake = 0; *cmdGear = -1; *cmdAcc = 0.1; }
				//else if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 0.5) { *bFinished = true; flagt = 1; }
			}
			flag = 2;
		}
		else if (((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 1500 && (dist2 < 24 || flag == 3)) && (flagt != 2)) 
		{//较接近停车位时，给一个大的转向	//1500 26

#pragma region drift

			/*if (dist > 20)
			{
				*cmdSteer = 0.2 * _yaw;
				*cmdAcc = 0;
				*cmdGear = 2;
				*cmdBrake = 0.3;
			}*/

			* cmdSteer = -1.0;
			*cmdAcc = 0;
			*cmdGear = 1;

			
			if (dist2 > 15)
			{
				a = -log10((dist2 - 14) / 10);
				printf("a:%.2f\t", a);
				*cmdBrake = constrain(0, 1, a / 7); //7-8
				brakeflag = false;
			}
			

			/*
			if (dist2 > 15)
			{
				if (brakeflag) 
				{
					a = -log10((dist2 - 14) / 10);
					printf("a:%.2f\t", a);
					*cmdBrake = constrain(0, 1, a / 6);
					brakeflag = false;
				}
				else 
				{
					*cmdBrake = 0;
					printf("a:%.2f\t", -1);
					brakeflag = true;
				}
			}
			*/

			if ((_caryaw - _lotAngle < PI / 9.0 * 3.0 && _caryaw>_lotAngle) || (_caryaw + 2 * PI - _lotAngle < PI / 9.0 * 3.0 && _caryaw < _lotAngle))
				*cmdBrake = 1;

			/*
			if ((_caryaw - _lotAngle < PI / 9.0 * 3.0 && _caryaw>_lotAngle) || (_caryaw + 2 * PI - _lotAngle < PI / 9.0 * 3.0 && _caryaw < _lotAngle))
				brakeflag = true;
			if (brakeflag)
				*cmdBrake = 1;
			*/
			

#pragma endregion


			if (abs(_speed) < 0.1) {
				flagt = 2;
			}

			flag = 3;
		}
		else if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 6050 && dist < 56)
		{ //提前将车控制在道路左侧
			*cmdSteer = (_yaw - atan2(_midline[10][0] - 1.0 * _width / 3.0, _midline[10][1])) / 3.14;
			/*if( _speed > 100 ){*cmdBrake = 0.2;*cmdGear = 2;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 2;*cmdAcc = 0.1;}*/
			expectedSpeed = 100;
#pragma region sPID1

			curSpeedErr = expectedSpeed - _speed;
			serr = curSpeedErr - tmps;// add a defferential term
			speedErrSum = 0.1 * speedErrSum + curSpeedErr; // weaken the influence of integral term
			tmps = curSpeedErr;
			if (curSpeedErr > 0)
			{
				if (abs(_yawrate) > 0.7)
				{
					*cmdAcc = 0.3 - 0.25 * abs(*cmdSteer) * abs(*cmdSteer);
					*cmdBrake = 0;

				}

				else if (abs(*cmdSteer) < 0.1)
				{
					*cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + kd_s * serr); // PID of speed
					*cmdBrake = 0;
				}


				else if (abs(*cmdSteer) > 0.1)
				{
					*cmdAcc = 0.3 - 0.25 * abs(*cmdSteer) * abs(*cmdSteer);
					*cmdBrake = 0;
				}

			}
			else if (curSpeedErr < 0)
			{
				if (abs(*cmdSteer) > 0.1 || abs(_yawrate) > 1.0)
				{
					*cmdAcc = 0;
					*cmdBrake = 0.1;
				}
				else
				{
					*cmdBrake = constrain(0.0, 1.0, -kp_s * curSpeedErr / 3 + 0.12);//4 0.02
					*cmdAcc = 0;
				}

			}
#pragma endregion
			flag = 4;
		}

		else
		{			                                                                         //其它路段按巡线方式行驶
			expectedSpeed = 100;
#pragma region sPID2
			curSpeedErr = expectedSpeed - _speed;
			serr = curSpeedErr - tmps;// add a defferential term
			speedErrSum = 0.1 * speedErrSum + curSpeedErr; // weaken the influence of integral term
			tmps = curSpeedErr;
			if (curSpeedErr > 0)
			{
				if (abs(_yawrate) > 0.7)
				{
					*cmdAcc = 0.3 - 0.25 * abs(*cmdSteer) * abs(*cmdSteer);
					*cmdBrake = 0;

				}

				else if (abs(*cmdSteer) < 0.1)
				{
					*cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + kd_s * serr); // PID of speed
					*cmdBrake = 0;
				}


				else if (abs(*cmdSteer) > 0.1)
				{
					*cmdAcc = 0.3 - 0.25 * abs(*cmdSteer) * abs(*cmdSteer);
					*cmdBrake = 0;
				}

			}
			else if (curSpeedErr < 0)
			{
				if (abs(*cmdSteer) > 0.1 || abs(_yawrate) > 1.0)
				{
					*cmdAcc = 0;
					*cmdBrake = 0.1;
				}
				else
				{
					*cmdBrake = constrain(0.0, 1.0, -kp_s * curSpeedErr / 3 + 0.12);//4 0.02
					*cmdAcc = 0;
				}

			}
#pragma endregion
			//*cmdAcc = 1;//油门给100%
			//*cmdBrake = 0;//无刹车
			* cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
			*cmdGear = 2;//档位始终挂1
			flag = 5;
		}
	}


	if (*bFinished)
	{
		/*if ((sqrt((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) < 0.5 )&&!goahead) { //十分接近停车位时，控制车的朝向与车位一致，直线出库
			*cmdSteer =constrain(-1,1,20*(_lotAngle -_caryaw)/3.14);//要改
			*cmdBrake = 0;
			*cmdGear = 1;
			*cmdAcc = 1;

			flag = 7;
		}
		else
		{*/
		if (sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) < 1)
			*cmdAcc = 1.0;
		else
			*cmdAcc = 0.5;
		*cmdBrake = 0;

		*cmdSteer = constrain(-1, 1, (0.0 * _yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14);
		*cmdGear = 1;
		flag = 9;
		//}
		/*else if (((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 50 && dist <1 )&&!goahead&&(_caryaw-_lotAngle<PI/2.0)){//较接近停车位时，给一个大的转向
			*cmdSteer = 1;
			*cmdAcc = 1;
			*cmdBrake = 0;
			*cmdGear = 1;
			flag = 8;
		}
		else {			                                                                         //其它路段按巡线方式行驶
			if (initial)
			{
				initial = false;
				initialx = _midline[0][0];
			}
			*cmdAcc = 1;
			*cmdBrake = 0;
			*cmdSteer = constrain(-1,1,0.2*_yaw);
			*cmdGear = 1;
			flag = 9;
			goahead = true;
		}*/
	}

	//if (sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) < 100) printf("  d:%f\tfinish:%d\t", sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)), *bFinished);


	printf("acc:%f\tSteer:%.2f\tbrake:%f\t", *cmdAcc, *cmdSteer, *cmdBrake);

	//printf("speed:%.2f\tcaryaw:%.2f\tyaw:%.2f\tdist:%.2f\t", _speed, _caryaw, _yaw, sgn * dist);
	printf("dist:%.2f\tdist2:%.2f\t", sgn * dist, dist2);
	printf("flag:%d\t", flag);

	printf("\n");
	printf("lotX %.6f  lotY %.6f   lotAngle: % .2f\n", _lotX, _lotY, _lotAngle);

	//printf("Steer:%.2f\tflag:%d\tspeed:%.2f\tdist:%.2f\tlotAngle:%.2f\tcaryaw:%.2f\tbrake:%f\n",*cmdSteer,flag,_speed,sgn*dist,_lotAngle,_caryaw,*cmdBrake);
	//printf("acc:%f\tflag:%d\tflagt:%d\taccreverse:%d\ts:%f\tdist2:%f\tsteer:%f\n",*cmdAcc,flag,flagt,accreverse,abs(-1.2),dist2,*cmdSteer);
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
