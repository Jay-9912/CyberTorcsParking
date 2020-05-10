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
int Sign(double x, double y);
double Sub(double x, double y);
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
double targetyaw = 0;
bool brakeflag = true;
bool flag0 = true;
bool reverse = true;
double lastd = 100;
double d = 0;


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

}

static int flag = 0;
static float k, b, dist;
static int flagt = 0;
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	if (abs(_lotAngle) > (PI / 2 - 0.05) && abs(_lotAngle) < (PI / 2 + 0.05))                       //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
	{
		if (_lotAngle<-0.5 * PI + 0.05 && _lotAngle>-0.5 * PI - 0.05)
			k = -1000;
		else
		{
			k = 1000;
		}
		b = (_lotY - k * _lotX);
		dist = abs(_carX - _lotX);
		if (k * _carX - _carY + b > 0) sgn = -1;
		else sgn = 1;
	}
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);
		dist = abs(k * _carX - _carY + b) / sqrt(k * k + 1);
		if (k * _carX - _carY + b > 0) sgn = -1;
		else sgn = 1;
	}
	if (_lotAngle >= 0) targetangle = _lotAngle;
	else targetangle = _lotAngle + PI * 2;
	if (_caryaw >= 0) targetyaw = _caryaw;
	else targetyaw = _caryaw + 2 * PI;
	if (sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) < 70)
	{
		if (flagt == 1) {

			flag = 6;
		}
		else
		{
			if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 2.5) {    //用车速判断是否完成泊车
				*cmdSteer = constrain(-1, 1, -20 * Sub(targetangle , targetyaw) / 3.14);
				d = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));

				if (abs(_speed) < 0.2) {//置位
					*bFinished = true;
					flagt = 1;
				}
				else//逐渐减速
				{
					*cmdBrake = 0.5;*cmdGear = -1;*cmdAcc = 0;
				}
				if (abs(_speed) < 1)//维持此速度刚好能保证一脚踩停
				{
					*cmdBrake = 0;
					*cmdGear = -1;
					*cmdAcc = 0.1;
				}
				if (d > lastd)//踩停
				{
					*cmdBrake = 1;
					*cmdAcc = 0;
				}
				lastd = d;
				flag = 1;
			}
			else if (flagt == 2)
			{ //接近停车位时，倒车入库
//				reverse = true;
				if (targetangle > 1.5 * PI || targetangle < 0.5 * PI)
					*cmdSteer = constrain(-1, 1, -40 * Sub(targetangle , targetyaw) / 3.14 - sgn * dist * 2.8);
				else
				{
					*cmdSteer = constrain(-1, 1, -40 * Sub(targetangle , targetyaw) / 3.14 + sgn * dist * 2.8);
				}

				if (abs(_speed) < 20 && reverse)//起始时加速倒车
				{
					*cmdGear = -1;
					*cmdAcc = 1;
					*cmdBrake = 0;
				}
				else//控制速度为20
				{
					reverse = 0;
					if (abs(_speed)<20) {
						*cmdAcc = 0.1;
						*cmdBrake = 0;
						*cmdGear = -1;
					}
					else {
						*cmdAcc = 0;
						*cmdBrake = 0.1;
						*cmdGear = -1;
					}
					
				}
				flag = 2;
			}
			else if (((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 1500) && dist < 14+0.5*abs(targetangle-targetyaw))//&& (dist2 < 24 || flag == 3)) ) 
			{//较接近停车位时，给一个大的转向	
				if (targetangle - targetyaw > PI)
					targetangle = targetangle - 2 * PI;
				if (targetangle - targetyaw < -PI)
					targetyaw -= 2 * PI;
				if (abs(dist) > 6)//较接近时猛打方向缓步减速
				{
					*cmdAcc = 0;
					*cmdGear = 1;
					*cmdBrake = 0.1;
					*cmdSteer = Sign(targetangle,targetyaw);
				}

				else {//很接近时猛踩刹车并调整方向盘
					*cmdAcc = 0;
					*cmdBrake = 1 - 0.2 * constrain(0, 1, abs(_yawrate));
					*cmdSteer = constrain(-0.15, -0.1, 0.1 * Sub(targetangle , targetyaw));
				}

				if (abs(targetyaw - targetangle) < PI / 4.0)//角度即将纠正时踩满刹车
					*cmdBrake = 1;
				if (abs(_speed) < 0.1) {//切换为倒车状态
					flagt = 2;
				}

				flag = 3;
			}
			else if ((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 8000 && dist < 80)
			{ //提前将车控制在道路左侧
				*cmdSteer = (_yaw - atan2(_midline[10][0] - 1.0 * _width / 3.0, _midline[10][1])) / 3.14;
				expectedSpeed = 80+4*constrain(-10,0,_midline[0][0]);
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
				flag = 4;
			}
		}
	}
	else
	{			                                                                         //其它路段按巡线方式行驶
		expectedSpeed = 80;
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
		*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
		*cmdGear = 2;//档位始终挂2
		flag = 5;
	}



	if (*bFinished)
	{
		if (sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)) < 2)
			*cmdAcc = 1.0;
		else *cmdAcc = constrain(0.4,0.6,0.6-0.15*abs(_yawrate));
		*cmdBrake = 0;
		*cmdSteer = constrain(-1, 1, (0.0 * _yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14);
		*cmdGear = 1;
		flag = 9;

	}


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
int Sign(double x, double y)
{
	if (x - y > PI) return -1;
	if (y - x > PI)return 1;
	if (x - y > 0)return 1;
	if (y - x > 0)return -1;
}

double Sub(double x, double y)
{
	if (x - y > PI) return x - y - 2 * PI;
	if (x - y < -PI)return x - y + 2 * PI;
	return x - y;
}