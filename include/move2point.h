/*
 * Class move2point:
 * Input: start point and goal point
 * output: current pose of a velocity trapezoidal trajectory
 *
 * 	Author: js
 * 	08/29/2014
 */


#ifndef MOVE2POINT_H
#define MOVE2POINT_H

#include <iostream>
#include <vector>
#include <utility>
#include <unistd.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <gsl/gsl_fit.h>

#include <barrett/math.h>
#include <barrett/units.h>
//#include <barrett/systems.h>
//#include <barrett/products/product_manager.h>


using namespace barrett;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

class move2point
{
public:
	move2point(pose_type init, double period, double vel_limit = 3, double omega = 3, double acc = 6.,
			double r_ws = 0.95, double r_b = 0.01, double d_close = 0.1) :
		_handPrev(init.get<0>()),
		_ret(init), _init(init),
		_t(period),
		_velLimit(vel_limit), _omega(omega),
		_accLimit(acc),
		_r_ws(r_ws),
		_r_b(r_b), _d_close(d_close),
		_vel(0), _velPrev(0),
		_MaxVelbeforeClose(0), _MaxVelbeforeClose_buff(0),
		_i(0), _div(1), _iBB(0),
		_Ifreach(false), _bounceback(false), _Ifclose(true)
	{}

	~move2point(){}

	void updateHandPose(pose_type handpose){
		_handPosFeedback = handpose.get<0>();
		_handOriFeedback = handpose.get<1>();
	}

	void updateHandPrev(pose_type handPrev){
		_handPrev = handPrev.get<0>();
		_oriPrev = handPrev.get<1>();
	}

	void updateHandVel_linear(double handvel){
		_velPrev = handvel;
		_vel = handvel;
	}

	void updateTarget(pose_type target){
		_targetCenter = target.get<0>();
		_targetOri = target.get<1>();
	}

	void updateMaxVel(double vel){
		_MaxVelbeforeClose_buff = vel;
		_MaxVelbeforeClose = vel;
	}

	void reset(){
		_Ifreach = false;
		_Ifclose = true;
		_vel = 0; _velPrev = 0;
		_MaxVelbeforeClose_buff = 0;
	}

	double module(cp_type vec) {
		return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
	}

	bool Ifarrive() {
		return _Ifreach;
	}

	double GetMaxAcc(){
		return _accLimit;
	}

	void ChangeMaxAcc(double input){
		_accLimit = input;
	}

	void ChangeVelLimit(double input){
		_velLimit = input;
	}

	pose_type getCurrentPose(){

/***************************************************************************************/
		/*_targetCenter = _objPosBuff;

		Eigen::AngleAxisd aa = _objOri;
		aa= aa.inverse();//This inverse is because the libbarrett is somehow calculate the Tool Ori wrong
		_objOri = aa;

		_targetOri = _objOri;*/
		/********************************************************/
			//Manually set the target point
			/*_targetCenter[0] = -0.3; _targetCenter[1] = 0.0; _targetCenter[2] = 0.1;
			_targetCenter = _init.get<0>();
			Eigen::Vector3d rotationAxis;
			rotationAxis[0] = 1; rotationAxis[1] = 0; rotationAxis[2] = 0;
			Eigen::AngleAxisd targetOri(-M_PI/6*0, rotationAxis);
	//		_objOri.w() = 0.87758; _objOri.x() = 0; _objOri.y() = 0.47943; _objOri.z() = 0;
			_targetOri = targetOri;*/

	//		if (_i == 1)
	//			printf("\ntarget ori: w:%f, x:%f, y:%f, z:%f.  here:%f\n", _objOri.w(),_objOri.x(),_objOri.y(),_objOri.z(), 60/180);
		/********************************************************/


		_deltaOri = _oriPrev.inverse()*_targetOri; //the rotation which need to be done
/***************************************************************************************/

		_d = module(_targetCenter - _handPrev);
//		_d = module(_targetCenter - _handPosFeedback);
		_rHand = module(_handPrev);
		//_rObj = module(_objPosBuff);
		_d_edge = _r_ws - _rHand;

		double acc = _accLimit;
		double d_close; // This the close distance calculate by the current reference velocity

		d_close = pow(_velPrev, 2)/(2*_accLimit)+0.01;

//		static bool ifclose = true;
//		if (_d >= _d_close) ifclose = false;
		if (_d >= d_close) _Ifclose = false;

		//static double MaxVelbeforeClose = 0; //Maximum Velocity before Close to target
		//static double MaxVelbeforeClose_buff = 0;
		if ( !_Ifclose && (_velPrev > _MaxVelbeforeClose_buff))
			_MaxVelbeforeClose_buff = _velPrev;

		// when hand is within the work-space and out of the dead zone of the target
		if ( ( _rHand <= _r_ws) && ( _d > _r_b) ) {

			_Ifreach = false;

			if ( _d < d_close) {// When arm is close to object
				if (!_Ifclose){
					_MaxVelbeforeClose = _MaxVelbeforeClose_buff;
					_MaxVelbeforeClose_buff = 0;
//					printf("\n\nMax velocity before close to the target: %3.2f m/s; Cycles: %d\n", MaxVelbeforeClose, _i);
				}
				_Ifclose = true;

				if (_velPrev > 0.00 ){
					acc = -_MaxVelbeforeClose*_MaxVelbeforeClose/(2*d_close);
//					acc = -_velPrev*_velPrev/(_d_close); //This is the acc can make the arm stop at the target point
				}
				else{
					acc = 0;
				}
			}

			if (_d_edge < 1*_d_close){ // When arm is close to the edge of the work-space
				if (_velPrev > 0.3)
					acc = -(_MaxVelbeforeClose_buff*_MaxVelbeforeClose_buff/(_d_close));
			}

			_vel = _velPrev + acc*_t;

			if (_vel >= _velLimit) _vel = _velLimit;
			if (_vel <= 0) _vel = 0.00; // _velLimit/10;

			_retPos = _handPrev + (_targetCenter - _handPrev)/_d*(_velPrev*_t + 0.5*acc*_t*_t);
//			_retPos = _handPrev + (_targetCenter - _handPosFeedback)/_d*(_vel*_t);
		}
		else {
//			_retPos = _targetCenter;
			_vel = 0;
//			if (fabs(_vel) < 1) {
				_Ifreach = true;

				_MaxVelbeforeClose = 0;
				_MaxVelbeforeClose_buff = 0;
//			}

//			if (_rHand > _r_ws) _bounceback = true;
		}

		/*if (_bounceback){
			 _retPos = _handPrev - _handPrev/_rHand*(_t*0.5);
			 _iBB = _iBB+1;
			 if (_iBB >= 20){
				 _bounceback = false;
				 _iBB = 0;
			 }
		}*/

		_dLead = module(_handPrev-_handPosFeedback);
		cp_type d, lead;
		d = _targetCenter - _handPrev;
		lead = _handPrev - _handPosFeedback;
		double angle_cos = (d.dot(lead)) / (module(d)*module(lead)); // Angle's cosine between vector d and lead
		if ( _dLead >= 0.02 && angle_cos > -0.0){ //When reference goes too faster than the feedback, let the reference wait a little bit
			_retPos = _handPrev;
			_vel = _velPrev;
//			printf("\nWAM fall behind to much! distance: %f, iteration: %d.\n", _dLead, _i);
		}

/******************************************************************************/
		Eigen::AngleAxisd deltaAA = _deltaOri;
		if ( deltaAA.angle()>_omega*_t )
			deltaAA.angle() = _omega*_t; // Angular velocity

		_deltaOri = deltaAA;
		_retOri = _oriPrev*_deltaOri;
//		printf("\n_vel = %f", _vel);
/******************************************************************************/

//		std::cout<<_objPosBuff<<_handPrev<< _retPos <<"\n";
//		_retPos[0] = -0.5; _retPos[1] = 0;_retPos[2] = 0.4;

		if (_Ifreach){
			_ret.get<0>() = _targetCenter;
//			_ret.get<1>() = _targetOri;
			_ret.get<1>() = _retOri;
		}
		else {
			_ret.get<0>() = _retPos;
			_ret.get<1>() = _retOri;
		}

		/*if (_i % 500 == 0){
			std::cout<< "\n(m2p::)The target point is: " <<_targetCenter<< ", orientation: " <<
					_targetOri.w()<<" i"<<_targetOri.x()<<" j"<<_targetOri.y()<<" k"<<_targetOri.z()
						<<"."<<std::endl;

			std::cout<< "\n(m2p::)The output is: " <<_ret.get<0>()<< ", orientation: " <<
									_ret.get<1>().w()<<" i"<<_ret.get<1>().x()<<" j"<<_ret.get<1>().y()<<" k"<<_ret.get<1>().z()
									<<"."<<std::endl;
		}*/

//		_ret.get<1>().w() = 1;_ret.get<1>().x() = 0;_ret.get<1>().y() = 0;_ret.get<1>().z() = 0;
		//this->outputValue->setData(&_ret);


		_handPrev = _retPos;
		_oriPrev = _retOri;
		//_lastObj = _objPosBuff;
		_velPrev = _vel;
		_i++;

		//static bool ifarrive = false;
		/*if (_d<_r_b && !_Ifreach) {
//			printf("\nGoal reached!! final err: %f. Cycles used: %d.\n", _d, _i);
			_Ifreach = true;
			fflush(stdout);
			//return;
		}*/


//		if ((_i+99) % 50 == 0 && abs(acc-20)>0.1){
//			printf("velocity: %f. acc: %f.  err: %f. Cycles used: %d.\n",  _vel,acc, _d, _i);
//		}
//		if ((_i+99) % 100 == 0){
//			printf("velocity: %f. acc: %f.  err: %f. Cycles used: %d.\n",  _vel,acc, _d, _i);
//		}
		return _ret;

	}

private:
	cp_type _handPosFeedback, _targetCenter, _handPrev, _retPos;
	pose_type _ret, _init;
	Eigen::Quaterniond _handOriFeedback, _targetOri, _oriPrev, _deltaOri, _retOri;
	double _t; // period
	double _velLimit; //Max velocity for the trajectory
	double _omega; //angular velocity of the hand
	double _accLimit; //acceleration limit
	double _r_ws, _r_b, _d_close; // Radius of the work space & radius of the ball centered at the object center which the arm cannot go into
	double _d, _d_edge, _dLead, _rHand, _rLastHand;
	double _vel, _velPrev;
	double _MaxVelbeforeClose, _MaxVelbeforeClose_buff;
	int _i, _div, _iBB;
	bool _Ifreach, _bounceback, _Ifclose;
};


#endif
