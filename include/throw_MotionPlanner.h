/*
 * Throw motion planner is the high-level planner for the throwing motion.
 *
 * Phases:
 * 		Moving to starting point --> throw
 * 
 * 
 * 	Author: js
 * 	08/29/2014
 */


#ifndef THROW_MOTIONPLANNER_H
#define THROW_MOTIONPLANNER_H

#include <iostream>
#include <vector>
#include <utility>
#include <unistd.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <boost/circular_buffer.hpp>

#include "move2point.h"

using namespace barrett;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

enum throw_phases {
	WAITING_throw = 0,
	MOVE2START = 1,
	BALANCING = 2,
	THROW = 3,
	STOPPING_throw = 4
};

class throw_MotionPlanner :	 public systems::System,
                                 public systems::SingleOutput<pose_type>
{
public:
	throw_MotionPlanner(double period, pose_type initpose, pose_type startpose,
						  const std::string& sysName = "softcatch_MotionPlanner") :
		systems::System(sysName),
		systems::SingleOutput<pose_type>(this),
		input_objpose(this),
		input_objvel(this),
		input_hand(this),
		input_handVel(this),
		_m2p(initpose, period),
		_period(period),
		_current_phase(WAITING_throw), _prev_phase(WAITING_throw),
//		_current_phase(STOPPING), _prev_phase(STOPPING),
		_vel(Eigen::Vector3d::Zero()), _velPrev(Eigen::Vector3d::Zero()),
		_initpose(initpose), _startpose(startpose), _ret(_initpose), _retPrev(_ret), _objpose(_ret),
		_targetPose(_ret),
		_Ifm2pStart(false), _Ifclose(false),
		_IfBalancing(true),
		_k_p(1.8), _k_d(1), _sat(0.6),
		_i(0)
	{

	}

	~throw_MotionPlanner(){ mandatoryCleanUp(); }

	double getCurrentTime() {
		struct timespec ts;
		double t = 0.0;

		if (!clock_gettime( CLOCK_REALTIME, &ts) )
			t = (double)ts.tv_sec + ts.tv_nsec*1.0e-9;
		else std::cout<< "\nError!...(Getting current time.)\n ::parabolaEstimator::getCurrentTime()\n" << std::endl;

		return t;
	}

	void GotoStart(){
		_prev_phase = _current_phase;
		_current_phase = MOVE2START;
	}

	void StartThrow(){
		_prev_phase = _current_phase;
		_current_phase = THROW;
	}

	Input<pose_type> input_objpose;
	Input<pose_type> input_objvel;
	Input<pose_type> input_hand;
	Input<cv_type> input_handVel;


private:
	move2point _m2p;

	double _period;

	throw_phases _current_phase, _prev_phase;
	cp_type _vel, _velPrev, _acc, _interVel;
	pose_type _initpose, _startpose,  _ret, _retPrev, _objpose, _objvel,_handpose;

	pose_type _targetPose; // target for move2point

	cv_type _handVel; //WAM end-point linear velocity

	Eigen::AngleAxisd _aaBuff;

	bool _Ifm2pStart, _Ifclose;
	bool _IfBalancing;

	//parameters for balancing PD controller
	cp_type _cp_err;
	cp_type _vel_err;
	double _k_p;//1.8;//1.6;
	double _k_d;//1.8;
	double _sat;
	double _angleX, _angleY;

	int _i;

protected:

	virtual void operate(){
		if (input_objpose.valueDefined()){
			_objpose = input_objpose.getValue();
		}
		else
			printf("Undefined object pose!! ::softcatch_Motionplanner::operate()");

		if (input_objvel.valueDefined()) {
			_objvel = input_objvel.getValue();
		}
		else
			printf("Undefined object velocity!! ::softcatch_Motionplanner::operate()");

		if (input_hand.valueDefined()) {
			_handpose = input_hand.getValue();
		}
		else
			printf("Undefined hand pose!! ::softcatch_Motionplanner::operate()");

		if (input_handVel.valueDefined()){
			_handVel = input_handVel.getValue();
		}
		else
			printf("Undefined hand velocity!! ::softcatch_Motionplanner::operate()");


		switch(_current_phase){
		/****************************************************
		 *
		 * **************************************************/
		case WAITING_throw:
			_ret = _retPrev;
			break;
		/****************************************************
		 *
		 * **************************************************/
		case MOVE2START:

			static bool enterflagM2S = false;
			_sat = 0.4;

			if (!enterflagM2S){
				_m2p.updateHandPrev(_handpose);
				_m2p.ChangeMaxAcc(0.2);
				_m2p.ChangeVelLimit(0.5);

				enterflagM2S = true;
			}

			_vel_err = Eigen::Vector3d::Zero() - _objvel.get<0>();
			_angleX = -_k_d*_vel_err[1];
			_angleY = _k_d*_vel_err[0];

			if (_angleX > _sat) _angleX = _sat; if (_angleX < -_sat) _angleX = -_sat;
			if (_angleY > _sat) _angleY = _sat; if (_angleY < -_sat) _angleY = -_sat;

			_targetPose.get<1>() = Eigen::Quaterniond(Eigen::AngleAxisd(_angleX, Eigen::Vector3d::UnitX()))*
								Eigen::Quaterniond(Eigen::AngleAxisd(_angleY, Eigen::Vector3d::UnitY()));

			_targetPose.get<0>() = _startpose.get<0>();

			_m2p.updateHandPose(_handpose);
			_m2p.updateTarget(_targetPose);

			_ret = _m2p.getCurrentPose();

			if (_m2p.Ifarrive()){
				_current_phase = BALANCING;
				_prev_phase = MOVE2START;
				std::cout<<"\nThrowing start point achieved! "
						  "\nMOVE2START-->BALANCING!\n"
						  "[Enter] to start throw.\n";
				fflush(stdout);
			}

			break;
		/*************************************************
		 *
		 * ***********************************************/
		case BALANCING:

			static bool enterflagSTOP = false;
			static double k_p = _k_p;//1.8;//1.6;
			static double k_d = _k_d;//1.8;
//			static double sat = 0.6;
//			static double angleX; static double angleY;

			static int i_prev = 0;

			_sat = 0.6;

			if (!enterflagSTOP || (_i-i_prev)==800){

				i_prev = _i;
				if (_IfBalancing){
					Eigen::AngleAxisd aa = _handpose.get<1>();
					cp_type axis = aa.axis();

					/*std::cout<<"\nIn STOPPING! And Balancing! \nReference tool pose: "<< _targetPose.get<0>()
//							<< "w:"<<_targetPose.get<1>().w()<<" , i:"<<_targetPose.get<1>().x()<<" j:"<<_targetPose.get<1>().y()<<" k:"<<_targetPose.get<1>().z()
							<<".\n"
							<< "Object location: "<< _objpose.get<0>()<<".\n"
							<< "cp_err: " << _cp_err <<".\n"
							<< "Object velocity: "<< _objvel.get<0>()<<"\n"
//							<< "integral_err: "<< cp_err_integral
							<< "angleX = "<<_angleX<<", angleY = "<<_angleY<<".\n"
							<< "HandOri: angle = " << aa.angle() <<", axis = "<< axis
							<< "\n m2pIfreach: "<<_m2p.Ifarrive()<<". \n";
					fflush(stdout);*/
				}
				else{
					std::cout<<"\nSTOPPING! \nReference tool pose: "<< _targetPose.get<0>()
							<< "w:"<<_targetPose.get<1>().w()<<" , i:"<<_targetPose.get<1>().x()<<" j:"<<_targetPose.get<1>().y()<<" k:"<<_targetPose.get<1>().z()
							<<".\n";
					fflush(stdout);
				}

				enterflagSTOP = true;

				if (!_Ifm2pStart){
					_m2p.updateHandPrev(_handpose);
					_Ifm2pStart = true;
				}
			}
//			_ret = _retPrev;

			if (_objpose.get<0>()[2] - _handpose.get<0>()[2] > -0.15 && _IfBalancing == true){

//				_IfBalancing = true;

				_cp_err = _targetPose.get<0>() - _objpose.get<0>();

				_vel_err = Eigen::Vector3d::Zero() - _objvel.get<0>();

				if (_vel_err.norm() > 0.02){
					k_p = 1.8;
					k_d = 1.2;//_k_d;
				}
				else{
					k_p = 5;
					k_d = 1;
				}
				_angleX = -k_p*_cp_err[1] - k_d*_vel_err[1];
				_angleY = k_p*_cp_err[0] + k_d*_vel_err[0];

				if (_angleX > _sat) _angleX = _sat; if (_angleX < -_sat) _angleX = -_sat;
				if (_angleY > _sat) _angleY = _sat; if (_angleY < -_sat) _angleY = -_sat;

				_targetPose.get<1>() = Eigen::Quaterniond(Eigen::AngleAxisd(_angleX, Eigen::Vector3d::UnitX()))*
								Eigen::Quaterniond(Eigen::AngleAxisd(_angleY, Eigen::Vector3d::UnitY()));

				_m2p.updateTarget(_targetPose);
				_m2p.updateHandPose(_handpose);

				_ret = _m2p.getCurrentPose();

			}
			else{
				_ret = _retPrev;
				_IfBalancing = false;
//				std::cout<<"\nObject fell off!! \n";
//				fflush(stdout);
			}
			break;
		/*************************************************
		 *
		 * ***********************************************/
		case THROW:

			static bool enterflagTH = false;
			static double tf = 0.4;
			static double t;
			static double A[6] = {29.2969, -21.4844, -4.6875, 0, 0, 0};//{48.8281, -48.8281, 4.6875, 0, 0, -0.1};
			static double B[6] = {37.1094, -13.6719, 1.5625, 0, 0, 0.62};//{48.8281, -25.3906, 4.6875, 0, 0, 0.6};
			static double C[6] = {97.6562, -89.8437, 9.375, 0, 0, -0.1};//{39.0625, -31.25, -6.25, 0, 0, 0};
			static double theta;
			static int i_start = 0;

			if(!enterflagTH){
				i_start = _i;
				enterflagTH = true;
			}

			t = (_i - i_start)*_period;

			if (t > tf){
				_prev_phase = _current_phase;
				_current_phase = STOPPING_throw;
				std::cout<<"\nDone throwing!! [Enter] to home.\n";
				fflush(stdout);
			}

			_ret.get<0>()[0] = A[0]*pow(t,5) + A[1]*pow(t,4) + A[2]*pow(t,3) + A[3]*pow(t,2) + A[4]*t + A[5];
			_ret.get<0>()[2] = B[0]*pow(t,5) + B[1]*pow(t,4) + B[2]*pow(t,3) + B[3]*pow(t,2) + B[4]*t + B[5];
			theta = C[0]*pow(t,5) + C[1]*pow(t,4) + C[2]*pow(t,3) + C[3]*pow(t,2) + C[4]*t + C[5];
			_ret.get<1>() = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()));

			break;
		/****************************************************
		 *
		 * **************************************************/
		case STOPPING_throw:
			_ret = _retPrev;
			break;
		/****************************************************
		 *
		 * **************************************************/
		default:
			printf("Phase variable error! current phase: %d", _current_phase);
			_ret = _retPrev;
			break;
		}

		_retPrev = _ret;
		_velPrev = _vel;

//		Eigen::AngleAxisd aa = _ret.get<1>();
//		aa = aa.inverse();//This inverse is because the libbarrett is somehow calculate the Tool Ori wrong
//		_ret.get<1>() = aa;

//		_ret.get<1>() = _retPrev.get<1>();
		this->outputValue->setData(&_ret);



		_i++;
	}



private:
	DISALLOW_COPY_AND_ASSIGN(throw_MotionPlanner);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};





#endif
