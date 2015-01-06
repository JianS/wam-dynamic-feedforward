/*
 * Soft catch motion planner is the high-level planner for the entire catching motion.
 * It tells which phase is going on and use different local planner to generate the 
 * motion of the robot.
 * Phases:
 * 		detecting free flying --> estimating intersection point of object parabola and robot 
 * 		work-space and robot move to that intersection point --> robot wait to moving down 
 * 		--> robot moving down to meet both position and velocity of obj --> approaching: 
 * 		end-effector attach to the obj --> arrestting: online planning a traj to arrest the object 
 * 		to stationary; with assuming no friction force.  
 * 
 * 
 * 	Author: js
 * 	08/29/2014
 */


#ifndef SOFTCATCH_MOTIONPLANNER_H
#define SOFTCATCH_MOTIONPLANNER_H

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
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <boost/circular_buffer.hpp>

#include "intersection.h"
#include "move2point.h"

using namespace barrett;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

enum catch_phases {
	WAITING = 0,
	HAND_RAISING = 1,
	VEL_MATCHING = 2,
	APPROACHING = 3,
	ARRESTING = 4,
	STOPPING = 5
};

class softcatch_MotionPlanner :	 public systems::System,
                                 public systems::SingleOutput<pose_type>
{
public:
	softcatch_MotionPlanner(parabolaEstimator* PE, double period, const std::string& sysName = "softcatch_MotionPlanner") :
		systems::System(sysName),
		systems::SingleOutput<pose_type>(this),
		input_objpose(this),
		input_objvel(this),
		input_hand(this),
		input_handVel(this),
		_PE(PE), _m2p(PE->getStartPose(), period),
		_period(period), _l_ec(0/*0.0243*/), _d_close(/*0.2+_l_ec*/0.05),
		_vel_limit(4), _acc_limit_down(20),
		_vel_z(0), _velPrev_z(0), _acc_z(0),
		_current_phase(WAITING), _prev_phase(WAITING),
//		_current_phase(STOPPING), _prev_phase(STOPPING),
		_objacc(_PE->getObjAcc()),
		_vel(Eigen::Vector3d::Zero()), _velPrev(Eigen::Vector3d::Zero()),
		_t_apex(0),
		_initpose(PE->getStartPose()), _ret(_initpose), _retPrev(_ret), _objpose(_ret),
		_interPose(_ret), _targetPose(_ret),
		_Ifm2pStart(false), _Ifclose(false),
		_IfBalancing(true),
		_k_p(1.8), _k_d(1), _sat(0.6),
		_i(0)
	{

	}

	~softcatch_MotionPlanner(){ mandatoryCleanUp(); }

	double getCurrentTime() {
		struct timespec ts;
		double t = 0.0;

		if (!clock_gettime( CLOCK_REALTIME, &ts) )
			t = (double)ts.tv_sec + ts.tv_nsec*1.0e-9;
		else std::cout<< "\nError!...(Getting current time.)\n ::parabolaEstimator::getCurrentTime()\n" << std::endl;

		return t;
	}

	bool Ifclose(pose_type handpose, pose_type objpose, double dis_close = 0.05/*0.16*/){
		Eigen::Vector3d sub;
		sub = handpose.get<0>() - objpose.get<0>();
		_Ifclose = (sub.norm() <= dis_close);
		return _Ifclose;
	}

	/* The safe region is defined by a box whose principle axis is collinear with z-axis.
	 * z_min --> bottom face height; z_max --> top face height
	 * x_min --> back face ; x_max --> front face
	 * y_min --> right face ; y_max --> left face
	 * */
	bool IfinSafeRegion(double z_min = 0.25, double z_max = 0.95,
						double r_workspace = 0.95){
		cp_type testPt = _handpose.get<0>();
		bool ret = true;
		if (testPt.norm() > r_workspace ||
			testPt[2]<z_min || testPt[2]>z_max ){
			ret = false;
			printf("\n!!!!!Endpoint exceeding safe region!!!!!\n");
			fflush(stdout);
		}
		testPt = _ret.get<0>();
		if (testPt.norm() > r_workspace ||
			testPt[2]<z_min || testPt[2]>z_max ){
			ret = false;
			printf("\n!!!!!Endpoint Ref exceeding safe region!!!!!\n");
			fflush(stdout);
		}

		if (!ret) _IfBalancing = false;

		return ret;
	}

	bool IfinSafeRegion(cp_type testPt, double z_min = 0.25, double z_max = 0.9,
							double r_workspace = 0.9){
		bool ret = true;
		if (testPt.norm() > r_workspace ||
			testPt[2]<z_min || testPt[2]>z_max ){
			ret = false;
		}
		return ret;
	}

	Input<pose_type> input_objpose;
	Input<pose_type> input_objvel;
	Input<pose_type> input_hand;
	Input<cv_type> input_handVel;


private:
	parabolaEstimator* _PE;
	intersection _interCal;
	move2point _m2p;

	double _period, _l_ec; // _l_ec is the distance from the end-point of WAM to the contact point
	double _d_close; // distance define if object is close to the hand.

	double _vel_limit, _acc_limit_down;

	double _vel_z, _velPrev_z, _acc_z;

	catch_phases _current_phase, _prev_phase;
	cp_type _objacc, _apex, _vel, _velPrev, _acc, _interVel;
	double _t_apex;
	pose_type _initpose, _ret, _retPrev, _objpose, _objvel,_handpose;
	pose_type _interPose;
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

	Eigen::Quaterniond CalHandQ_norm2input(cp_type input){
		Eigen::Quaterniond ret;
		cp_type handNormVec; //norm vector of hand
		cp_type rotaxis;
		double rotangle;

		handNormVec = _handpose.get<1>().toRotationMatrix()*Eigen::Vector3d::UnitZ();
		rotaxis = handNormVec.cross(input);
		rotangle = acos(handNormVec.dot(input)/(input.norm()));


		if (rotangle < 2*M_PI || rotangle > -2*M_PI)
			ret = Eigen::Quaterniond(Eigen::AngleAxisd(rotangle, rotaxis))*_handpose.get<1>();
		else
			ret = _handpose.get<1>();

		return ret;
	}

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
		case WAITING:
			_ret = _retPrev;
			if (_PE->IfFreeFly() && _PE->IfVelBuffFull()) {
				_current_phase = HAND_RAISING;
				_prev_phase = WAITING;
				printf("\nWAITING-->HAND_RAISING!\n");
				fflush(stdout);
			}
			break;
		/****************************************************
		 *
		 * **************************************************/
		case HAND_RAISING:
			static bool enterflagHR = false; // enter flag for HAND_RAISING
			if (!enterflagHR || (_i % 2 ==0) ){
				_interCal.updateValue(_objpose.get<0>(), _objvel.get<0>(), _PE->getObjAcc() );
				_interCal.evaluate();
//				std::cout<<"\nIntersection point: "<<_interCal.getInterPt()<<"its norm = "<<_interCal.getInterPt().norm()
//						<<", ifsuccess = "<<_interCal.Ifsuccess()<<", i = "<<_i<<".";
//				fflush(stdout);

			}

//			static cp_type handNormVec; //norm vector of hand
//			static cp_type rotaxis;
//			static double rotangle;
			static cp_type TargetHandNormVec; // Target norm vector of hand

			if (_interCal.Ifsuccess()){
//			if (1){
//				_interVel = _objvel.get<0>() + _interCal.getInterTime()*_objacc;
//				handNormVec = _handpose.get<1>().toRotationMatrix()*Eigen::Vector3d::UnitZ();
//				rotaxis = handNormVec.cross(-1*_interVel);
//				rotangle = acos(handNormVec.dot(-1*_interVel)/(_interVel.norm()));

//				if (rotangle < 2*M_PI || rotangle > -2*M_PI)
//					_interPose.get<1>() = Eigen::Quaterniond(Eigen::AngleAxisd(-rotangle, rotaxis))*_handpose.get<1>();
//				else
//					_interPose.get<1>() = Eigen::Quaterniond(1,0,0,0);

				_interVel = _objvel.get<0>() + _interCal.getInterTime()*_objacc;
				TargetHandNormVec = -1*(_interVel/_interVel.norm()) + 2*Eigen::Vector3d::UnitZ();
				_interPose.get<1>() = CalHandQ_norm2input(TargetHandNormVec);

//				Eigen::Matrix3d rm = _interPose.get<1>().toRotationMatrix();
//				Eigen::Vector3d vb;
//				vb[0] = 0; vb[1] = 0; vb[2] = _l_ec;
				_interPose.get<0>() = _interCal.getInterPt();// - rm*vb;

//				vb[0] = -0.3; vb[1] = 0; vb[2] = 0.6;
//				_interPose.get<0>() = vb;

				/*if (_i%300 == 0){
					std::cout<< "\nThe intersect point is: " <<_interPose.get<0>()<< ", orientation: "
							<<_interPose.get<1>().w()<<" i"<<_interPose.get<1>().x()<<" j"<<_interPose.get<1>().y()<<" k"<<_interPose.get<1>().z()
							<<".\n ";//Angle Axis:"<<rotangle<<rotaxis<<std::endl;
					fflush(stdout);
				}*/

				_m2p.updateHandPose(_handpose);
				if (!_Ifm2pStart){
					_m2p.updateHandPrev(_handpose);
					_Ifm2pStart = true;

					/*std::cout<< "\nThe target point is: " <<_interPose.get<0>()<< ", orientation: " <<
							_interPose.get<1>().w()<<" i"<<_interPose.get<1>().x()<<" j"<<_interPose.get<1>().y()<<" k"<<_interPose.get<1>().z()
							<<"."<<std::endl;
					fflush(stdout);*/
				}
				_m2p.updateTarget(_interPose);
				_ret = _m2p.getCurrentPose();
			}
			else{
				if (/*!enterflagHR ||*/ (_i%700 == 0) ){
					printf("\nObject won't go into the work-space! Cannot catch!! \n");// HAND_RAISING-->STOPPING \n");
					fflush(stdout);
				}

			}

			enterflagHR = true;

			Ifclose(_handpose, _objpose, 0.2);
			if (_m2p.Ifarrive() || _Ifclose ){
				_current_phase = VEL_MATCHING;
//				_current_phase = STOPPING;
//				_IfBalancing = false;
				_prev_phase = HAND_RAISING;

				_apex = _PE->getApex();
				_t_apex = _PE->getT2apex();

				/*printf("\nHAND_RAISING-->VEL_MATCHING!\n"
						"m2p.ifarrive = %d; Ifclose = %d.\n"
						"Time after apex is: %4.3fs.\n"
						"Time of apex is: %4.3fs.\n",
						_m2p.Ifarrive(), _Ifclose, (getCurrentTime() - _t_apex), _t_apex);
				fflush(stdout);*/

				_Ifm2pStart = false;
			}

			break;
		/*************************************************
		 * detail of the calculation in VEL_MATCHING see softcatch_simulation.m
		 * ***********************************************/
		case VEL_MATCHING:
			static bool handstartmove = false; // flag for if hand starts moving down
			static cp_type d0;
			static cp_type catchVel;
			static double tf;
			static double tf_min;
			static double tf_max;

			tf = getCurrentTime() - _t_apex;

			Ifclose(_handpose, _objpose, 0.03/*0.09*/);
			if (fabs(_handVel[2]) >= (_vel_limit-1) || -_handVel[2]+0.2 >= fabs(_objvel.get<0>()[2]) || _Ifclose || !IfinSafeRegion()){
				if ( handstartmove ){
					_current_phase = ARRESTING;
					_prev_phase = VEL_MATCHING;
					printf("\nVEL_MATCHING-->ARRESTING!\n"
							"_velRef_z = %3.2f; Ifclose = %d.\n"
							"_handVel_z = %4.2f, _objvel_z = %4.2f. \n"
							"Time after apex is: %4.3fs.\n",
							_vel[2], _Ifclose, _handVel[2], _objvel.get<0>()[2], tf);
					fflush(stdout);
				}
				else{
					_current_phase = STOPPING;
					_prev_phase = VEL_MATCHING;
					_IfBalancing = false;
					printf("\nCannot catch, VEL_MATCHING-->STOPPING!\n"
							"_vel_z = %3.2f; Ifclose = %d.\n"
							"Time after apex is: %4.3fs.\n", _vel[2], _Ifclose, tf);
					fflush(stdout);
				}
//				_Ifm2pStart = false;
			}

			_acc[2] = pow((_objvel.get<0>()[2]-_vel[2]),2)*0.5/(_handpose.get<0>()[2] - _objpose.get<0>()[2] + 0.03/*0.14*/) + _objacc[2];
			_acc[1] = pow((_objvel.get<0>()[1]-_vel[1]),2)*0.5/(_handpose.get<0>()[1] - _objpose.get<0>()[1] + 0.00) + _objacc[1];
			_acc[0] = pow((_objvel.get<0>()[0]-_vel[0]),2)*0.5/(_handpose.get<0>()[0] - _objpose.get<0>()[0] + 0.00/*0.07*/) + _objacc[0];

			if (_acc[2] < -_acc_limit_down) _acc[2] = -_acc_limit_down;
			if (_acc[2] > _acc_limit_down) _acc[2] = _acc_limit_down;

			if (_acc[1] < -_acc_limit_down) _acc[1] = -_acc_limit_down;
			if (_acc[1] > _acc_limit_down) _acc[1] = _acc_limit_down;

			if (_acc[0] < -_acc_limit_down) _acc[0] = -_acc_limit_down;
			if (_acc[0] > _acc_limit_down) _acc[0] = _acc_limit_down;

			if (!handstartmove){

//				d0 = _interCal.getInterPt() - _apex;
				d0 = _handpose.get<0>() - _apex;

				tf_min = fabs(2*d0[2]/_vel_limit);
				tf_max = sqrt(fabs(2*d0[2]*(_objacc[2] - _acc_limit_down)/(_objacc[2] * _acc_limit_down)));

				catchVel = 2*d0/tf;

//				if (tf > tf_max)
//					tf = tf_max;

//				_acc_z = 2*_objacc[2]*d0[2]/(2*d0[2] - tf*tf*_objacc[2]);
//				_acc[2] = _objvel.get<0>()[2]*_objvel.get<0>()[2]*0.5/(_handpose.get<0>()[2] - _objpose.get<0>()[2] + 0.05/*0.14*/) + _objacc[2];
//				_acc[1] = _objvel.get<0>()[1]*_objvel.get<0>()[1]*0.5/(_handpose.get<0>()[1] - _objpose.get<0>()[1] + 0.02) + _objacc[1];
//				_acc[0] = _objvel.get<0>()[0]*_objvel.get<0>()[0]*0.5/(_handpose.get<0>()[0] - _objpose.get<0>()[0] + 0.035/*0.07*/) + _objacc[0];


				if (fabs(_acc[2]) > 15 && tf <= tf_max && tf >= tf_min){
					std::cout<<"\nCurrent tf = "<<tf<<", tf_min = "<<tf_min<<", tf_max = "<<tf_max<<
							". \nacc = "<<_acc<<".\n"
							 << "interPoint: "<<_interCal.getInterPt()<<", Apex: "<<_apex<<". \n"
							 << "d0: " << d0 << ".\n"
							 << "Catching velocity: "<< catchVel << "\n"
//							 << "abs(-4):"<<abs(-4)<<"; sqrt(abs(-4)):"<<sqrt(abs(-4))<<".\n"
							 <<"object linear vel: "<< _objvel.get<0>() << ".\n";
					fflush(stdout);
					handstartmove = true;


				}
				else{
//					_ret = _retPrev;
//					break;
					if (tf_min > tf_max && _i%50 == 0){
						std::cout<<"\nCannot catch!! tf_min = "<< tf_min <<", tf_max = "<<tf_max<<", tf = "<<tf<<". \n";
						fflush(stdout);
					}

//					if (Ifclose(_handpose, _objpose, 0.1/*0.35*/)){
//						_acc[2] = -18;
//					}
//					else{
						_vel = Eigen::Vector3d::Zero();
						_acc = Eigen::Vector3d::Zero();
//					}

				}

			}

			_vel = _velPrev + _acc*_period;
			_ret.get<0>() = _retPrev.get<0>() + _velPrev*_period + _acc*(0.5*_period*_period);

			TargetHandNormVec = -1*(_objvel.get<0>()/_objvel.get<0>().norm()) + 2*Eigen::Vector3d::UnitZ();
			_ret.get<1>() = CalHandQ_norm2input(TargetHandNormVec);

			break;
		/****************************************************
		 *
		 * **************************************************/
		case APPROACHING:
			break;
		/****************************************************
		 *
		 * **************************************************/
		case ARRESTING:
			static bool enterflagAR = false; // enter flag for ARRESTING
//			static pose_type targetpose = _interPose;
			static cv_type init_vel;
			static double dis; // the distance from arresting start to finally stop
//			static double _t_arr; // time from apex to arrest starting
			static pose_type init_pose;
			_sat = 0.4;

			_vel_err = Eigen::Vector3d::Zero() - _objvel.get<0>();
			_angleX = -_k_d*_vel_err[1];
			_angleY = _k_d*_vel_err[0];

			if (_angleX > _sat) _angleX = _sat; if (_angleX < -_sat) _angleX = -_sat;
			if (_angleY > _sat) _angleY = _sat; if (_angleY < -_sat) _angleY = -_sat;

			_targetPose.get<1>() = Eigen::Quaterniond(Eigen::AngleAxisd(_angleX, Eigen::Vector3d::UnitX()))*
								Eigen::Quaterniond(Eigen::AngleAxisd(_angleY, Eigen::Vector3d::UnitY()));

			if (!enterflagAR){
//				_t_arr = getCurrentTime() - _t_apex;

//				init_vel = _objvel.get<0>();
				init_vel = _handVel;

				dis = pow(init_vel.norm(),2)/(2*_m2p.GetMaxAcc());

				if (dis < 0.1) dis = 0.1;

//				_targetPose.get<0>() = _retPrev.get<0>() + init_vel*(dis/init_vel.norm());
				_targetPose.get<0>() = _handpose.get<0>() + init_vel*(dis/init_vel.norm());

				while(_targetPose.get<0>()[2] < 0.35){
					dis = dis - 0.05;
					_targetPose.get<0>() = _handpose.get<0>() + init_vel*(dis/init_vel.norm());
				}

//				_targetPose.get<1>() = Eigen::Quaterniond(1,0,0,0);

				/*std::cout << "\nArresting start position: " << _retPrev.get<0>() << "m.\n"
						  << "Final target hand position: " << _targetPose.get<0>() << "m.\n"
						  << "velocity of obj before arresting: "<< init_vel <<"m/s."
						  << "objvel_z from time = "<< _objacc[2]*_t_arr <<std::endl;
				fflush(stdout);*/

				_m2p.reset();
//				init_pose.get<0>() = _handpose.get<0>();
//				init_pose.get<1>() = _ret.get<1>();
				init_pose = _handpose;
				if (!_Ifm2pStart){
					_m2p.updateHandPrev(init_pose);
					_Ifm2pStart = true;
				}
				_m2p.updateHandVel_linear(init_vel.norm());
				_m2p.updateMaxVel(init_vel.norm());

//				_m2p.updateHandPrev(_handpose);

				enterflagAR = true;
			}

			_m2p.updateTarget(_targetPose);
			_m2p.updateHandPose(_handpose);

			_ret = _m2p.getCurrentPose();

//			if (_m2p.Ifarrive()){
			if ( _m2p.Ifarrive() /*|| Ifclose(_handpose, _targetPose, 0.01)*/ || !IfinSafeRegion() ) {
				_current_phase = STOPPING;
				_prev_phase = ARRESTING;

				std::cout<<"\nARRESTING-->STOPPING!\n m2p.ifarrive ="<<_m2p.Ifarrive()
						<<"\nInitial obj vel = "<<init_vel<<", dis = "<<dis
						<<"\nTarget final point: "<<_targetPose.get<0>()
						<<"\nTime after apex is:"<<(getCurrentTime() - _t_apex)<<".\n";
				fflush(stdout);
				_Ifm2pStart = false;
			}
			break;
		/****************************************************
		 *
		 * **************************************************/
		case STOPPING:
			static bool enterflagSTOP = false;
//			static bool _IfBalancing = true;
//			static cp_type cp_err;
//			static cp_type vel_err;
			static double k_p = _k_p;//1.8;//1.6;
			static double k_d = _k_d;//1.8;
//			static double sat = 0.6;
//			static double angleX; static double angleY;
			static cp_type cp_err_integral = Eigen::Vector3d::Zero();
			static double k_i = 0.0;
			static double sat_i = 0.3;
			static int i_prev = 0;
			_sat = 0.6;

			if (!enterflagSTOP || (_i-i_prev)==800){

				i_prev = _i;
				if (_IfBalancing){
					Eigen::AngleAxisd aa = _handpose.get<1>();
					cp_type axis = aa.axis();

					std::cout<<"\nIn STOPPING! And Balancing! \nReference tool pose: "<< _targetPose.get<0>()
//							<< "w:"<<_targetPose.get<1>().w()<<" , i:"<<_targetPose.get<1>().x()<<" j:"<<_targetPose.get<1>().y()<<" k:"<<_targetPose.get<1>().z()
							<<".\n"
							<< "Object location: "<< _objpose.get<0>()<<".\n"
							<< "cp_err: " << _cp_err <<".\n"
							<< "Object velocity: "<< _objvel.get<0>()<<"\n"
//							<< "integral_err: "<< cp_err_integral
							<< "angleX = "<<_angleX<<", angleY = "<<_angleY<<".\n"
							<< "HandOri: angle = " << aa.angle() <<", axis = "<< axis
							<< "\n m2pIfreach: "<<_m2p.Ifarrive()<<". \n";
					fflush(stdout);
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

				cp_err_integral = cp_err_integral + _cp_err;
				if (cp_err_integral[0]>sat_i) cp_err_integral[0] = sat_i;
				if (cp_err_integral[0]<-sat_i) cp_err_integral[0] = -sat_i;
				if (cp_err_integral[1]>sat_i) cp_err_integral[1] = sat_i;
				if (cp_err_integral[1]<-sat_i) cp_err_integral[1] = -sat_i;


				_vel_err = Eigen::Vector3d::Zero() - _objvel.get<0>();

				if (_vel_err.norm() > 0.02){
					k_p = 1.8;
					k_d = 1.2;//_k_d;
				}
				else{
					k_p = 5;
					k_d = 1;
				}
				_angleX = -k_p*_cp_err[1] - k_d*_vel_err[1] - k_i*cp_err_integral[1];
				_angleY = k_p*_cp_err[0] + k_d*_vel_err[0] + k_i*cp_err_integral[0];

				if (_angleX > _sat) _angleX = _sat; if (_angleX < -_sat) _angleX = -_sat;
				if (_angleY > _sat) _angleY = _sat; if (_angleY < -_sat) _angleY = -_sat;

				_targetPose.get<1>() = Eigen::Quaterniond(Eigen::AngleAxisd(_angleX, Eigen::Vector3d::UnitX()))*
								Eigen::Quaterniond(Eigen::AngleAxisd(_angleY, Eigen::Vector3d::UnitY()));
//				_targetPose.get<1>() = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/7, Eigen::Vector3d::UnitY()));

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
	DISALLOW_COPY_AND_ASSIGN(softcatch_MotionPlanner);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};





#endif
