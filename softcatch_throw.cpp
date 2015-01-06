/*
 *
 *
 *    Author: js
 *    NxR lab, Northwestern University
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Core>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>
#include <barrett/products/safety_module.h>

#define BARRETT_SMF_VALIDATE_ARGS
#define BARRETT_SMF_CONFIGURE_PM
#include <barrett/standard_main_function.h>


#include "../include/OptiTrackVision.h"
#include "../include/parabolaEstimator.h"
#include "include/softcatch_MotionPlanner.h"
#include "include/throw_MotionPlanner.h"

#include <NatNetLinux/NatNet.h>
#include <Eigen/Core>
#include <barrett/math/traits.h>

using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;

bool validate_args(int argc, char** argv) {
	if (argc != 6) {
		printf("Usage: %s <wam_traj> <cam_traj> <wamRef> <objvel> <wamToolVel>\n", argv[0]);
		return false;
	}
	return true;
}

bool configure_pm(int argc, char** argv, ::barrett::ProductManager& pm) {
	//Set the velocity limit
	SafetyModule* sm = pm.getSafetyModule();
	double warning = 1.5, fault = 4;
	sm->setVelocityLimit(fault, warning);
	printf("\nThe end-point velocity limits: warning: %3.2f m/s, fault: %3.2f m/s. \n", warning, fault);

	//Set the torque limit
	warning = 3;
	fault = 5;
	sm->setTorqueLimit(fault, warning);
	printf("\nThe torque safety limits have been set to: warning: %5.1f, fault: %5.1f. \n", warning*1000, fault*1000);

	// Set the period
	double period = 0.002777; //360Hz
	period = 0.002;
    pm.getExecutionManager(period); // Default rate is 0.002 seconds, new rate is
    printf("\nThe current frequency is: %fHz. \n", 1/period);
    return true;
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// Initialize the vision system
/********************************************************************/
	std::string local ("129.105.69.25");
	std::string server ("129.105.69.46");
	char* address[2] = {new char[15], new char[16]};
	std::strcpy (address[0], local.c_str());
	std::strcpy (address[1], server.c_str());


	printf("Press [Enter] then start the vision system.\n");
	waitForEnter();
	//visionCP vCP(address);
	OptiTrackVision vision(address);
/********************************************************************/


	/*disconnect<cp_type>(wam.tpController.feedbackInput);
	disconnect(wam.toolPosition.output);

	connect(vision.cpOutput_Hand->output, wam.tpController.feedbackInput);
*/


	/*
	 * set up logging
	 */
	char tmpFile_wam[] = "/tmp/btXXXXXX", tmpFile_cam[] = "/tmp/btXXXXXX" ,
		  tmpFile_wamRef[] = "/tmp/btXXXXXX", tmpFile_objvel[] = "/tmp/btXXXXXX",
		  tmpFile_wamToolVel[] = "/tmp/btXXXXXX";
		if ( (mkstemp(tmpFile_wam) == -1) ||(mkstemp(tmpFile_cam) == -1)
				||(mkstemp(tmpFile_wamRef) == -1) || (mkstemp(tmpFile_objvel) == -1)
				||(mkstemp(tmpFile_wamToolVel) == -1)) {
			printf("ERROR: Couldn't create temporary file!\n");
			return 1;
		}

		const double T_s = pm.getExecutionManager()->getPeriod();

		//wam.gravityCompensate();

		//systems::Ramp time(pm.getExecutionManager()); //timer for traj tracking
		systems::Ramp time2(pm.getExecutionManager()); //timer for logging

		typedef boost::tuple<double, cp_type> cp_sample_type;
		typedef boost::tuple<double, cv_type> cv_sample_type;
		typedef boost::tuple<double, pose_type> pose_sample_type;
		typedef boost::tuple<double, math::Vector<7>::type> time_sample_type;


//		systems::TupleGrouper<double, cp_type> cpLogTg_wam, cpLogTg_cam;
		systems::TupleGrouper<double, pose_type> poseLogTg_wamRef, poseLogTg_wam, poseLogTg_cam, poseLogTg_objvel;
		systems::TupleGrouper<double, cv_type> cvLogTg_wamToolVel;
//		systems::TupleGrouper<double, math::Vector<7>::type> cpLogTg_time;

			// Record at 1/nth of the loop rate
		const size_t n = 1;
	/*	systems::PeriodicDataLogger<cp_sample_type> cpLogger_wam(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<cp_sample_type>(tmpFile_wam, n*T_s), n);

		systems::PeriodicDataLogger<cp_sample_type> cpLogger_cam(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<cp_sample_type>(tmpFile_cam, n*T_s), n);*/



		systems::PeriodicDataLogger<pose_sample_type> poseLogger_wam(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<pose_sample_type>(tmpFile_wam, n*T_s), n);

		systems::PeriodicDataLogger<pose_sample_type> poseLogger_cam(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<pose_sample_type>(tmpFile_cam, n*T_s), n);

		systems::PeriodicDataLogger<pose_sample_type> poseLogger_wamRef(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<pose_sample_type>(tmpFile_wamRef, n*T_s), n);

		systems::PeriodicDataLogger<pose_sample_type> poseLogger_objvel(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<pose_sample_type>(tmpFile_objvel, n*T_s), n);

		systems::PeriodicDataLogger<cv_sample_type> cvLogger_wamToolVel(pm.getExecutionManager(),
				new barrett::log::RealTimeWriter<cv_sample_type>(tmpFile_wamToolVel, n*T_s), n);

//		systems::PeriodicDataLogger<time_sample_type> cpLogger_time(pm.getExecutionManager(),
//				new barrett::log::RealTimeWriter<time_sample_type>(tmpFile_time, n*T_s), n);



	std::cout<< "The tool-position control gain(Kp,Ki,Kd):" << wam.tpController.getKp()<<wam.tpController.getKi()<<wam.tpController.getKd() << std::endl;
	std::cout<< "The tool-orientation control gain(Kp,Kd):" << wam.toController.getKp()<<", "<<wam.toController.getKd()<<std::endl;

	wam.gravityCompensate(true);
	printf("Gravity Compensation is on!\n");
	printf("\n*****Press [Enter] to move to the start position.*****\n");
	fflush(stdout);
	waitForEnter();

	/*
	 * Get start point position in vision
	 */
	pose_type start;
	//std::cout<<start<<std::endl;
//	std::vector<double> TM = vision.visionFilter1->getTransferMatrix();

	for (int i = 0; i<20; i++)
		start = vision.visionFilter1->getLatestPose(2);
	vision.visionFilter1->ResetPkgbuff();

//	start.get<0>() = vision.visionFilter1->Cam2WamPos(start.get<0>());
//	start.get<1>() = vision.visionFilter1->Cam2WamOri(start.get<1>());

	Eigen::AngleAxisd aa(M_PI*0., Eigen::Vector3d::UnitY());
	Eigen::Quaterniond transQ(aa);
//	transQ.w() = 0;transQ.x() = 1;transQ.y() = 0;transQ.z() = 0;
	start.get<1>() = transQ;

	start.get<0>()[0] = -0.4; start.get<0>()[1] = 0; start.get<0>()[2] = 0.6;
//	start.get<0>()[0] = -0.3; start.get<0>()[1] = 0; start.get<0>()[2] = 0.4;
//	start.get<1>().w() = 1;start.get<1>().x() = 0;start.get<1>().y() = 0;start.get<1>().z() = 0;
//	start.get<1>().w() = 0;start.get<1>().x() = 0.707;start.get<1>().y() = 0;start.get<1>().z() = 0.707;


	std::cout<< "\nThe target start point is: " <<start.get<0>()<< ", distance to center: "
				<<start.get<0>().norm()<< "m, orientation: " <<
				start.get<1>().w()<<" i"<<start.get<1>().x()<<" j"<<start.get<1>().y()<<" k"<<start.get<1>().z()
				<<", wait for WAM finish moving."<<std::endl;

	wam.moveTo(start, true, 1, 1);

	start = wam.getToolPose();

	std::cout<< "\nThe start point is: " <<start.get<0>()<< ", distance to center: "
			<<start.get<0>().norm()<<"m, orientation: " <<
			start.get<1>().w()<<" i"<<start.get<1>().x()<<" j"<<start.get<1>().y()<<" k"<<start.get<1>().z()
			<<". \n\n*****Press [Enter] to start setting logging.*****"<<std::endl;

	cp_type acc; // acceleration of the object free flight
	acc[0] = -1;  acc[1] = -0.45; acc[2] = -9;
	parabolaEstimator pe(start, T_s, acc);
	softcatch_MotionPlanner sm(&pe, T_s);

	waitForEnter();

	//Traj<DOF> traj(start, T_s);
	//connect(vision.cpOutput_Hand->output, traj.input_hand);



	{
		// Make sure the Systems are connected on the same execution cycle
		// that the time is started. Otherwise we might record a bunch of
		// samples all having t=0; this is bad because the Spline requires time
		// to be monotonic.
		BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());

		connect(vision.poseOutput_Obj_filtered->output, pe.input);
		connect(vision.poseOutput_Obj_filtered->output, sm.input_objpose);
		connect(pe.output, sm.input_objvel);

		connect(wam.toolPose.output, sm.input_hand);
		connect(wam.toolVelocity.output, sm.input_handVel);
//		connect(vision.poseOutput_Obj_filtered->output, traj.input_obj);
//		connect(pe.output, traj.input_obj);
//		connect(pe._2ndOutput.output, traj.input_obj);

		/*connect(time2.output, cpLogTg_wam.template getInput<0>());
		connect(wam.toolPosition.output, cpLogTg_wam.template getInput<1>());
		connect(cpLogTg_wam.output, cpLogger_wam.input);

		connect(time2.output, cpLogTg_cam.template getInput<0>());
//		connect(vision.cpOutput_Hand->output, cpLogTg_cam.template getInput<1>());
		connect(traj.output, cpLogTg_cam.template getInput<1>());
		connect(cpLogTg_cam.output, cpLogger_cam.input);*/

		connect(time2.output, poseLogTg_wam.template getInput<0>());
		connect(wam.toolPose.output, poseLogTg_wam.template getInput<1>());
//		connect(pe._2ndOutput.output, poseLogTg_wam.template getInput<1>());
		connect(poseLogTg_wam.output, poseLogger_wam.input);

		connect(time2.output, poseLogTg_cam.template getInput<0>());
		connect(vision.poseOutput_Obj_filtered->output, poseLogTg_cam.template getInput<1>());
		connect(poseLogTg_cam.output, poseLogger_cam.input);

		connect(time2.output, poseLogTg_wamRef.template getInput<0>());
		connect(sm.output, poseLogTg_wamRef.template getInput<1>());
		connect(poseLogTg_wamRef.output, poseLogger_wamRef.input);

		connect(time2.output, poseLogTg_objvel.template getInput<0>());
		connect(pe.output, poseLogTg_objvel.template getInput<1>());
		connect(poseLogTg_objvel.output, poseLogger_objvel.input);

		connect(time2.output, cvLogTg_wamToolVel.template getInput<0>());
		connect(wam.toolVelocity.output, cvLogTg_wamToolVel.template getInput<1>());
		connect(cvLogTg_wamToolVel.output, cvLogger_wamToolVel.input);
	}

	printf("\n*****Press [Enter] to start the task.*****\n");
	//fflush(stdout);
	waitForEnter(); // (11_2_14) Need to wait for a little while before start track signal. Can't figure why...

	time2.start();  // Start the task here

	printf("\n***** Trajectory tracking on! You can throw the object now! *****\n");
	wam.trackReferenceSignal(sm.output);

	printf("\nIn tracking! [Enter] to throw.\n");
	waitForEnter();

	sm.Idle();

	// Setting up throw
	start.get<0>()[0] = 0.; start.get<0>()[1] = 0.; start.get<0>()[2] = 0.62;
	throw_MotionPlanner tm(T_s, wam.getToolPose(), start);
	{
		connect(vision.poseOutput_Obj_filtered->output, tm.input_objpose);
		connect(pe.output, tm.input_objvel);

		connect(wam.toolPose.output, tm.input_hand);
		connect(wam.toolVelocity.output, tm.input_handVel);

		disconnect(poseLogTg_wamRef.template getInput<1>());
		connect(tm.output, poseLogTg_wamRef.template getInput<1>());
	}
	wam.trackReferenceSignal(tm.output);

	printf("\nIn throwing! [Enter] to go to start position.\n");
	waitForEnter();
	tm.GotoStart();

	printf("\nIn throwing! [Enter] to start throwing.\n");
	waitForEnter();
	tm.StartThrow();

	printf("\n[Enter] to stop and go back to home position.\n");
	waitForEnter();

	/*cpLogger_wam.closeLog();
	disconnect(cpLogger_wam.input);
	cpLogger_cam.closeLog();
	disconnect(cpLogger_cam.input);*/
	poseLogger_wam.closeLog();
	disconnect(poseLogger_wam.input);

	poseLogger_cam.closeLog();
	disconnect(poseLogger_cam.input);

	poseLogger_wamRef.closeLog();
	disconnect(poseLogger_wamRef.input);

	poseLogger_objvel.closeLog();
	disconnect(poseLogger_objvel.input);

	cvLogger_wamToolVel.closeLog();
	disconnect(cvLogger_wamToolVel.input);
//	cpLogger_time.closeLog();
//	disconnect(cpLogger_time.input);

	time2.stop();


	usleep(100000);
	wam.moveHome();

/*******************************************************************
 * Translate logged data to csv files and end up.
 * */

	/*log::Reader<cp_sample_type> lr_wam(tmpFile_wam);
	lr_wam.exportCSV(argv[1]);
	printf("Wam CP data has been written to %s.\n", argv[1]);

	log::Reader<cp_sample_type> lr_cam(tmpFile_cam);
	lr_cam.exportCSV(argv[2]);
	printf("Cam CP data has been written to %s.\n", argv[2]);*/

	log::Reader<pose_sample_type> lr_wam(tmpFile_wam);
	lr_wam.exportCSV(argv[1]);
	printf("WAM Pose data has been written to %s.\n", argv[1]);

	log::Reader<pose_sample_type> lr_cam(tmpFile_cam);
	lr_cam.exportCSV(argv[2]);
	printf("Cam Pose data has been written to %s.\n", argv[2]);

	log::Reader<pose_sample_type> lr_wamRef(tmpFile_wamRef);
	lr_wamRef.exportCSV(argv[3]);
	printf("WAM Ref Pose data has been written to %s.\n", argv[3]);

	log::Reader<pose_sample_type> lr_objvel(tmpFile_objvel);
	lr_objvel.exportCSV(argv[4]);
	printf("Object velocity has been written to %s.\n", argv[4]);

	log::Reader<cp_sample_type> lr_wamToolVel(tmpFile_wamToolVel);
	lr_wamToolVel.exportCSV(argv[5]);
	printf("WAM Tool velocity data has been written to %s.\n", argv[5]);

//	log::Reader<time_sample_type> lr_time(tmpFile_time);
//	lr_time.exportCSV(argv[3]);
//	printf("Cam Time data has been written to %s.\n", argv[3]);

	std::remove(tmpFile_wam);
	std::remove(tmpFile_cam);
	std::remove(tmpFile_wamRef);
	std::remove(tmpFile_objvel);
	std::remove(tmpFile_wamToolVel);


	printf("\nPress [Enter] then stop the vision system.\n");
	waitForEnter();
	{
		vision.stopvision();
	}


	printf("Press [Enter] then idle the WAM.\n");
	wam.gravityCompensate(false);
	waitForEnter();
	wam.idle();

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
