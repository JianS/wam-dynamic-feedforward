/*
 * Intersection class is for solving the intersection point of a parabola and the
 * work-space. Here assume the work-space is sphere shape with the radius r.
 * Newton's method is used here to solve the nonlinear equation.
 *
 *
 * 	Author: js
 * 	08/29/2014
 */


#ifndef INTERSECTION_H
#define INTERSECTION_H

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

class intersection
{
public:
	intersection() : _Ifupdated(false), _Ifsuccess(false)
	{}

	~intersection(){}

	void updateValue(cp_type objpos, cp_type objvel, cp_type objacc,
					   cp_type workspaceCenter = Eigen::Vector3d::Zero(), double workspace_r = 0.87)
	{
		x0 = objpos[0]; y0 = objpos[1]; z0 = objpos[2];
		Vx = objvel[0]; Vy = objvel[1]; Vz = objvel[2];
		ax = objacc[0]; ay = objacc[1]; az = objacc[2];

		xc = workspaceCenter[0];yc = workspaceCenter[1];zc = workspaceCenter[2];
		r = workspace_r;

		_Ifupdated = true;
	}

	double func(double t){
		return -r*r + pow((0.5*ax*t*t + t*Vx + x0 - xc),2) +
			pow((0.5*ay*t*t + t*Vy + y0 - yc),2) +
			pow((0.5*az*t*t + t*Vz + z0 - zc),2);
	}

	double Dfunc(double t){
		return 2*(ax*t + Vx)*(0.5*ax*t*t + t*Vx + x0 - xc) +
			2*(ay*t + Vy)*(0.5*ay*t*t + t*Vy + y0 - yc) +
			2*(az*t + Vz)*(0.5*az*t*t + t*Vz + z0 - zc);
	}

	double GetX(double t){
		return x0 + Vx*t + 0.5*ax*t*t;
	}
	double GetY(double t){
		return y0 + Vy*t + 0.5*ay*t*t;
	}
	double GetZ(double t){
		return z0 + Vz*t + 0.5*az*t*t;
	}

	cp_type getInterPt() {
		return _interPt;
	}

	double getInterTime() {
		return _interT;
	}

	bool Ifsuccess(){
		return _Ifsuccess;
	}

	bool evaluate(){
		_Ifsuccess = false;

		if (_Ifupdated){
			double t1 = 0, t2 = 10, err, j;
			int i;
			// err = t2 - t1;
			_Ifsuccess = false;
			for (j = 0; j < 2; j+=0.05){
				t1 = j; t2 = 10;
				for (i = 0; i < 20; i++){
					t2 = t1 - func(t1)/Dfunc(t1);
					err = t2 - t1;
					// printf("\n err:%f, z:%f, t2:%f, t1:%f, j:%f, i:%d.\n",
					// 		err, Cal.GetZ(t2), t2,t1,j,i);
					if (fabs(err) < 1.0e-15) {
						break;
					}
					t1 = t2;
				}
				if ((fabs(err) < 1.0e-15) && (GetZ(t2) > zc + 0.1)) {
					_interPt[0] = GetX(t2);
					_interPt[1] = GetY(t2);
					_interPt[2] = GetZ(t2);

					if ( fabs(_interPt.norm() - r) <= 0.01 ){
						_Ifsuccess = true;
						_interT = t2;
//						std::cout<<"\n_interPt = "<<_interPt<<", its norm = "<<_interPt.norm()<<", r = "<<r<<"\n";
						break;
					}
				}
			}

			/*if (_Ifsuccess){
				_interPt[0] = GetX(t2);
				_interPt[1] = GetY(t2);
				_interPt[2] = GetZ(t2);

				_interT = t2;
			}*/

			/*else{
				printf("\nNot intersected...  j = %f, i = %d.\n", j, i);
			}*/
			_Ifupdated = false;

		}
		else{
			printf("\nHavn't update state! ::intersection::evaluate()");

		}

		return _Ifsuccess;
	}



private:

	bool _Ifupdated, _Ifsuccess;
	cp_type _interPt;
	double x0, y0, z0, xc,yc,zc,Vx,Vy,Vz, ax,ay,az, r;
	double _interT; //Time from now to intersect

};






#endif
