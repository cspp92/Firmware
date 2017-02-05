/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file main.c
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <vector>
#include <matrix/math.hpp>
#define _USE_MATH_DEFINES
#include <cmath>
#include <array>

/* process-specific header files */
//#include "params.h"

/* Prototypes */

/**
 * Initialize all parameter handles and values
 *
 */
extern "C" int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
extern "C" int parameters_update(const struct param_handles *h, struct params *p);

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
extern "C" __EXPORT int offsetCalcStart_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int offsetCalc(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);


/**
 * Control heading.
 *
 * This very simple heading to roll angle controller outputs the desired roll angle based on
 * the current position of the system, the desired position (the setpoint) and the current
 * heading.
 *
 * @param pos The current position of the system
 * @param sp The current position setpoint
 * @param att The current attitude
 * @param att_sp The attitude setpoint. This is the output of the controller
 */
//void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
//		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

//Input Positions
double p1[3];
double p2[3];
double p12[3];

//Input Offsets
double offset_height;
double offset_dist;
double offset_angle;

double pnorm[3];
double psoll[3];
double rot_matrix_yaw[2][3];
double rot_matrix_new_z[3][3];
double magnitude;
//Radius of earth in meters
double rad_earth_eq=6378137;
double rad_earth_pole=6356752.3;
double rad_earth_p2;
double rad_earth_p1;
double rad_earth_soll;
double rot_to_dist;
double new_z[3];
double alt;
//static struct params p;
//static struct param_handles ph;


/* Main Thread */
int offsetCalc(int argc, char *argv[])
{
    //TESTING
    p1[0]=47.3977420;
    p1[1]=8.5455940;
    p1[2]=25;

    p2[0]=47.3977443;
    p2[1]=8.5457241;
    p2[2]=25;
    alt=p2[2];

    //Deg to Rad
    offset_angle*= M_PI/180.0;

    //Transform spherical to cartesian
    rad_earth_p2=sqrt((pow(pow(rad_earth_eq,2)*cos(p2[0]*M_PI/180.0),2)+pow(pow(rad_earth_pole,2)*sin(p2[0]*M_PI/180.0),2))/(pow(rad_earth_eq*cos(p2[0]*M_PI/180.0),2)+pow(rad_earth_pole*sin(p2[0]*M_PI/180.0),2)));
    rad_earth_p2+=p2[2];//altitude regarding sea level or actual position

    rad_earth_p1=sqrt((pow(pow(rad_earth_eq,2)*cos(p1[0]*M_PI/180.0),2)+pow(pow(rad_earth_pole,2)*sin(p1[0]*M_PI/180.0),2))/(pow(rad_earth_eq*cos(p1[0]*M_PI/180.0),2)+pow(rad_earth_pole*sin(p1[0]*M_PI/180.0),2)));
    rad_earth_p1+=p1[2];//altitude regarding sea level or actual position
    rad_earth_soll=rad_earth_p2+offset_height;

    p1[0]=rad_earth_p2*sin(p1[0]*M_PI/180.0)*cos(p1[1]*M_PI/180.0);
    p1[1]=rad_earth_p2*sin(p1[0]*M_PI/180.0)*sin(p1[1]*M_PI/180.0);
    p1[2]=rad_earth_p2*cos(p1[0]*M_PI/180.0);

    p2[0]=rad_earth_p2*sin(p2[0]*M_PI/180.0)*cos(p2[1]*M_PI/180.0);
    p2[1]=rad_earth_p2*sin(p2[0]*M_PI/180.0)*sin(p2[1]*M_PI/180.0);
    p2[2]=rad_earth_p2*cos(p2[0]*M_PI/180.0);

    //Calc vector to last point
    p12[0]=p1[0]-p2[0];
    p12[1]=p1[1]-p2[1];
    p12[2]=p1[2]-p2[2];

    //Norm vector pointing to last position -> offset direction
    magnitude= sqrt(pow(p12[0],2)+pow(p12[1],2)+pow((p12[2]),2));

    pnorm[0]= p12[0]/magnitude;
    pnorm[1]= p12[1]/magnitude;
    pnorm[2]= p12[2]/magnitude;


    //Rotate to follow from left or right
    rot_matrix_yaw[0][0]=cos(offset_angle);    rot_matrix_yaw[0][1]=-sin(offset_angle);    rot_matrix_yaw[0][2]=0;
    rot_matrix_yaw[1][0]=sin(offset_angle);    rot_matrix_yaw[1][1]=cos(offset_angle);     rot_matrix_yaw[1][2]=0;

    psoll[0]=pnorm[0]*rot_matrix_yaw[0][0]    +pnorm[1]*rot_matrix_yaw[0][1]   +pnorm[2]*rot_matrix_yaw[0][2];
    psoll[1]=pnorm[0]*rot_matrix_yaw[1][0]    +pnorm[1]*rot_matrix_yaw[1][1]   +pnorm[2]*rot_matrix_yaw[1][2];

    //Scale vector
    psoll[0]*=offset_dist;
    psoll[1]*=offset_dist;
    psoll[2]*=offset_dist;

    magnitude= sqrt(pow(psoll[0],2)+pow(psoll[1],2)+pow((psoll[2]),2));
    printf("M: %f\n",magnitude);


    //Calc position
    psoll[0]=p2[0]+psoll[0];
    psoll[1]=p2[1]+psoll[1];
    psoll[2]=p2[2]+psoll[2];



    //Transform back to spherical coordinates
    psoll[0]= acos(psoll[2]/rad_earth_p2)*180.0/M_PI;
    psoll[1]=asin(psoll[1]/(sin(psoll[0]*M_PI/180.0)*rad_earth_p2))*180.0/M_PI;
    psoll[2]=alt+offset_height;

    for(int i=0;i<3;i++)
    {
        printf("%f\n", psoll[i]);
    }

//    psoll[1]= atan(psoll[0]/psoll[1]);
//    psoll[0]= asin(psoll[0]/(rad_earth_soll*cos(psoll[1])));
//    psoll[2]=psoll[2]/(rad_earth_soll*cos());

//    psoll[0]=psoll[0]*180.0/M_PI;
//    psoll[1]=psoll[1]*180.0/M_PI;



    //new z axis
//    new_z[0]=psoll[1]*p2[2]-psoll[2]*p2[1];
//    new_z[1]=psoll[2]*p2[0]-psoll[0]*p2[2];
//    new_z[2]=psoll[0]*p2[1]-psoll[1]*p2[0];

//    magnitude= sqrt(pow(new_z[0],2)+pow(new_z[1],2)+pow((new_z[2]),2));

//    new_z[0]/=magnitude;
//    new_z[1]/=magnitude;
//    new_z[2]/=magnitude;

//    rot_to_dist=2*asin(offset_dist/(2*rad_earth));

//    rot_matrix_new_z[0][0]=pow(new_z[0],2)*(1-cos(rot_to_dist))+cos(rot_to_dist);               rot_matrix_new_z[0][1]=new_z[0]*new_z[1]*(1-cos(rot_to_dist))-new_z[2]*sin(rot_to_dist);    rot_matrix_new_z[0][2]=new_z[0]*new_z[2]*(1-cos(rot_to_dist))+new_z[1]*sin(rot_to_dist);
//    rot_matrix_new_z[1][0]=new_z[0]*new_z[1]*(1-cos(rot_to_dist))+new_z[2]*sin(rot_to_dist);    rot_matrix_new_z[1][1]=pow(new_z[1],2)*(1-cos(rot_to_dist))+cos(rot_to_dist);               rot_matrix_new_z[1][2]=new_z[1]*new_z[2]*(1-cos(rot_to_dist))-new_z[0]*sin(rot_to_dist);
//    rot_matrix_new_z[2][0]=new_z[2]*new_z[0]*(1-cos(rot_to_dist))-new_z[1]*sin(rot_to_dist);    rot_matrix_new_z[2][1]=new_z[2]*new_z[1]*(1-cos(rot_to_dist))+new_z[0]*sin(rot_to_dist);    rot_matrix_new_z[2][2]=pow(new_z[2],2)*(1-cos(rot_to_dist))+cos(rot_to_dist);


//    psoll[0]=p2[0]*rot_matrix_new_z[0][0]    +p2[1]*rot_matrix_new_z[0][1]   +p2[2]*rot_matrix_new_z[0][2];
//    psoll[1]=p2[0]*rot_matrix_new_z[1][0]    +p2[1]*rot_matrix_new_z[1][1]   +p2[2]*rot_matrix_new_z[1][2];
//    psoll[2]=p2[2]+offset_height;



    return 0;

}
/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

        fprintf(stderr, "usage: offsetCalcStart {start offset_dist offset_heigt offset_angle|stop|status|reset offset_dist offset_heigt offset_angle}\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int offsetCalcStart_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
                        printf("eoffsetCalc already running\n");
			/* this is not an error */
                        return 0;
        }else{
            if(argv[2]!=NULL && argv[3]!=NULL && argv[2]!=NULL){
                offset_dist= std::stod(argv[2],NULL);
               offset_height=std::stod(argv[3],NULL);
               offset_angle=std::stod(argv[4],NULL);
            }else{
                usage("missing parameters");
                return 0;
            }

        }

		thread_should_exit = false;
                deamon_task = px4_task_spawn_cmd("offsetCalc",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
                                                 offsetCalc,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		thread_running = true;
                return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
                return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
                        printf("\toffsetCalc is running\n");

		} else {
                        printf("\toffsetCalc not started\n");
		}

                return 0;
	}

    if (!strcmp(argv[1], "reset")) {

        if(thread_running){
            if(argv[2]!=NULL && argv[3]!=NULL && argv[4]!=NULL){
                offset_dist= std::stod(argv[2],NULL);
               offset_height=std::stod(argv[3],NULL);
               offset_angle=std::stod(argv[4],NULL);
               thread_should_exit = false;
               deamon_task = px4_task_spawn_cmd("offsetCalc",
                                                SCHED_DEFAULT,
                                                SCHED_PRIORITY_MAX - 20,
                                                2048,
                                                offsetCalc,
                                                (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
               thread_running = true;
               return 0;
            }else{
                usage("missing parameters");
                return 0;
            }
        }else{
            usage("app is not running");
            return 0;
        }

        }

	usage("unrecognized command");
        return 1;
}
