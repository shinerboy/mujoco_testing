/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

//#include <iostream>
#include<stdbool.h> //for bool
#include<unistd.h> //for usleep
#include <time.h> 
#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
//#include <Eigen/Dense>
//#include "../../../eigen-master/Eigen/Eigen"
//#include "../../../eigen-master/Eigen/Dense"
//#include "../../../eigen-master/Eigen/Core"
//extern "C"{
#include "../../../lowlevelapi_test/libtest.h"
#include "../../../lowlevelapi_test/matram.h"
#include "../../../lowlevelapi_test/controller.h"
#include "../../../lowlevelapi_test/traj.h"
#include "../../../lowlevelapi_test/get_params.h"

//#include "./../../lowlevelapi_test/pbplots/pbPlots.h"
//#include "./../../lowlevelapi_test/pbplots/supportLib.h"
//}

//using namespace std;
//using namespace Eigen::MatrixXd;

time_t          s;  // Seconds
  long time_start_l;
  struct timespec spec;
  double time_start;
  double tf = 1.0;
  double s0[1][2] = {0, 0};
  double sf[1][2] = {0, 0};
  double v0[1][2] = {0,0};
  double vf[1][2] = {0,0};
  double a0[1][2]= {0,0};
  double af[1][2] = {0,0};
  double ss[1][2]={0};
  double vv[1][2]={0}; 
  double aa[1][2]={0};
  double msd=0;
  double *z;
  double *params;
  
  double *uu;
  int counter=0; //Counter used for finite state machine/state transitions
  int plotcounter = 5; //Used to plot only during one cycle
  int plotbreak=0;

  //////////////////////
  double theta1;
  double theta2;
  double omega1;
  double omega2; 
  ///////////////////////

  
  //double traj_des[4]={0};
  double *traj_des;
  

  double step =0;
  int j = 0;


  //clock_gettime(CLOCK_REALTIME, &spec);

  //time_start_l  = (spec.tv_sec)*1000+(spec.tv_nsec)/1.0e6;
  //time_start = (double)time_start_l;
  double setup_time = 3.00;
  long ms;

  

  double edt[2][1]={{0},{0}};
  double timestep[1][1]={0.0001};

  double *xx;//for plotting 
    double *yy1;
    double *yy2;
    double *yy3;
    double *yy4;
    double *yt1;
    double *yt2;
    double *ya1; //to plot acceleration trajectory
    double *ya2;
    double *yg1;
    double *yg2;




// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// control loop callback
void mycontroller(const mjModel* m, mjData* d)
{
    
    // double *xx = calloc(1, sizeof(double));//for plotting 
    // double *yy1 = calloc(1, sizeof(double));
    // double *yy2 = calloc(1, sizeof(double));
    // double *yy3 = calloc(1, sizeof(double));
    // double *yy4 = calloc(1, sizeof(double));
    // double *yt1 = calloc(1, sizeof(double));
    // double *yt2 = calloc(1, sizeof(double));
    // double *ya1 = calloc(1, sizeof(double)); //to plot acceleration trajectory
    // double *ya2 = calloc(1, sizeof(double));

    int a =7;
    int b =5;
    int c;
    sumtest(a, b, &c);
    printf("%i\n", c);
    
    double M[2][2]={0};
    M[0][0]=1;
    M[0][1]=2;
    M[1][0]=3;
    M[1][1]=4;


    double Minv[2][2]={0};

  matInv2(2, M, Minv);
  matPrint(2,2,Minv);

     // MatrixXd m(2,2);
  //m(0,0) = 3;
  //m(1,0) = 2.5;
  //m(0,1) = -1;
 // m(1,1) = m(1,0) + m(0,1);
  //std::cout << m << std::endl;

    //Matrix <float, 3, 3,> matrixA;
    //matrixA.setZero();
    //cout << matrixA << endl;


    // printouts for debugging purposes
    //std::cout << "number of position coordinates: " << m->nq << std::endl;
    //std::cout << "number of degrees of freedom: " << m->nv << std::endl;
    //std::cout << "number of joints: " << m->njnt << std::endl;
    //std::cout << "joint position: " << d->qpos[0] << std::endl;
    //std::cout << "joint velocity: " << d->qvel[0] << std::endl;
    //std::cout << "Sensor output: " << d->sensordata[0] << std::endl;

    //printf("%d \n",m->nq);
    //printf("%f \n",d->qpos[17]);
    const char* L_elbow_joint_name = "left-elbow";
    int L_elbow_sensorID = mj_name2id(m, mjOBJ_SENSOR, L_elbow_joint_name);
    int L_elbow_sensor_adr = m->sensor_adr[L_elbow_sensorID];
    int L_elbow_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, L_elbow_joint_name);
    int L_elbow_jointID = mj_name2id(m, mjOBJ_JOINT, L_elbow_joint_name);
    int L_elbow_joint_adr = m->jnt_qposadr[L_elbow_jointID];
    int L_elbow_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-elbow-vel");
    int L_elbow_vel_sensor_adr = m->sensor_adr[L_elbow_vel_sensorID];
    double L_elbow_ctrl_limit = m->actuator_ctrlrange[L_elbow_actuatorID];
    // Left Hip Roll
    const char* LHR_joint_name = "left-hip-roll";
    int LHR_sensorID = mj_name2id(m, mjOBJ_SENSOR, LHR_joint_name);
    int LHR_sensor_adr = m->sensor_adr[LHR_sensorID];
    int LHR_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LHR_joint_name);
    int LHR_jointID = mj_name2id(m, mjOBJ_JOINT, LHR_joint_name);
    int LHR_joint_adr = m->jnt_qposadr[LHR_jointID];
    int LHR_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-hip-roll-vel");
    int LHR_vel_sensor_adr = m->sensor_adr[LHR_vel_sensorID];
    double LHR_ctrl_limit = m->actuator_ctrlrange[LHR_actuatorID];
    // Left Hip yAW
    const char* LHY_joint_name = "left-hip-yaw";
    int LHY_sensorID = mj_name2id(m, mjOBJ_SENSOR, LHY_joint_name);
    int LHY_sensor_adr = m->sensor_adr[LHY_sensorID];
    int LHY_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LHY_joint_name);
    int LHY_jointID = mj_name2id(m, mjOBJ_JOINT, LHY_joint_name);
    int LHY_joint_adr = m->jnt_qposadr[LHY_jointID];
    int LHY_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-hip-yaw-vel");
    int LHY_vel_sensor_adr = m->sensor_adr[LHY_vel_sensorID];
    double LHY_ctrl_limit = m->actuator_ctrlrange[LHY_actuatorID];
    // Left Hip Pitch
    const char* LHP_joint_name = "left-hip-pitch";
    int LHP_sensorID = mj_name2id(m, mjOBJ_SENSOR, LHP_joint_name);
    int LHP_sensor_adr = m->sensor_adr[LHP_sensorID];
    int LHP_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LHP_joint_name);
    int LHP_jointID = mj_name2id(m, mjOBJ_JOINT, LHP_joint_name);
    int LHP_joint_adr = m->jnt_qposadr[LHP_jointID];
    int LHP_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-hip-pitch-vel");
    int LHP_vel_sensor_adr = m->sensor_adr[LHP_vel_sensorID];
    double LHP_ctrl_limit = m->actuator_ctrlrange[LHP_actuatorID];
    // Left Knee
    const char* LK_joint_name = "left-knee";
    int LK_sensorID = mj_name2id(m, mjOBJ_SENSOR, LK_joint_name);
    int LK_sensor_adr = m->sensor_adr[LK_sensorID];
    int LK_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LK_joint_name);
    int LK_jointID = mj_name2id(m, mjOBJ_JOINT, LK_joint_name);
    int LK_joint_adr = m->jnt_qposadr[LK_jointID];
    int LK_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-knee-sensor");
    int LK_vel_sensor_adr = m->sensor_adr[LK_vel_sensorID];
    double LK_ctrl_limit = m->actuator_ctrlrange[LK_actuatorID];
    // Left Shoulder Roll
    const char* LSR_joint_name = "left-shoulder-roll";
    int LSR_sensorID = mj_name2id(m, mjOBJ_SENSOR, LSR_joint_name);
    int LSR_sensor_adr = m->sensor_adr[LSR_sensorID];
    int LSR_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LSR_joint_name);
    int LSR_jointID = mj_name2id(m, mjOBJ_JOINT, LSR_joint_name);
    int LSR_joint_adr = m->jnt_qposadr[LSR_jointID];
    int LSR_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-shoulder-roll-vel");
    int LSR_vel_sensor_adr = m->sensor_adr[LSR_vel_sensorID];
    double LSR_ctrl_limit = m->actuator_ctrlrange[LSR_actuatorID];
    // Left Shoulder yAW
    const char* LSY_joint_name = "left-shoulder-yaw";
    int LSY_sensorID = mj_name2id(m, mjOBJ_SENSOR, LSY_joint_name);
    int LSY_sensor_adr = m->sensor_adr[LSY_sensorID];
    int LSY_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LSY_joint_name);
    int LSY_jointID = mj_name2id(m, mjOBJ_JOINT, LSY_joint_name);
    int LSY_joint_adr = m->jnt_qposadr[LSY_jointID];
    int LSY_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-shoulder-yaw-vel");
    int LSY_vel_sensor_adr = m->sensor_adr[LSY_vel_sensorID];
    double LSY_ctrl_limit = m->actuator_ctrlrange[LSY_actuatorID];
    // Left Shoulder Pitch
    const char* LSP_joint_name = "left-shoulder-pitch";
    int LSP_sensorID = mj_name2id(m, mjOBJ_SENSOR, LSP_joint_name);
    int LSP_sensor_adr = m->sensor_adr[LSP_sensorID];
    int LSP_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LSP_joint_name);
    int LSP_jointID = mj_name2id(m, mjOBJ_JOINT, LSP_joint_name);
    int LSP_joint_adr = m->jnt_qposadr[LSP_jointID];
    int LSP_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-shoulder-pitch-vel");
    int LSP_vel_sensor_adr = m->sensor_adr[LSP_vel_sensorID];
    double LSP_ctrl_limit = m->actuator_ctrlrange[LSP_actuatorID];
    // Right Hip Roll
    const char* RHR_joint_name = "right-hip-roll";
    int RHR_sensorID = mj_name2id(m, mjOBJ_SENSOR, RHR_joint_name);
    int RHR_sensor_adr = m->sensor_adr[RHR_sensorID];
    int RHR_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RHR_joint_name);
    int RHR_jointID = mj_name2id(m, mjOBJ_JOINT, RHR_joint_name);
    int RHR_joint_adr = m->jnt_qposadr[RHR_jointID];
    int RHR_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-hip-roll-vel");
    int RHR_vel_sensor_adr = m->sensor_adr[RHR_vel_sensorID];
    double RHR_ctrl_limit = m->actuator_ctrlrange[RHR_actuatorID];
    // Right Hip yAW
    const char* RHY_joint_name = "right-hip-yaw";
    int RHY_sensorID = mj_name2id(m, mjOBJ_SENSOR, RHY_joint_name);
    int RHY_sensor_adr = m->sensor_adr[RHY_sensorID];
    int RHY_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RHY_joint_name);
    int RHY_jointID = mj_name2id(m, mjOBJ_JOINT, RHY_joint_name);
    int RHY_joint_adr = m->jnt_qposadr[RHY_jointID];
    int RHY_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-hip-yaw-vel");
    int RHY_vel_sensor_adr = m->sensor_adr[RHY_vel_sensorID];
    double RHY_ctrl_limit = m->actuator_ctrlrange[RHY_actuatorID];
    // Right Hip Pitch
    const char* RHP_joint_name = "right-hip-pitch";
    int RHP_sensorID = mj_name2id(m, mjOBJ_SENSOR, RHP_joint_name);
    int RHP_sensor_adr = m->sensor_adr[RHP_sensorID];
    int RHP_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RHP_joint_name);
    int RHP_jointID = mj_name2id(m, mjOBJ_JOINT, RHP_joint_name);
    int RHP_joint_adr = m->jnt_qposadr[RHP_jointID];
    int RHP_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-hip-pitch-vel");
    int RHP_vel_sensor_adr = m->sensor_adr[RHP_vel_sensorID];
    double RHP_ctrl_limit = m->actuator_ctrlrange[RHP_actuatorID];
    // Right Knee
    const char* RK_joint_name = "right-knee";
    int RK_sensorID = mj_name2id(m, mjOBJ_SENSOR, RK_joint_name);
    int RK_sensor_adr = m->sensor_adr[RK_sensorID];
    int RK_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RK_joint_name);
    int RK_jointID = mj_name2id(m, mjOBJ_JOINT, RK_joint_name);
    int RK_joint_adr = m->jnt_qposadr[RK_jointID];
    int RK_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-knee-sensor");
    int RK_vel_sensor_adr = m->sensor_adr[RK_vel_sensorID];
    double RK_ctrl_limit = m->actuator_ctrlrange[RK_actuatorID];
    // Right Shoulder Roll
    const char* RSR_joint_name = "right-shoulder-roll";
    int RSR_sensorID = mj_name2id(m, mjOBJ_SENSOR, RSR_joint_name);
    int RSR_sensor_adr = m->sensor_adr[RSR_sensorID];
    int RSR_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RSR_joint_name);
    int RSR_jointID = mj_name2id(m, mjOBJ_JOINT, RSR_joint_name);
    int RSR_joint_adr = m->jnt_qposadr[RSR_jointID];
    int RSR_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-shoulder-roll-vel");
    int RSR_vel_sensor_adr = m->sensor_adr[RSR_vel_sensorID];
    double RSR_ctrl_limit = m->actuator_ctrlrange[RSR_actuatorID];
    // right Shoulder yAW
    const char* RSY_joint_name = "right-shoulder-yaw";
    int RSY_sensorID = mj_name2id(m, mjOBJ_SENSOR, RSY_joint_name);
    int RSY_sensor_adr = m->sensor_adr[RSY_sensorID];
    int RSY_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RSY_joint_name);
    int RSY_jointID = mj_name2id(m, mjOBJ_JOINT, RSY_joint_name);
    int RSY_joint_adr = m->jnt_qposadr[RSY_jointID];
    int RSY_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-shoulder-yaw-vel");
    int RSY_vel_sensor_adr = m->sensor_adr[RSY_vel_sensorID];
    double RSY_ctrl_limit = m->actuator_ctrlrange[RSY_actuatorID];
    // Right Shoulder Pitch
    const char* RSP_joint_name = "right-shoulder-pitch";
    int RSP_sensorID = mj_name2id(m, mjOBJ_SENSOR, RSP_joint_name);
    int RSP_sensor_adr = m->sensor_adr[RSP_sensorID];
    int RSP_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RSP_joint_name);
    int RSP_jointID = mj_name2id(m, mjOBJ_JOINT, RSP_joint_name);
    int RSP_joint_adr = m->jnt_qposadr[RSP_jointID];
    int RSP_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-shoulder-pitch-vel");
    int RSP_vel_sensor_adr = m->sensor_adr[RSP_vel_sensorID];
    double RSP_ctrl_limit = m->actuator_ctrlrange[RSP_actuatorID];
    // rIGHT ELBOW
    const char* R_elbow_joint_name = "right-elbow";
    int R_elbow_sensorID = mj_name2id(m, mjOBJ_SENSOR, R_elbow_joint_name);
    int R_elbow_sensor_adr = m->sensor_adr[R_elbow_sensorID];
    int R_elbow_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, R_elbow_joint_name);
    int R_elbow_jointID = mj_name2id(m, mjOBJ_JOINT, R_elbow_joint_name);
    int R_elbow_joint_adr = m->jnt_qposadr[R_elbow_jointID];
    int R_elbow_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-elbow-vel");
    int R_elbow_vel_sensor_adr = m->sensor_adr[R_elbow_vel_sensorID];
    double R_elbow_ctrl_limit = m->actuator_ctrlrange[R_elbow_actuatorID];

    //int j = 26;
    //double ctrl = -100*(d->qpos[26]-0.75);
    //double ctrl = -100*(d->qpos[L_elbow_joint_adr]-0.75);
    double L_elbow_ctrl = -1*(d->sensordata[L_elbow_sensor_adr]+0.0)-(1*d->sensordata[L_elbow_vel_sensor_adr]);
    double LHR_ctrl = -10*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(1*d->sensordata[LHR_vel_sensor_adr]);
    double LHY_ctrl = -10*(d->sensordata[LHY_sensor_adr]+0.0)-(1*d->sensordata[LHY_vel_sensor_adr]);
    //double LHP_ctrl = -150*(d->sensordata[LHP_sensor_adr]+0.0);
    double LSR_ctrl = -1*(d->sensordata[LSR_sensor_adr]+0.0)-(1*d->sensordata[LSR_vel_sensor_adr]);
    double LSY_ctrl = -1*(d->sensordata[LSY_sensor_adr]+0.0)-(1*d->sensordata[LSY_vel_sensor_adr]);
    double LSP_ctrl = -1*(d->sensordata[LSP_sensor_adr]+0.0)-(1*d->sensordata[LSP_vel_sensor_adr]);
    double RHR_ctrl = -10*(d->sensordata[RHR_sensor_adr]+0.0)-(1*d->sensordata[RHR_vel_sensor_adr]);
    double RHY_ctrl = -10*(d->sensordata[RHY_sensor_adr]+0.0)-(1*d->sensordata[RHY_vel_sensor_adr]);
    double RHP_ctrl = -10*(d->sensordata[RHP_sensor_adr]+0.0)-(1*d->sensordata[RHP_vel_sensor_adr]);
    double RK_ctrl = -10*(d->sensordata[RK_sensor_adr]+0.0)-(1*d->sensordata[RK_vel_sensor_adr]);
    double RSR_ctrl = -1*(d->sensordata[RSR_sensor_adr]+0.0)-(1*d->sensordata[RSR_vel_sensor_adr]);
    double RSY_ctrl = -1*(d->sensordata[RSY_sensor_adr]+0.0)-(1*d->sensordata[RSY_vel_sensor_adr]);
    double RSP_ctrl = -1*(d->sensordata[RSP_sensor_adr]+0.0)-(1*d->sensordata[RSP_vel_sensor_adr]);
    double R_elbow_ctrl = -1*(d->sensordata[R_elbow_sensor_adr]+0.0)-(1*d->sensordata[R_elbow_vel_sensor_adr]);
    //double ctrl = -100*(d->sensordata[16]-0.5);
    //printf("%f %f %f\n",d->qpos[j],d->sensordata[16],d->sensordata[43]);
    //d->ctrl[15] = ctrl;
    d->ctrl[L_elbow_actuatorID] = L_elbow_ctrl;
    d->ctrl[LHY_actuatorID] = LHY_ctrl;
    d->ctrl[LHR_actuatorID] = LHR_ctrl;
    //d->ctrl[LHP_actuatorID] = LHP_ctrl;
    d->ctrl[LSY_actuatorID] = LSY_ctrl;
    d->ctrl[LSR_actuatorID] = LSR_ctrl;
    d->ctrl[LSP_actuatorID] = LSP_ctrl;
    d->ctrl[RHY_actuatorID] = RHY_ctrl;
    d->ctrl[RHR_actuatorID] = RHR_ctrl;
    d->ctrl[RHP_actuatorID] = RHP_ctrl;
    d->ctrl[RK_actuatorID] = RK_ctrl;
    d->ctrl[RSR_actuatorID] = RSR_ctrl;
    d->ctrl[RSP_actuatorID] = RSP_ctrl;
    d->ctrl[RSY_actuatorID] = RSY_ctrl;
    d->ctrl[R_elbow_actuatorID] = R_elbow_ctrl;

    printf("position = %f \n",d->qpos[L_elbow_joint_adr]);
    //for (int i=0;i<=22;i++)
      printf("%f \n",d->sensordata[L_elbow_sensor_adr]);

      clock_gettime(CLOCK_REALTIME, &spec);
      ms = (spec.tv_sec)*1000+(spec.tv_nsec)/1.0e6 - time_start;
      msd = (double)ms/1000 - setup_time;


    printf("The knee motor position is " );
    printf("%lf", d->sensordata[LK_sensor_adr]);
    printf("\n");

    printf("The hip pitch joint position is " );
    printf("%lf", d->sensordata[LHP_sensor_adr]);
    printf("\n");

    printf("The hip pitch joint gravity torque is " );
    printf("%lf", d->qfrc_bias[LHP_joint_adr]);
    printf("\n");

    printf("The hip pitch joint controller torque is " );
    printf("%lf", uu[0]);
    printf("\n");

    printf("The knee joint gravity torque is " );
    printf("%lf", d->qfrc_bias[LK_joint_adr]);
    printf("\n");

    printf("The knee joint controller torque is " );
    printf("%lf", uu[1]);
    printf("\n");



    theta1 = d->sensordata[LHP_sensor_adr]+45*(M_PI/180);
    theta2 = d->sensordata[LK_sensor_adr]-82.25*(M_PI/180);
    omega1 = d->sensordata[LHP_vel_sensor_adr];
    omega2 = d->sensordata[LK_vel_sensor_adr];

    if (msd>=tf && counter ==0)
    {
      s0[0][0] = d->sensordata[LHP_sensor_adr]+45*(M_PI/180);
      s0[0][1] = d->sensordata[LK_sensor_adr]-82.25*(M_PI/180);
      v0[0][0] = d->sensordata[LHP_vel_sensor_adr];
      v0[0][1] = d->sensordata[LK_vel_sensor_adr];
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      sf[0][0] = (45+10)*(M_PI/180);
      //sf[0][0] = s0[0][0];
      sf[0][1] = (-82.25-15)*(M_PI/180);
      //sf[0][1] = s0[0][1];
      edt[0][0]=0; //accumulated error for integral control
      edt[0][1]=0;

      counter = 1;
      plotcounter=0;
      j=0;
    }
    if (msd>=2*tf && counter ==1)
    {
      s0[0][0] = d->sensordata[LHP_sensor_adr]+45*(M_PI/180);
      s0[0][1] = d->sensordata[LK_sensor_adr]-82.25*(M_PI/180);
      v0[0][0] = d->sensordata[LHP_vel_sensor_adr];
      v0[0][1] = d->sensordata[LK_vel_sensor_adr];
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      sf[0][0] = (45-25)*(M_PI/180);
      //sf[0][0] = s0[0][0];
      sf[0][1] = (-82.25+15)*(M_PI/180);
      //sf[0][1] = s0[0][1];
      edt[0][0]=0;
      edt[0][1]=0;
  

      counter = 2;
    }
    printf("%d \n", counter);


    

    if(msd<tf){

      //plotcounter=plotcounter+1;
      double LHP_ctrl = (-200*(d->sensordata[LHP_sensor_adr]+0.0*(M_PI/180))/16)-(1*omega1);
      d->ctrl[LHP_actuatorID] = LHP_ctrl;
      double LK_ctrl = (-200*(d->sensordata[LK_sensor_adr]-0*(M_PI/180))/16)-(1*omega2);
      d->ctrl[LK_actuatorID] = LK_ctrl;     


    }

    else if(msd>=tf && msd<2*tf){

        for(int i=0;i<2;i++){
        ctraj(&ss[0][i],&vv[0][i],&aa[0][i],msd,tf,tf*2, s0[0][i], sf[0][i], v0[0][i], vf[0][i], a0[0][i], af[0][i]);
      }

     // printf("%f", msd);
     // printf("\n");
     // printf("%f,%f,%f \n", ss[0][0],vv[0][0],aa[0][0]);
     // printf("%f,%f,%f \n", ss[0][1],vv[0][1],aa[0][1]);

      traj_des[0]=ss[0][0];
      traj_des[1]=ss[0][1];
      traj_des[2]=vv[0][0];
      traj_des[3]=vv[0][1];
      traj_des[4]=aa[0][0];
      traj_des[5]=aa[0][1];

      z[0]=theta1;
      z[1]=omega1;
      z[2]=theta2;
      z[3]=omega2;

      //c_controller(uu,z,params,traj_des,timestep,edt); 
      c_controller(uu,z,params,traj_des); 

      // if (uu[0]>(LK_ctrl_limit[0])){
      //  uu[0]=LK_ctrl_limit;
      // }
      // if (uu[1]>(LHP_ctrl_limit))[0]{
      //  uu[1]=LHP_ctrl_limit[0];
      // }
      // if (uu[0]<-1*(LK_ctrl_limit[0])){
      //  uu[0]=-1*(LK_ctrl_limit[0]);
      // }
      // if (uu[1]<-1*(LHP_ctrl_limit[0])){
      //  uu[1]=-1*(LHP_ctrl_limit[0]);
      // }

      //uu[0]=(-1*(d->sensordata[LHP_sensor_adr]+10.0*(M_PI/180)))-(1*omega1);
      //uu[1]= (-1*(d->sensordata[LK_sensor_adr]-20.0*(M_PI/180)))-(1*omega2);   

      d->ctrl[LHP_actuatorID] = uu[0]/16.0;
      d->ctrl[LK_actuatorID] = uu[1]/16.0;


        ///////////////////////////
  }

    else if(msd>=2*tf && msd<3*tf){
        for(int i=0;i<2;i++){
        ctraj(&ss[0][i],&vv[0][i],&aa[0][i],msd,2*tf,3*tf, s0[0][i], sf[0][i], v0[0][i], vf[0][i], a0[0][i], af[0][i]);
      }

      traj_des[0]=ss[0][0];
      traj_des[1]=ss[0][1];
      traj_des[2]=vv[0][0];
      traj_des[3]=vv[0][1];
      traj_des[4]=aa[0][0];
      traj_des[5]=aa[0][1];

      z[0]=theta1;
      z[1]=omega1;
      z[2]=theta2;
      z[3]=omega2;

      //c_controller2(uu,z,params,traj_des,timestep,edt);
      c_controller(uu,z,params,traj_des); 
      // if (uu[0]>(LK_ctrl_limit[0])){
      //  uu[0]=LK_ctrl_limit;
      // }
      // if (uu[1]>(LHP_ctrl_limit[0])){
      //  uu[1]=LHP_ctrl_limit[0];
      // }
      // if (uu[0]<-1*(LK_ctrl_limit[0])){
      //  uu[0]=-1*(LK_ctrl_limit[0]);
      // }
      // if (uu[1]<-1*(LHP_ctrl_limit[0])){
      //  uu[1]=-1*(LHP_ctrl_limit[0]);
      // }

      //uu[0]=(-1*(d->sensordata[LHP_sensor_adr]+10.0*(M_PI/180)))-(1*omega1);
      //uu[1]= (-1*(d->sensordata[LK_sensor_adr]-20.0*(M_PI/180)))-(1*omega2);   

      d->ctrl[LHP_actuatorID] = uu[0]/16.0;
      d->ctrl[LK_actuatorID] = uu[1]/16.0;
      
    }
        


      else{
        clock_gettime(CLOCK_REALTIME, &spec);

        time_start_l  = (spec.tv_sec)*1000+(spec.tv_nsec)/1.0e6;
        time_start = (double)time_start_l;
        msd=tf;
        counter=0;
        plotcounter=plotcounter+1;
        
      }


      if (plotcounter==0 && plotbreak==0){
        yy1 = realloc(yy1, (j+1)*sizeof(double));
        yy2 = realloc(yy2, (j+1)*sizeof(double));
        xx = realloc(xx, (j+1)*sizeof(double));
        xx [j]=msd;
        yy1 [j]=theta1; //actual red 
        yy2 [j]=traj_des[0]; //desired blue
        yy3 = realloc(yy3, (j+1)*sizeof(double));
        yy4 = realloc(yy4, (j+1)*sizeof(double));    
        yy3 [j]=theta2; //actual red 
        yy4 [j]=traj_des[1]; //desired blue
        yt1 = realloc(yt1, (j+1)*sizeof(double));
        yt2 = realloc(yt2, (j+1)*sizeof(double));
        yt1 [j]=uu[0]; //hip torque
        yt2 [j]=uu[1]; //knee torque
        ya1 = realloc(ya1, (j+1)*sizeof(double));
        ya1 [j]=traj_des[4];
        ya2 = realloc(ya2, (j+1)*sizeof(double));
        ya2 [j]=traj_des[5];
        yg1 = realloc(yg1, (j+1)*sizeof(double));
        yg1 [j]=d->qfrc_bias[LHP_joint_adr]*1; //LHP gravity joint torques
        yg2 = realloc(yg2, (j+1)*sizeof(double));
        yg2 [j]= d->qfrc_bias[LK_joint_adr]*1;


        
      }

      if (plotcounter==1 && plotbreak==0){
        printf("----------------------------------------");
        plotcounter=2;
        plotbreak=1;
        
        //remove("example77.png");

       

        matPlot2(xx,yy1,xx,yy2,j,"theta1vstraj_des_KP120_01.png",L"Hip Angle Position", L"time (s)",L"Position (rad)");
        matPlot2(xx,yy3,xx,yy4,j,"theta2vstraj_des_KP120_01.png",L"Knee Angle Position",L"time (s)",L"Position (rad)");
        matPlot2(xx,yt1,xx,yg1,j,"hip_torque_and_gravity_torque.png",L"Hip Control (red) Gravity Torque (blue)",L"time (s)",L"Torque (N-m)");
        matPlot2(xx,yt2,xx,yg2,j,"knee_torque_and_gravity_torque.png",L"Knee Control (red) Gravity (blue)",L"time (s)",L"Torque (N-m)");
        free(xx);
        free(yy1);
        free(yy2);
        //RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();

        //DrawScatterPlot(imageRef, 600, 400, xx, j, yy, j);

        //size_t length;
        //double *pngData = ConvertToPNG(&length, imageRef->image);
        //WriteToFile(pngData, length, "plot.png");
        
      }

      

    //cout << endl;


    /* int l =9;
    int k=0;
    target_position[l]=-0.4; //Right knee (9)
    target_position[k]=-0.8; //Left hip pitch (2)


    command.motors[l].torque =
        150.0 * (target_position[l]+step - observation.motor.position[l]);
    command.motors[l].velocity = 0.0;
    command.motors[l].damping = 0.75 * limits->damping_limit[l];

    command.motors[k].torque =
        150.0 * (target_position[k]+step - observation.motor.position[k]);
    command.motors[k].velocity = 0.0;
    command.motors[k].damping = 0.75 * limits->damping_limit[k]; */


    
    j=j+1;

}



// main function
int main(int argc, const char** argv)
{
    clock_gettime(CLOCK_REALTIME, &spec);

  time_start_l  = (spec.tv_sec)*1000+(spec.tv_nsec)/1.0e6;
  time_start = (double)time_start_l;
  params = (double*)malloc(sizeof(double)*23);
    traj_des = (double *)malloc(sizeof(double)*4);
    z = (double *)malloc(sizeof(double)*4);
    uu = (double *)malloc(sizeof(double)*2);
    get_params(params);

    xx = calloc(1, sizeof(double));//for plotting 
    yy1 = calloc(1, sizeof(double));
    yy2 = calloc(1, sizeof(double));
    yy3 = calloc(1, sizeof(double));
    yy4 = calloc(1, sizeof(double));
    yt1 = calloc(1, sizeof(double));
    yt2 = calloc(1, sizeof(double));
    ya1 = calloc(1, sizeof(double)); //to plot acceleration trajectory
    ya2 = calloc(1, sizeof(double));
    yg1 = calloc(1, sizeof(double)); //to plot joint gravity torque
    yg2 = calloc(1, sizeof(double));
  


  //// Above this line was Copied from lowlevel api
    
    //printf("%i", c);
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // activate software
    mj_activate("../../../mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    //printf("%d \n",m->nq);
    //d->qpos[26]=40*3.14/180;
    //d->qpos[30]=40*3.14/180; //this is in joint qposadr
    //d->qpos[31]=-20*3.14/180; //this is in joint qposadr
    //d->qpos[32]=-10*3.14/180; //this is in joint qposadr
    //d->qpos[33]=30*3.14/180; //this is in joint qposadr
    //mju_copy(d->qpos, m->key_qpos, m->nq*keyframenumber);
    //mj_kinematics(m,d);

    /*
    //xpos
    //std::cout << "cartesian position of body  " << d->xpos[6] << d->xpos[7] << d->xpos[8] << std::endl;
    //16 left-shoulder-roll
    //17 left-shoulder-pitch
    //18 left-shoulder-yaw
    //19 left-elbow
    //int index = [16, 17, 18, 19];
    for (int i=16;i<=19;i++)
    {
    printf("********** %d ********* \n",i);
    //int i = 19;
    //mjtNum*   xpos;                 // Cartesian position of body frame         (nbody x 3)
    //mjtNum*   xquat;                // Cartesian orientation of body frame      (nbody x 4)
    //mjtNum*   xmat;                 // Cartesian orientation of body frame      (nbody x 9)
    //mjtNum*   xipos;                // Cartesian position of body com           (nbody x 3)
    //mjtNum*   ximat;                // Cartesian orientation of body inertia    (nbody x 9)
    printf("body = %d \n",i);
    printf("Cartesian position of body frame (xpos) = %1.3f \t %1.3f \t %1.3f \n",d->xpos[3*i],d->xpos[3*i+1],d->xpos[3*i+2]);
    printf("Cartesian orientation of body frame (xmat) = \n");
    printf(" \t%1.3f \t %1.3f \t %1.3f \n",d->xmat[9*i+0],d->xmat[9*i+1],d->xmat[9*i+2]);
    printf(" \t %1.3f \t %1.3f \t %1.3f \n",d->xmat[9*i+3],d->xmat[9*i+4],d->xmat[9*i+5]);
    printf(" \t %1.3f \t %1.3f \t %1.3f \n",d->xmat[9*i+6],d->xmat[9*i+7],d->xmat[9*i+8]);
    //printf("Cartesian position of body orientation (xquat) = %1.3f \t %1.3f \t %1.3f \t %1.3f \n",d->xquat[4*i],d->xquat[4*i+1],d->xquat[4*i+2],d->xquat[4*i+3]);
    printf(" \n");
    printf("Cartesian position of body com (xipos) = %1.3f \t %1.3f \t %1.3f \n",d->xipos[3*i],d->xipos[3*i+1],d->xipos[3*i+2]);
    printf("\n\n");
    }
    //printf("Cartesian orientation of body inertia (ximat) = \n");
    //printf(" \t%1.3f \t %1.3f \t %1.3f \n",d->ximat[9*i+0],d->ximat[9*i+1],d->ximat[9*i+2]);
    //printf(" \t %1.3f \t %1.3f \t %1.3f \n",d->ximat[9*i+3],d->ximat[9*i+4],d->ximat[9*i+5]);
    //printf(" \t %1.3f \t %1.3f \t %1.3f \n",d->ximat[9*i+6],d->ximat[9*i+7],d->ximat[9*i+8]);
    */


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);//1200,900
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // install control callback
    mjcb_control = mycontroller;


    //set camera distance
    cam.azimuth = -160;
    cam.elevation = -22.8;
    cam.distance = 4;
    cam.lookat[0] = -0.18;
    cam.lookat[1] = 0.2;
    cam.lookat[2] = 0.48;
    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        //printf("%f \n",d->time);

        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);


        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

        // move the animation to get updated values.
        //printf("%f %f %f\n",cam.azimuth,cam.elevation, cam.distance);
        //printf("%f %f %f \n",cam.lookat[0],cam.lookat[1],cam.lookat[2]);

    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}