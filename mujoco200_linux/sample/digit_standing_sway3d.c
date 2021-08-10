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
#include "../../../lowlevelapi_test/controller_walk.h"
#include "../../../lowlevelapi_test/traj.h"
#include "../../../lowlevelapi_test/get_params_walk.h"
#include "../../../lowlevelapi_test/parallel_toe.h"
#include "../../../lowlevelapi_test/getcom.h"
#include "../../../lowlevelapi_test/getphaseangles.h"

//#include "./../../lowlevelapi_test/pbplots/pbPlots.h"
//#include "./../../lowlevelapi_test/pbplots/supportLib.h"
//}

//using namespace std;
//using namespace Eigen::MatrixXd;

time_t          s;  // Seconds
long time_start_l;
struct timespec spec;
double time_start;
double tf = 8;
double traj_time = 4;//0.7 works well
double traj_time2 = 0.25;
double stage_time=0;
double s0[1][5] = {0, 0, 0, 0, 0};
double sf[1][5] = {0, 0, 0, 0, 0};
double v0[1][5] = {0, 0, 0, 0, 0};
double vf[1][5] = {0, 0, 0, 0, 0};
double a0[1][5] = {0, 0, 0, 0, 0};
double af[1][5] = {0, 0, 0, 0, 0};
double ss[1][5] = {0, 0, 0, 0, 0};
double vv[1][5] = {0, 0, 0, 0, 0};
double aa[1][5] = {0, 0, 0, 0, 0};
double s0r[1][2] = {0, 0};//Initial position of hip roll
double sfr[1][2] = {0, 0};
double v0r[1][2] = {0, 0};
double vfr[1][2] = {0, 0};
double a0r[1][2] = {0, 0};
double afr[1][2] = {0, 0};
double ssr[1][2] = {0, 0};
double vvr[1][2] = {0, 0};
double aar[1][2] = {0, 0};
double msd=0;
double cont_time = 0;
double *z;
double *params;
double x_com;
double y_com;
double x_Bf; //x position of base frame
double y_Bf;//y position of base frame
double LToe_x;
double LToe_y;
double RToe_x;
double RToe_y;
double stanceToe_x;
double LtoeA_pitch;
double LtoeB_pitch;
double RtoeA_pitch;
double RtoeB_pitch;
double LHP_x;
double LHP_y;
double RHP_x;
double RHP_y;
double LHS_defl; //Left Heel Spring deflection
double RHS_defl;
double LShin_defl;
double RShin_defl;
double axis[3]={0}; //Used for base quaternion
double sin_half_angle;//Half angle of quaternion rotation
double quat_angle;//Angle of axis rotation used for base quaternion
double base_quat[4]={0};//Quaternion values [w,x,y,z]
double base_angvel[3]={0};


double *uu;
  int counter=-1; //Counter used for finite state machine/state transitions
  int plotcounter = 50; //Used to plot only during one cycle
  int plotbreak=0;
  int stage=0; //Used to keep track of state machine stage (4 stages)

  //////////////////////
  double theta0;
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;
  double theta6;
  double omega0;
  double omega1;
  double omega2; 
  double omega3;
  double omega4; 
  double omega5;
  double omega6; 
  double theta1_mo = +45*(M_PI/180); //Add this to sensor reading to get angle in model frame. Subtract to desired model angle to get mujoco angle
  double theta2_mo = -82.25*(M_PI/180); //Add this to sensor reading to get angle in model frame. Subtract to desired model angle to get mujoco angle
  double theta0_des;
  double theta1_des;
  double theta2_des;
  double theta3_des;
  double theta4_des;
  double omega0_des;
  double omega1_des;
  double omega2_des;
  double omega3_des;
  double omega4_des;
  double torso_des;
  double LHP_des;
  double RHP_des;
  double LK_des;
  double RK_des;
  //Used for get phase angle method
  double L; //desired leg length.
  double step_angle; //desired step angle

  double mid1_angle1; //Midstance Hip Pitch of stance leg
  double mid1_angle2; //Midstance Knee of stance leg
  double fs1_angle1; //Foot strike Hip pitch of trailing leg
  double fs1_angle2; //Foot strike Knee of trailing leg
  double mid2_angle1; //Midstance Hip Pitch of swing leg
  double mid2_angle2; //Midstance knee of swing leg
  double fs2_angle1;  //Foot strike Hip Pitch of leading leg
  double fs2_angle2;  //Foot strike knee of leading leg
  double ground_z;


  // double mid1_angle1 = 0.388961234296406; //Midstance Hip Pitch of stance leg
  // double mid1_angle2 = -0.911380565589932; //Midstance Knee of stance leg
  // double fs1_angle1 = 0.179521724057087; //Foot strike Hip pitch of trailing leg
  // double fs1_angle2 = -0.911380565589932; //Foot strike Knee of trailing leg
  // double mid2_angle1 = 0.555804955644691; //Midstance Hip Pitch of swing leg
  // double mid2_angle2 = -1.259523521544439; //Midstance knee of swing leg
  // double fs2_angle1 = 0.598400744535726;  //Foot strike Hip Pitch of leading leg
  // double fs2_angle2 = -0.911380565589932;  //Foot strike knee of leading leg

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

double (*Data_csv)[11];//Array to store data to save




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
    //Base
    const char* base_pos_name = "base-pos";
    int base_sensorID = mj_name2id(m, mjOBJ_SENSOR, base_pos_name);
    int base_sensor_adr = m->sensor_adr[base_sensorID];
    const char* base_quat_name = "base-quat";
    int base_quat_sensorID = mj_name2id(m, mjOBJ_SENSOR, base_quat_name);
    int base_quat_sensor_adr = m->sensor_adr[base_quat_sensorID];

    
    
    int base_angvel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "base-angvel");
    int base_angvel_sensor_adr = m->sensor_adr[base_angvel_sensorID];
    //

    //Left elbow
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
    printf("the LHR joint address is: ");
    printf("%d \n",LHR_joint_adr); //3
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
    printf("the LHP joint address is: ");
    printf("%d \n",LHP_joint_adr); //5
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
    printf("the LK joint address is: ");
    printf("%d \n",LK_joint_adr); //10
    int LK_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-knee-vel");
    int LK_vel_sensor_adr = m->sensor_adr[LK_vel_sensorID];
    double LK_ctrl_limit = m->actuator_ctrlrange[LK_actuatorID];
    int LK_servoID = mj_name2id(m, mjOBJ_ACTUATOR, "left-knee-servo");
    printf("the LK servo ID is: ");
    printf("%d \n",LK_servoID); //4
    printf("The LKnee position is: ");
    printf("%f \n", d->sensordata[LK_sensor_adr]);
    // Left toe A
    const char* LTA_joint_name = "left-toe-A";
    int LTA_sensorID = mj_name2id(m, mjOBJ_SENSOR, LTA_joint_name);
    int LTA_sensor_adr = m->sensor_adr[LTA_sensorID];
    int LTA_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LTA_joint_name);
    int LTA_jointID = mj_name2id(m, mjOBJ_JOINT, LTA_joint_name);
    int LTA_joint_adr = m->jnt_qposadr[LTA_jointID];
    int LTA_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-toe-A-vel");
    int LTA_vel_sensor_adr = m->sensor_adr[LTA_vel_sensorID];
    double LTA_ctrl_limit = m->actuator_ctrlrange[LTA_actuatorID];
    // Left toe B
    const char* LTB_joint_name = "left-toe-B";
    int LTB_sensorID = mj_name2id(m, mjOBJ_SENSOR, LTB_joint_name);
    int LTB_sensor_adr = m->sensor_adr[LTB_sensorID];
    int LTB_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, LTB_joint_name);
    int LTB_jointID = mj_name2id(m, mjOBJ_JOINT, LTB_joint_name);
    int LTB_joint_adr = m->jnt_qposadr[LTB_jointID];
    int LTB_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "left-toe-B-vel");
    int LTB_vel_sensor_adr = m->sensor_adr[LTB_vel_sensorID];
    double LTB_ctrl_limit = m->actuator_ctrlrange[LTB_actuatorID];
    //Left toe pitch joint sensor
    const char* LToe_pitch_joint_name = "left-toe-pitch";
    int LToe_pitch_sensorID = mj_name2id(m, mjOBJ_SENSOR, LToe_pitch_joint_name);
    int LToe_pitch_sensor_adr = m->sensor_adr[LToe_pitch_sensorID];
    //Left heel spring joint sensor
    const char* LHeel_spring_joint_name = "left-heel-spring";
    int LHeel_spring_sensorID = mj_name2id(m, mjOBJ_SENSOR, LHeel_spring_joint_name);
    int LHeel_spring_sensor_adr = m->sensor_adr[LHeel_spring_sensorID];
    //Left shin spring joint sensor
    const char* LShin_spring_joint_name = "left-shin";
    int LShin_spring_sensorID = mj_name2id(m, mjOBJ_SENSOR, LShin_spring_joint_name);
    int LShin_spring_sensor_adr = m->sensor_adr[LShin_spring_sensorID];
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
    printf("the RHR joint address is: ");
    printf("%d \n",RHR_joint_adr); //30
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
    printf("the RHP joint address is: ");
    printf("%d \n",RHP_joint_adr); //32
    // Right Knee
    const char* RK_joint_name = "right-knee";
    int RK_sensorID = mj_name2id(m, mjOBJ_SENSOR, RK_joint_name);
    int RK_sensor_adr = m->sensor_adr[RK_sensorID];
    int RK_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RK_joint_name);
    int RK_jointID = mj_name2id(m, mjOBJ_JOINT, RK_joint_name);
    int RK_joint_adr = m->jnt_qposadr[RK_jointID];
    int RK_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-knee-vel");
    int RK_vel_sensor_adr = m->sensor_adr[RK_vel_sensorID];
    double RK_ctrl_limit = m->actuator_ctrlrange[RK_actuatorID];
    int RK_servoID = mj_name2id(m, mjOBJ_ACTUATOR, "right-knee-servo");
    printf("the RK servo ID is: ");
    printf("%d \n",RK_servoID); //11
    // Right toe A
    const char* RTA_joint_name = "right-toe-A";
    int RTA_sensorID = mj_name2id(m, mjOBJ_SENSOR, RTA_joint_name);
    int RTA_sensor_adr = m->sensor_adr[RTA_sensorID];
    int RTA_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RTA_joint_name);
    int RTA_jointID = mj_name2id(m, mjOBJ_JOINT, RTA_joint_name);
    int RTA_joint_adr = m->jnt_qposadr[RTA_jointID];
    int RTA_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-toe-A-vel");
    int RTA_vel_sensor_adr = m->sensor_adr[RTA_vel_sensorID];
    double RTA_ctrl_limit = m->actuator_ctrlrange[RTA_actuatorID];
    // Right toe B
    const char* RTB_joint_name = "right-toe-B";
    int RTB_sensorID = mj_name2id(m, mjOBJ_SENSOR, RTB_joint_name);
    int RTB_sensor_adr = m->sensor_adr[RTB_sensorID];
    int RTB_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, RTB_joint_name);
    int RTB_jointID = mj_name2id(m, mjOBJ_JOINT, RTB_joint_name);
    int RTB_joint_adr = m->jnt_qposadr[RTB_jointID];
    int RTB_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "right-toe-B-vel");
    int RTB_vel_sensor_adr = m->sensor_adr[RTB_vel_sensorID];
    double RTB_ctrl_limit = m->actuator_ctrlrange[RTB_actuatorID];
    //Right toe pitch joint sensor
    const char* RToe_pitch_joint_name = "right-toe-pitch";
    int RToe_pitch_sensorID = mj_name2id(m, mjOBJ_SENSOR, RToe_pitch_joint_name);
    int RToe_pitch_sensor_adr = m->sensor_adr[RToe_pitch_sensorID];
    //Right heel spring joint sensor
    const char* RHeel_spring_joint_name = "right-heel-spring";
    int RHeel_spring_sensorID = mj_name2id(m, mjOBJ_SENSOR, RHeel_spring_joint_name);
    int RHeel_spring_sensor_adr = m->sensor_adr[RHeel_spring_sensorID];
    //Right shin spring joint sensor
    const char* RShin_spring_joint_name = "right-shin";
    int RShin_spring_sensorID = mj_name2id(m, mjOBJ_SENSOR, RShin_spring_joint_name);
    int RShin_spring_sensor_adr = m->sensor_adr[RShin_spring_sensorID];
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
    // Body pitch joint
    const char* body_pitch_joint_name = "body-pitch";
    int body_pitch_sensorID = mj_name2id(m, mjOBJ_SENSOR, body_pitch_joint_name);
    int body_pitch_sensor_adr = m->sensor_adr[body_pitch_sensorID];
    int body_pitch_jointID = mj_name2id(m, mjOBJ_JOINT, body_pitch_joint_name);
    int body_pitch_joint_adr = m->jnt_qposadr[body_pitch_jointID];
    int body_pitch_vel_sensorID = mj_name2id(m, mjOBJ_SENSOR, "body-pitch-vel");
    int body_pitch_vel_sensor_adr = m->sensor_adr[body_pitch_vel_sensorID];
    printf("Body pitch position = %f \n",d->qpos[body_pitch_joint_adr]);
    printf("the body pitch joint address is: ");
    printf("%d \n",body_pitch_joint_adr); //30

    const char* LTarsus_joint_name = "left-tarsus";
    int LTarsus_jointID = mj_name2id(m, mjOBJ_JOINT, LTarsus_joint_name);
    int LTarsus_joint_adr = m->jnt_qposadr[LTarsus_jointID];
    printf("the LTarsus joint address is: ");
    printf("%d \n",LTarsus_joint_adr); //
    int LTarsus_sensorID = mj_name2id(m, mjOBJ_SENSOR, LTarsus_joint_name);
    int LTarsus_sensor_adr = m->sensor_adr[LTarsus_sensorID];
    printf("The LTarsus position is: ");
    printf("%f \n", d->sensordata[LTarsus_sensor_adr]);

    const char* body_z_joint_name = "body_z";
    int body_z_jointID = mj_name2id(m, mjOBJ_JOINT, body_z_joint_name);
    int body_z_joint_adr = m->jnt_qposadr[body_z_jointID];
    int body_z_sensorID = mj_name2id(m, mjOBJ_SENSOR, body_z_joint_name);
    int body_z_sensor_adr = m->sensor_adr[body_z_sensorID];
    const char* body_x_joint_name = "body_x";
    int body_x_jointID = mj_name2id(m, mjOBJ_JOINT, body_x_joint_name);
    int body_x_joint_adr = m->jnt_qposadr[body_x_jointID];
    int body_x_sensorID = mj_name2id(m, mjOBJ_SENSOR, body_x_joint_name);
    int body_x_sensor_adr = m->sensor_adr[body_x_sensorID];

    const char* base_joint_name = "base";
        int base_jointID = mj_name2id(m, mjOBJ_JOINT, base_joint_name);
        int base_joint_adr = m->jnt_qposadr[base_jointID];
        


    const char* LToe_pos_name = "left-toe-pos";
        int LToe_sensorID = mj_name2id(m, mjOBJ_SENSOR, LToe_pos_name);
        int LToe_sensor_adr = m->sensor_adr[LToe_sensorID];
    const char* RToe_pos_name = "right-toe-pos";
        int RToe_sensorID = mj_name2id(m, mjOBJ_SENSOR, RToe_pos_name);
        int RToe_sensor_adr = m->sensor_adr[RToe_sensorID];
    const char* LHP_pos_name = "left-hip-pos";
        int LHP_pos_sensorID = mj_name2id(m, mjOBJ_SENSOR, LHP_pos_name);
        int LHP_pos_sensor_adr = m->sensor_adr[LHP_pos_sensorID];
    const char* RHP_pos_name = "right-hip-pos";
        int RHP_pos_sensorID = mj_name2id(m, mjOBJ_SENSOR, RHP_pos_name);
        int RHP_pos_sensor_adr = m->sensor_adr[RHP_pos_sensorID];
    //COM sensor
    const char* com_sensor = "com-sensor";
    int com_sensorID = mj_name2id(m, mjOBJ_SENSOR, com_sensor);
    int com_sensor_adr = m->sensor_adr[com_sensorID];
    double sensor_comx = d->sensordata[com_sensor_adr];
    printf("Sensor com x = %f \n",sensor_comx);


    //ground_z=d->sensordata[LToe_sensor_adr+2];
    ground_z=0.0645;//0.0645; //0.061728 rounded up
    LToe_x=d->sensordata[LToe_sensor_adr];
    LToe_y=d->sensordata[LToe_sensor_adr+2];
    LHP_x = d->sensordata[LHP_pos_sensor_adr];
    LHP_y = d->sensordata[LHP_pos_sensor_adr+2];

    RToe_x=d->sensordata[RToe_sensor_adr];
    RToe_y=d->sensordata[RToe_sensor_adr+2];
    RHP_x = d->sensordata[RHP_pos_sensor_adr];
    RHP_y = d->sensordata[RHP_pos_sensor_adr+2];

    LHS_defl=d->sensordata[LHeel_spring_sensor_adr]; //Left Heel Spring deflection
    RHS_defl=d->sensordata[RHeel_spring_sensor_adr];
    LShin_defl=d->sensordata[LShin_spring_sensor_adr];
    RShin_defl=d->sensordata[RShin_spring_sensor_adr];
    printf("Left shin spring deflection = %f \n",LShin_defl);
    printf("Left heel spring deflection = %f \n",LHS_defl);

    //int j = 26;
    //double ctrl = -100*(d->qpos[26]-0.75);
    //double ctrl = -100*(d->qpos[L_elbow_joint_adr]-0.75);
    double L_elbow_ctrl = -1*(d->sensordata[L_elbow_sensor_adr]+0.0)-(1*d->sensordata[L_elbow_vel_sensor_adr]);
    //double LHR_ctrl = -500*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
    double LHY_ctrl = -500*(d->sensordata[LHY_sensor_adr]+0.0)-(1*d->sensordata[LHY_vel_sensor_adr]);
    //double LHP_ctrl = -150*(d->sensordata[LHP_sensor_adr]+0.0);
    double LSR_ctrl = -1*(d->sensordata[LSR_sensor_adr]+0.0)-(1*d->sensordata[LSR_vel_sensor_adr]);
    double LSY_ctrl = -1*(d->sensordata[LSY_sensor_adr]+0.0)-(1*d->sensordata[LSY_vel_sensor_adr]);
    double LSP_ctrl = -1*(d->sensordata[LSP_sensor_adr]+0.0)-(1*d->sensordata[LSP_vel_sensor_adr]);
    double LTA_ctrl = -1*(d->sensordata[LTA_sensor_adr]+0.0)-(1*d->sensordata[LTA_vel_sensor_adr]);
    double LTB_ctrl = -1*(d->sensordata[LTB_sensor_adr]+0.0)-(1*d->sensordata[LTB_vel_sensor_adr]);
    //double RHR_ctrl = -500*(d->sensordata[RHR_sensor_adr]+20*M_PI/180)-(10*d->sensordata[RHR_vel_sensor_adr]);
    double RHY_ctrl = -500*(d->sensordata[RHY_sensor_adr]+0.0)-(1*d->sensordata[RHY_vel_sensor_adr]);
    double RHP_ctrl = -500*(d->sensordata[RHP_sensor_adr]-(-theta1_mo+mid2_angle1))-(1*d->sensordata[RHP_vel_sensor_adr]);
    double RK_ctrl = -500*(d->sensordata[RK_sensor_adr]-(theta2_mo-mid2_angle2))-(1*d->sensordata[RK_vel_sensor_adr]);
    double RSR_ctrl = -1*(d->sensordata[RSR_sensor_adr]+0.0)-(1*d->sensordata[RSR_vel_sensor_adr]);
    double RSY_ctrl = -1*(d->sensordata[RSY_sensor_adr]+0.0)-(1*d->sensordata[RSY_vel_sensor_adr]);
    double RSP_ctrl = -1*(d->sensordata[RSP_sensor_adr]+0.0)-(1*d->sensordata[RSP_vel_sensor_adr]);
    double R_elbow_ctrl = -1*(d->sensordata[R_elbow_sensor_adr]+0.0)-(1*d->sensordata[R_elbow_vel_sensor_adr]);
    double RTA_ctrl = -1*(d->sensordata[RTA_sensor_adr]+0.0)-(1*d->sensordata[RTA_vel_sensor_adr]);
    double RTB_ctrl = -1*(d->sensordata[RTB_sensor_adr]+0.0)-(1*d->sensordata[RTB_vel_sensor_adr]);

    //double ctrl = -100*(d->sensordata[16]-0.5);
    //printf("%f %f %f\n",d->qpos[j],d->sensordata[16],d->sensordata[43]);
    //d->ctrl[15] = ctrl;
    d->ctrl[L_elbow_actuatorID] = L_elbow_ctrl;
    d->ctrl[LHY_actuatorID] = LHY_ctrl;
    //d->ctrl[LHR_actuatorID] = LHR_ctrl;
    //d->ctrl[LHP_actuatorID] = LHP_ctrl;
    d->ctrl[LSY_actuatorID] = LSY_ctrl;
    d->ctrl[LSR_actuatorID] = LSR_ctrl;
    d->ctrl[LSP_actuatorID] = LSP_ctrl;
    //d->ctrl[LTA_actuatorID] = LTA_ctrl;
    //d->ctrl[LTB_actuatorID] = LTB_ctrl;
    d->ctrl[RHY_actuatorID] = RHY_ctrl;
    //d->ctrl[RHR_actuatorID] = RHR_ctrl;
    //d->ctrl[RHP_actuatorID] = RHP_ctrl;
    //d->ctrl[RK_actuatorID] = RK_ctrl;
    d->ctrl[RSR_actuatorID] = RSR_ctrl;
    d->ctrl[RSP_actuatorID] = RSP_ctrl;
    d->ctrl[RSY_actuatorID] = RSY_ctrl;
    d->ctrl[R_elbow_actuatorID] = R_elbow_ctrl;
    //d->ctrl[RTA_actuatorID] = RTA_ctrl;
    //d->ctrl[RTB_actuatorID] = RTB_ctrl;



    //printf("Toe pitch orientation = %f \n",d->xquat[RTP]);

    
    // printf("position = %f \n",d->qpos[L_elbow_joint_adr]);
    // //for (int i=0;i<=22;i++)
    //   printf("%f \n",d->sensordata[L_elbow_sensor_adr]);

      clock_gettime(CLOCK_REALTIME, &spec);
      ms = (spec.tv_sec)*1000+(spec.tv_nsec)/1.0e6 - time_start;
      msd = (double)ms/1000 - setup_time;
      cont_time = (double)ms/1000 - setup_time;


    // printf("The knee motor position is " );
    // printf("%lf", d->sensordata[LK_sensor_adr]);
    // printf("\n");

    // printf("The hip pitch joint position is " );
    // printf("%lf", d->sensordata[LHP_sensor_adr]);
    // printf("\n");

    // printf("The hip pitch joint gravity torque is " );
    // printf("%lf", d->qfrc_bias[LHP_joint_adr]);
    // printf("\n");

    // printf("The hip pitch joint controller torque is " );
    // printf("%lf", uu[0]);
    // printf("\n");

    // printf("The knee joint gravity torque is " );
    // printf("%lf", d->qfrc_bias[LK_joint_adr]);
    // printf("\n");

    // printf("The knee joint controller torque is " );
    // printf("%lf", uu[1]);
    // printf("\n");

    base_quat[0]=d->sensordata[base_quat_sensor_adr];
    base_quat[1]=d->sensordata[base_quat_sensor_adr+1];
    base_quat[2]=d->sensordata[base_quat_sensor_adr+2];
    base_quat[3]=d->sensordata[base_quat_sensor_adr+3];
    axis[0] = base_quat[1];
    axis[1] = base_quat[2];
    axis[2] = base_quat[3];
    sin_half_angle = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    quat_angle = 2 * atan2(sin_half_angle, base_quat[0]);

    base_angvel[0]=d->sensordata[base_angvel_sensor_adr];
    base_angvel[1]=d->sensordata[base_angvel_sensor_adr+1];
    base_angvel[2]=d->sensordata[base_angvel_sensor_adr+2];


    //theta0 = -1*d->sensordata[body_pitch_sensor_adr]*0;
    if (axis[1]>=0){
        quat_angle = quat_angle;
    }
    if (axis[1]<0){
        quat_angle = -1*quat_angle;
    }
    theta0 = quat_angle;
    theta1 = theta1_mo-d->sensordata[LHP_sensor_adr];
    theta2 = theta1_mo+d->sensordata[RHP_sensor_adr];
    theta3 = theta2_mo+d->sensordata[LK_sensor_adr];
    theta4 = theta2_mo-d->sensordata[RK_sensor_adr];

    theta5 = d->sensordata[LHR_sensor_adr]-20*M_PI/180; //LHR
    theta6 = d->sensordata[RHR_sensor_adr]+20*M_PI/180; //RHR

    //omega0 = -1*d->sensordata[body_pitch_vel_sensor_adr];
    double omega00 = -1*d->sensordata[base_angvel_sensor_adr+1];
    omega0 = omega00;
    omega1 = -1*d->sensordata[LHP_vel_sensor_adr];
    omega2=  d->sensordata[RHP_vel_sensor_adr];
    omega3 = d->sensordata[LK_vel_sensor_adr];
    omega4 = -1*d->sensordata[RK_vel_sensor_adr];

    omega5 = d->sensordata[LHR_vel_sensor_adr];
    omega6 = d->sensordata[RHR_vel_sensor_adr];

    z[0]=theta0;
    z[1]=omega0;
    z[2]=theta1; //Uncontrolled state
    z[3]=omega1;
    z[4]=theta2;
    z[5]=omega2;
    z[6]=theta3;
    z[7]=omega3;
    z[8]=theta4;
    z[9]=omega4;

    double Ltoe_pitch;
    parallel_toe(&theta0, &theta1, &theta3, &Ltoe_pitch);
    double Rtoe_pitch;
    parallel_toe(&theta0, &theta2, &theta4, &Rtoe_pitch);

    printf("The torso position is: ");
    printf("%f \n", theta0);

    printf("The left hip-pitch position is: ");
    printf("%f \n", theta1);

    printf("The right hip-pitch position is: ");
    printf("%f \n", theta2);

    printf("The left-knee position is: ");
    printf("%f \n", theta3);
    printf("The left-knee joint mjposition is: ");
    printf("%f \n", d->sensordata[LK_sensor_adr]);

    printf("The right-knee position is: ");
    printf("%f \n", theta4);

    printf("The left toe-pitch absolute parallel position is: ");
    printf("%f \n", Ltoe_pitch);

    printf("The left toe-pitch actual position is: ");
    printf("%f \n", d->sensordata[LToe_pitch_sensor_adr]);

    printf("The right toe-pitch absolute parallel position is: ");
    printf("%f \n", Rtoe_pitch);

    printf("The right toe-pitch actual position is: ");
    printf("%f \n", d->sensordata[RToe_pitch_sensor_adr]);

    printf("mid1 angle1 is: %f \n", mid1_angle1);
    printf("mid1 angle2 is: %f \n", mid1_angle2);
    printf("mid2 angle1 is: %f \n", mid2_angle1);
    printf("mid2 angle2 is: %f \n", mid2_angle2);
    printf("fs1 angle1 is: %f \n", fs1_angle1);
    printf("fs1 angle2 is: %f \n", fs1_angle2);
    printf("fs2 angle1 is: %f \n", fs2_angle1);
    printf("fs2 angle2 is: %f \n", fs2_angle2);

    printf("Ground z is: %f \n", ground_z);

    printf("base pos sensor data X: ");
    printf("%f \n",d->sensordata[base_sensor_adr]);
    printf("base x joint pos sensor data X: ");
    printf("%f \n",d->sensordata[body_x_sensor_adr]);
    printf("base pos sensor data Z: ");
    printf("%f \n",d->sensordata[base_sensor_adr+2]);
    printf("base z joint pos sensor data Z: ");
    printf("%f \n",d->sensordata[body_z_sensor_adr]);

    //x_Bf=d->sensordata[body_x_sensor_adr];
    //y_Bf=d->sensordata[body_z_sensor_adr];
    x_Bf=d->sensordata[base_sensor_adr];
    y_Bf=d->sensordata[base_sensor_adr+2];

    getcom(&x_com, &y_com, params,z,&x_Bf,&y_Bf );
    printf("the xcom is: %f \n", x_com);
    printf("the LToe_x is: %f \n", LToe_x);

    printf("x_H - x_Bf should be: %f \n", LHP_x-x_Bf);
    printf("y_H - y_Bf should be: %f \n", LHP_y-y_Bf);

    printf("the LToeA_pitch: %f \n", LtoeA_pitch);
    printf("the LToeB_pitch: %f \n", LtoeB_pitch);

    printf("base_quat[0]:  %f \n", base_quat[0]);
    printf("base_quat[1]:  %f \n", base_quat[1]);
    printf("base_quat[2]:  %f \n", base_quat[2]);
    printf("base_quat[3]:  %f \n", base_quat[3]);

    printf("quat_angle:  %f \n", quat_angle);

    printf("base_angvel[0]:  %f \n", base_angvel[0]);
    printf("base_angvel[1]:  %f \n", base_angvel[1]);

    printf("theta5:  %f \n", theta5);


    if (msd<3*tf/5 && counter == -1)
    {
        Ltoe_pitch=d->sensordata[LToe_pitch_sensor_adr];
        LtoeA_pitch = Ltoe_pitch;
        LtoeB_pitch = Ltoe_pitch;
        double LTA_ctrl = (500*(d->sensordata[LToe_pitch_sensor_adr]-LtoeA_pitch));
        d->ctrl[LTA_actuatorID] = LTA_ctrl;
        double LTB_ctrl = (-500*(d->sensordata[LToe_pitch_sensor_adr]-LtoeB_pitch));
        d->ctrl[LTB_actuatorID] = LTB_ctrl;  

        double LHR_ctrl = -100*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
        d->ctrl[LHR_actuatorID] = LHR_ctrl;

       //Rtoe_pitch=d->sensordata[RToe_pitch_sensor_adr];
        //double RTA_ctrl = (500*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
        //d->ctrl[RTA_actuatorID] = RTA_ctrl;
        //double RTB_ctrl = (-500*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
        //d->ctrl[RTB_actuatorID] = RTB_ctrl; 

        
    }
    if(msd>=3*tf/5 &&counter == -1){
        LtoeA_pitch=d->sensordata[LTA_sensor_adr];//get actuator position
        LtoeB_pitch=d->sensordata[LTB_sensor_adr];
        counter=0;

        double LHR_ctrl = -100*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
        d->ctrl[LHR_actuatorID] = LHR_ctrl;
    }


    if (msd>=tf && counter ==0)
    {
      s0[0][0] = theta0;
      s0[0][1] = theta1;
      s0[0][2] = theta2;
      s0[0][3] = theta3;
      s0[0][4] = theta4;
      v0[0][0] = omega0*1;
      v0[0][1] = omega1*1;
      v0[0][2] = omega2*1;
      v0[0][3] = omega3*1;
      v0[0][4] = omega4*1;
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      a0[0][2]=0;
      a0[0][3]=0;
      a0[0][4]=0; 

      s0r[0][0]=theta5;
      s0r[0][1]=theta6;
      v0r[0][0]=omega5*0;
      v0r[0][1]=omega6*0;
      a0r[0][0]=0;
      a0r[0][1]=0;

      //sf[0][0] = s0[0][0];
      //sf[0][1] = s0[0][1];
      sf[0][0] = -0.00;
      sf[0][1] = mid1_angle1;
      sf[0][2] = mid1_angle1;
      sf[0][3] = mid1_angle2;
      sf[0][4] = mid1_angle2;

      sfr[0][0]=0*M_PI/180;
      sfr[0][1]=0*M_PI/180;

      //sf[0][1] = s0[0][1];
      //sf[0][2] = s0[0][2];
      //sf[0][3] = s0[0][3];
      //sf[0][4] = s0[0][4];
      
      
      edt[0][0]=0; //accumulated error for integral control
      edt[0][1]=0;

      counter = 1;
      if(plotcounter>10){
        plotcounter=0;
        j=0;
      }
      
      
      LtoeA_pitch=d->sensordata[LTA_sensor_adr];//get actuator position
      LtoeB_pitch=d->sensordata[LTB_sensor_adr];
      // double Ltoe_pitch;
      // parallel_toe(&theta0, &theta1, &theta3, &Ltoe_pitch);
      // double Rtoe_pitch;
      // parallel_toe(&theta0, &theta2, &theta4, &Rtoe_pitch);
    }
    if (stage == 1 && counter ==1)
    {
      s0[0][0] = theta0;
      s0[0][1] = theta2;
      s0[0][2] = theta1;
      s0[0][3] = theta4;
      s0[0][4] = theta3;
      v0[0][0] = omega0*0;
      v0[0][1] = omega2*0;
      v0[0][2] = omega1*0;
      v0[0][3] = omega4*0;
      v0[0][4] = omega3*0;
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      a0[0][2]=0;
      a0[0][3]=0;
      a0[0][4]=0; 

      s0r[0][0]=theta5;
      s0r[0][1]=theta6;
      v0r[0][0]=omega5;
      v0r[0][1]=omega6;

      //sf[0][0] = s0[0][0];
      //sf[0][1] = s0[0][1];
      sf[0][0] = -0.02;
      sf[0][1] = mid1_angle1;
      sf[0][2] = mid1_angle1;
      sf[0][3] = mid1_angle2;
      sf[0][4] = mid1_angle2;

      sfr[0][0]=4*M_PI/180;
      sfr[0][1]=4*M_PI/180;

      //sf[0][1] = s0[0][1];
      //sf[0][2] = s0[0][2];
      //sf[0][3] = s0[0][3];
      //sf[0][4] = s0[0][4];
      RtoeA_pitch=d->sensordata[RTA_sensor_adr];//get actuator position
      RtoeB_pitch=d->sensordata[RTB_sensor_adr];

      counter = 2;
    }
    if (stage == 2 && counter ==2)
    {
      s0[0][0] = theta0;
      s0[0][1] = theta2;
      s0[0][2] = theta1;
      s0[0][3] = theta4;
      s0[0][4] = theta3;
      v0[0][0] = omega0*1;
      v0[0][1] = omega2*1;
      v0[0][2] = omega1*1;
      v0[0][3] = omega4*1;
      v0[0][4] = omega3*1;
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      a0[0][2]=0;
      a0[0][3]=0;
      a0[0][4]=0; 

      s0r[0][0]=theta5;
      s0r[0][1]=theta6;
      v0r[0][0]=omega5;
      v0r[0][1]=omega6;

      //sf[0][0] = s0[0][0];
      //sf[0][1] = s0[0][1];
      sf[0][0] = -0.02;
      sf[0][1] = mid1_angle1;
      sf[0][2] = fs2_angle1;
      sf[0][3] = mid1_angle2;
      sf[0][4] = fs2_angle2;

      sfr[0][0]=0*M_PI/180;
      sfr[0][1]=0*M_PI/180;

      //sf[0][1] = s0[0][1];
      //sf[0][2] = s0[0][2];
      //sf[0][3] = s0[0][3];
      //sf[0][4] = s0[0][4];
      RtoeA_pitch=d->sensordata[RTA_sensor_adr];//get actuator position
      RtoeB_pitch=d->sensordata[RTB_sensor_adr];

      counter = 3;
    }

    if (stage == 3 && counter ==3)
    {
      s0[0][0] = theta0;
      s0[0][1] = theta1;
      s0[0][2] = theta2;
      s0[0][3] = theta3;
      s0[0][4] = theta4;
      v0[0][0] = omega0*1;
      v0[0][1] = omega1*1;
      v0[0][2] = omega2*1;
      v0[0][3] = omega3*1;
      v0[0][4] = omega4*1;
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      a0[0][2]=0;
      a0[0][3]=0;
      a0[0][4]=0; 

      s0r[0][0]=theta5;
      s0r[0][1]=theta6;
      v0r[0][0]=omega5;
      v0r[0][1]=omega6;

      //sf[0][0] = s0[0][0];
      //sf[0][1] = s0[0][1];
      sf[0][0] = 0.5;
      sf[0][1] = mid2_angle1;
      sf[0][2] = mid1_angle1;
      sf[0][3] = mid2_angle2;
      sf[0][4] = mid1_angle2;

      sfr[0][0]=-0*M_PI/180;
      sfr[0][1]=0*M_PI/180;

      //sf[0][1] = s0[0][1];
      //sf[0][2] = s0[0][2];
      //sf[0][3] = s0[0][3];
      //sf[0][4] = s0[0][4];
      

      counter = 4;

      LtoeA_pitch=d->sensordata[LTA_sensor_adr];//get actuator position
      LtoeB_pitch=d->sensordata[LTB_sensor_adr];
    }
    printf("%d \n", counter);

    if (msd<3*tf/5){
        //d->qpos[base_joint_adr]=0;
        //d->qpos[base_joint_adr+1]=0;
        //d->qpos[base_joint_adr+2]=1.065;
        d->qpos[base_joint_adr+3]=1;
        d->qpos[base_joint_adr+4]=0;
        d->qpos[base_joint_adr+5]=0;
        d->qpos[base_joint_adr+6]=0;
        d->qvel[base_joint_adr]=0;
        d->qvel[base_joint_adr+1]=0;
        d->qvel[base_joint_adr+2]=0;
       // d->qpos[body_x_joint_adr] = 0.0;
        //d->qpos[body_pitch_joint_adr] = 0.0;


        //d->qpos[body_z_joint_adr] = -0.92;
        //d->qpos[body_x_joint_adr] = 0.0;
        //d->qpos[body_pitch_joint_adr] = -0.01;//-0.01
        printf("--------------------------------------------------------------------------------------");

    }




    if(msd<tf){

        double LHR_ctrl = -500*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
        double RHR_ctrl = -500*(d->sensordata[RHR_sensor_adr]+20*M_PI/180)-(10*d->sensordata[RHR_vel_sensor_adr]);
        d->ctrl[LHR_actuatorID] = LHR_ctrl;
        d->ctrl[RHR_actuatorID] = RHR_ctrl;

        //d->qpos[body_x_joint_adr] = 0.0;
        //d->qpos[body_pitch_joint_adr] = -0.115;
        //d->qpos[body_pitch_joint_adr] = 0.5;
        //d->qpos[body_z_joint_adr] = 1.5;
        

       // d->qpos[2]=theta1_mo-mid1_angle1;//30.0/180.0*mjPI; //LHP

        //d->qpos[7] = 0.524087;//-theta2_mo+mid1_angle2;
    //d->qpos[9] = -0.524487 ;

        //d->ctrl[4] = -theta2_mo-mid1_angle2;//LK servo

    //d->qpos[2]=theta1_mo-mid1_angle1;//30.0/180.0*mjPI; //LHP

      //plotcounter=plotcounter+1;
      double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-mid1_angle1)))-(-10*omega1);
      d->ctrl[LHP_actuatorID] = LHP_ctrl;
      double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+mid1_angle2)))-(10*omega3);
      d->ctrl[LK_actuatorID] = LK_ctrl;

      double RHP_ctrl = (-500*(d->sensordata[RHP_sensor_adr]+(theta1_mo-mid1_angle1)))-(10*omega2);
      d->ctrl[RHP_actuatorID] = RHP_ctrl;
      double RK_ctrl = (-500*(d->sensordata[RK_sensor_adr]+(-theta2_mo+mid1_angle2)))-(-10*omega4);
      d->ctrl[RK_actuatorID] = RK_ctrl;


      //Left toe control
     //double LTA_ctrl = (500*(d->sensordata[LToe_pitch_sensor_adr]-LtoeA_pitch));
      //d->ctrl[LTA_actuatorID] = LTA_ctrl;
      //double LTB_ctrl = (-500*(d->sensordata[LToe_pitch_sensor_adr]-LtoeB_pitch));
      //d->ctrl[LTB_actuatorID] = LTB_ctrl;     

      double RTA_ctrl = (50*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      d->ctrl[RTA_actuatorID] = RTA_ctrl;
      double RTB_ctrl = (-50*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      d->ctrl[RTB_actuatorID] = RTB_ctrl;  

      if(msd>=3*tf/5){
        
        //d->qpos[base_joint_adr+3]=1;
        //d->qpos[base_joint_adr+4]=0;
        //d->qpos[base_joint_adr+5]=0;
        //d->qpos[base_joint_adr+6]=0;
        double LTA_ctrl = (-500*(d->sensordata[LTA_sensor_adr]-LtoeA_pitch));
      d->ctrl[LTA_actuatorID] = LTA_ctrl;
      double LTB_ctrl = (-500*(d->sensordata[LTB_sensor_adr]-LtoeB_pitch));
      d->ctrl[LTB_actuatorID] = LTB_ctrl;  
      }




    }

    else if(msd>=tf && stage==0 && counter==1){
        //theta0=-1*quat_angle;
        printf("stage0--------------------------------------------------------------------------------------");
        printf("tf: %f \n", tf);
        printf("msd: %f \n", msd);
        stanceToe_x = LToe_x;

        //d->qpos[body_x_joint_adr] = 0.0;
        //d->qpos[body_pitch_joint_adr] = 0.0;
        //d->qpos[body_z_joint_adr] = 1.5;

        if (msd < tf+traj_time){

            for(int i=0;i<5;i++){
            ctraj(&ss[0][i],&vv[0][i],&aa[0][i],msd,tf,tf+traj_time, s0[0][i], sf[0][i], v0[0][i], vf[0][i], a0[0][i], af[0][i]);
            }
            for(int i=0;i<2;i++){
                ctraj(&ssr[0][i],&vvr[0][i],&aar[0][i],msd,tf,tf+traj_time, s0r[0][i], sfr[0][i], v0r[0][i], vfr[0][i], a0r[0][i], afr[0][i]);
            }
            
        }

        printf("ssr[0][0]: %f \n", ssr[0][0]);
        printf("ssr[0][1]: %f \n", ssr[0][1]);

        

     // printf("%f", msd);
     // printf("\n");
     // printf("%f,%f,%f \n", ss[0][0],vv[0][0],aa[0][0]);
     // printf("%f,%f,%f \n", ss[0][1],vv[0][1],aa[0][1]);

      traj_des[0]=ss[0][0];
      traj_des[1]=ss[0][1];
      //traj_des[2]=ss[0][2];
      traj_des[3]=ss[0][3];
      //traj_des[4]=ss[0][4];
      traj_des[2]=theta1;
      traj_des[4]=theta3;

      theta2_des=theta2;//swing leg hip angle (not desired)
      theta3_des=theta3;//stance leg knee angle
      theta4_des=theta4;//swing leg knee angle

      torso_des=traj_des[0];
      LHP_des=traj_des[1];
      RHP_des=traj_des[2];
      LK_des=traj_des[3];
      RK_des=traj_des[4];
      
      traj_des[5]=vv[0][0];
      //traj_des[6]=vv[0][1];
      traj_des[7]=vv[0][2];
      //traj_des[8]=vv[0][3];
      traj_des[9]=vv[0][4];
      traj_des[6]=0;
      traj_des[8]=0;

      traj_des[10]=aa[0][0];
      traj_des[11]=aa[0][1];
      //traj_des[12]=aa[0][2];
      traj_des[13]=aa[0][3];
      //traj_des[14]=aa[0][4];
      traj_des[11]=0;
      traj_des[13]=0;

      z[0]=theta0;
      z[1]=omega0;
      z[2]=theta1; //Uncontrolled state
      z[3]=omega1;
      z[4]=theta2;
      z[5]=omega2;
      z[6]=theta3;
      z[7]=omega3;
      z[8]=theta4;
      z[9]=omega4;
   

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

      //d->ctrl[LHP_actuatorID] = -1*uu[0]/16.0;
      //d->ctrl[RHP_actuatorID] = uu[1]/16.0;
      //d->ctrl[LK_actuatorID] = 1*uu[2]/16.0;
      //d->ctrl[RK_actuatorID] = -1*uu[3]/16.0;

      double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-traj_des[1])))-(-10*omega1);
      d->ctrl[LHP_actuatorID] = LHP_ctrl;
      double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+traj_des[3])))-(10*omega3);
      d->ctrl[LK_actuatorID] = LK_ctrl;

      double RHP_ctrl = (-500*(d->sensordata[RHP_sensor_adr]+(theta1_mo-traj_des[1])))-(10*omega2);
      d->ctrl[RHP_actuatorID] = RHP_ctrl;
      double RK_ctrl = (-500*(d->sensordata[RK_sensor_adr]+(-theta2_mo+traj_des[3])))-(-10*omega4);
      d->ctrl[RK_actuatorID] = RK_ctrl;

      double LHR_ctrl = -500*(theta5-ssr[0][0])-(10*omega5); //double LHR_ctrl = -500*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
      double RHR_ctrl = -500*(theta6-ssr[0][1])-(10*omega6); //double RHR_ctrl = -500*(d->sensordata[RHR_sensor_adr]+20*M_PI/180)-(10*d->sensordata[RHR_vel_sensor_adr]);

      d->ctrl[LHR_actuatorID] = LHR_ctrl;
      d->ctrl[RHR_actuatorID] = RHR_ctrl;

        
      //double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-mid1_angle1)))-(-10*omega1);
      //d->ctrl[LHP_actuatorID] = LHP_ctrl;
      //double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+mid1_angle2)))-(10*omega3);
      //d->ctrl[LK_actuatorID] = LK_ctrl;

      //Left toe control
      //double LTA_ctrl = (5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTA_actuatorID] = LTA_ctrl;
      //double LTB_ctrl = (-5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTB_actuatorID] = LTB_ctrl;     

      // double RTA_ctrl = (500*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      // d->ctrl[RTA_actuatorID] = RTA_ctrl;
      // double RTB_ctrl = (-500*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      // d->ctrl[RTB_actuatorID] = RTB_ctrl;  

      // //anchor foot
      // double LTA_ctrl = (-0*(d->sensordata[LTA_sensor_adr]-LtoeA_pitch));
      // d->ctrl[LTA_actuatorID] = LTA_ctrl;
      // double LTB_ctrl = (-0*(d->sensordata[LTB_sensor_adr]-LtoeB_pitch));
      // d->ctrl[LTB_actuatorID] = LTB_ctrl;

      double Ltoe_pitch;
      parallel_toe(&theta0, &theta1, &theta3, &Ltoe_pitch);
      double Rtoe_pitch;
      parallel_toe(&theta0, &theta2, &theta4, &Rtoe_pitch);

      double LTA_ctrl = (10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1])-1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTA_actuatorID] = LTA_ctrl;
      double LTB_ctrl = (-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1])+1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTB_actuatorID] = LTB_ctrl; 

       // double RTA_ctrl = 15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])+2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);;
       // d->ctrl[RTA_actuatorID] = RTA_ctrl;
       // double RTB_ctrl = -15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])-2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
       // d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      double RTA_ctrl = (-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1])+1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);;
       d->ctrl[RTA_actuatorID] = RTA_ctrl;
       double RTB_ctrl = (10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1])-1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);
       d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      if (msd>tf+traj_time){ //0.027check spring deflection to ensure weight is on stance leg
        stage=1;
        stage_time=msd;
        //plotcounter=10;
      }

     /* if (RToe_y <= ground_z && (sensor_comx-RToe_x)>-0.159){ //0.027check spring deflection to ensure weight is on stance leg
        stage=1;
        stage_time=msd;
        //plotcounter=10;
      }*/

        ///////////////////////////
  }

    else if(stage==1 && counter==2){
        printf("****************************************************************************8");
        stanceToe_x = RToe_x;
       
       if (msd < stage_time+traj_time){

            for(int i=0;i<5;i++){
            ctraj(&ss[0][i],&vv[0][i],&aa[0][i],msd,stage_time,stage_time+traj_time, s0[0][i], sf[0][i], v0[0][i], vf[0][i], a0[0][i], af[0][i]);
            }
            for(int i=0;i<2;i++){
                ctraj(&ssr[0][i],&vvr[0][i],&aar[0][i],msd,stage_time,stage_time+traj_time, s0r[0][i], sfr[0][i], v0r[0][i], vfr[0][i], a0r[0][i], afr[0][i]);
            }
        }

        

     // printf("%f", msd);
     // printf("\n");
     // printf("%f,%f,%f \n", ss[0][0],vv[0][0],aa[0][0]);
     // printf("%f,%f,%f \n", ss[0][1],vv[0][1],aa[0][1]);

      traj_des[0]=ss[0][0];
      traj_des[1]=ss[0][1];
      traj_des[2]=ss[0][2];
      traj_des[3]=ss[0][3];
      traj_des[4]=ss[0][4];
      
      traj_des[5]=vv[0][0];
      traj_des[6]=vv[0][1];
      traj_des[7]=vv[0][2];
      traj_des[8]=vv[0][3];
      traj_des[9]=vv[0][4];

      traj_des[10]=aa[0][0];
      traj_des[11]=aa[0][1];
      traj_des[12]=aa[0][2];
      traj_des[13]=aa[0][3];
      traj_des[14]=aa[0][4];

      theta2_des=theta1;
      theta3_des=theta4;
      theta4_des=theta3;//swing leg knee angle

      torso_des=traj_des[0];
      LHP_des=traj_des[2];
      RHP_des=traj_des[1];
      LK_des=traj_des[4];
      RK_des=traj_des[3];

      z[0]=theta0;
      z[1]=omega0;
      z[2]=theta2; //Uncontrolled state
      z[3]=omega2;
      z[4]=theta1;
      z[5]=omega1;
      z[6]=theta4;
      z[7]=omega4;
      z[8]=theta3;
      z[9]=omega3;
   

      //c_controller(uu,z,params,traj_des,timestep,edt); 
      // c_controller(uu,z,params,traj_des); 



      // d->ctrl[RHP_actuatorID] = 1*uu[0]/16.0;
      // d->ctrl[LHP_actuatorID] = -1*uu[1]/16.0;
      // d->ctrl[RK_actuatorID] = -1*uu[2]/16.0;
      // d->ctrl[LK_actuatorID] = 1*uu[3]/16.0;

      //double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-mid1_angle1)))-(-10*omega1);
      //d->ctrl[LHP_actuatorID] = LHP_ctrl;
      //double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+mid1_angle2)))-(10*omega3);
      //d->ctrl[LK_actuatorID] = LK_ctrl;

      //Left toe control
      //double LTA_ctrl = (5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTA_actuatorID] = LTA_ctrl;
      //double LTB_ctrl = (-5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTB_actuatorID] = LTB_ctrl;     

      // double RTA_ctrl = (-100*0*(d->sensordata[RTA_sensor_adr]-RtoeA_pitch));
      // d->ctrl[RTA_actuatorID] = RTA_ctrl;
      // double RTB_ctrl = (-100*0*(d->sensordata[RTB_sensor_adr]-RtoeB_pitch));
      // d->ctrl[RTB_actuatorID] = RTB_ctrl;  

      // //anchor foot
      // double LTA_ctrl = (500*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      // d->ctrl[LTA_actuatorID] = LTA_ctrl;
      // double LTB_ctrl = (-500*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      // d->ctrl[LTB_actuatorID] = LTB_ctrl; 

      double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-traj_des[1])))-(-10*omega1);
      //double LHP_ctrl = (5000*(theta0-traj_des[0]))-(-200*omega1);
      d->ctrl[LHP_actuatorID] = LHP_ctrl;
      double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+traj_des[3])))-(10*omega3);
      d->ctrl[LK_actuatorID] = LK_ctrl;

      //double RHP_ctrl = (-5000*(theta0-traj_des[0]))-(200*omega2);
      double RHP_ctrl = (-500*(d->sensordata[RHP_sensor_adr]+(theta1_mo-traj_des[1])))-(10*omega2);
      d->ctrl[RHP_actuatorID] = RHP_ctrl;
      double RK_ctrl = (-500*(d->sensordata[RK_sensor_adr]+(-theta2_mo+traj_des[3])))-(-10*omega4);
      d->ctrl[RK_actuatorID] = RK_ctrl;

      double LHR_ctrl = -500*(theta5-ssr[0][0])-(10*omega5); //double LHR_ctrl = -500*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
      double RHR_ctrl = -500*(theta6-ssr[0][1])-(10*omega6); //double RHR_ctrl = -500*(d->sensordata[RHR_sensor_adr]+20*M_PI/180)-(10*d->sensordata[RHR_vel_sensor_adr]);

      d->ctrl[LHR_actuatorID] = LHR_ctrl;
      d->ctrl[RHR_actuatorID] = RHR_ctrl;

      double Ltoe_pitch;
      parallel_toe(&theta0, &theta1, &theta3, &Ltoe_pitch);
      double Rtoe_pitch;
      parallel_toe(&theta0, &theta2, &theta4, &Rtoe_pitch);

      double LTA_ctrl = (10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1])-1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTA_actuatorID] = LTA_ctrl;
      double LTB_ctrl = (-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1])+1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTB_actuatorID] = LTB_ctrl; 

       // double RTA_ctrl = 15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])+2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);;
       // d->ctrl[RTA_actuatorID] = RTA_ctrl;
       // double RTB_ctrl = -15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])-2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
       // d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      double RTA_ctrl = (-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1])+1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);;
       d->ctrl[RTA_actuatorID] = RTA_ctrl;
       double RTB_ctrl = (10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1])-1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);
       d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      if (msd >= stage_time+traj_time){
        stage=2;
        stage_time=msd;
        //plotcounter=2;
      }
      
    }

    else if(stage==2 && counter==3){
        printf("****************************************************************************9");
        stanceToe_x = RToe_x;
       
       if (msd < stage_time+traj_time2){

            for(int i=0;i<5;i++){
            ctraj(&ss[0][i],&vv[0][i],&aa[0][i],msd,stage_time,stage_time+traj_time2, s0[0][i], sf[0][i], v0[0][i], vf[0][i], a0[0][i], af[0][i]);
            }
            for(int i=0;i<2;i++){
                ctraj(&ssr[0][i],&vvr[0][i],&aar[0][i],msd,stage_time,stage_time+traj_time2, s0r[0][i], sfr[0][i], v0r[0][i], vfr[0][i], a0r[0][i], afr[0][i]);
            }
        }

        

     // printf("%f", msd);
     // printf("\n");
     // printf("%f,%f,%f \n", ss[0][0],vv[0][0],aa[0][0]);
     // printf("%f,%f,%f \n", ss[0][1],vv[0][1],aa[0][1]);

      traj_des[0]=ss[0][0];
      traj_des[1]=ss[0][1];
      traj_des[2]=ss[0][2];
      traj_des[3]=ss[0][3];
      traj_des[4]=ss[0][4];
      
      traj_des[5]=vv[0][0];
      traj_des[6]=vv[0][1];
      traj_des[7]=vv[0][2];
      traj_des[8]=vv[0][3];
      traj_des[9]=vv[0][4];

      traj_des[10]=aa[0][0];
      traj_des[11]=aa[0][1];
      traj_des[12]=aa[0][2];
      traj_des[13]=aa[0][3];
      traj_des[14]=aa[0][4];

      theta2_des=theta1;
      theta3_des=theta4;
      theta4_des=theta3;//swing leg knee angle

      torso_des=traj_des[0];
      LHP_des=traj_des[2];
      RHP_des=traj_des[1];
      LK_des=traj_des[4];
      RK_des=traj_des[3];

      z[0]=theta0;
      z[1]=omega0;
      z[2]=theta2; //Uncontrolled state
      z[3]=omega2;
      z[4]=theta1;
      z[5]=omega1;
      z[6]=theta4;
      z[7]=omega4;
      z[8]=theta3;
      z[9]=omega3;
   

      //c_controller(uu,z,params,traj_des,timestep,edt); 
      c_controller(uu,z,params,traj_des); 



      // d->ctrl[RHP_actuatorID] = 1*uu[0]/16.0;
      // d->ctrl[LHP_actuatorID] = -1*uu[1]/16.0;
      // d->ctrl[RK_actuatorID] = -1*uu[2]/16.0;
      // d->ctrl[LK_actuatorID] = 1*uu[3]/16.0;

      //double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-mid1_angle1)))-(-10*omega1);
      //d->ctrl[LHP_actuatorID] = LHP_ctrl;
      //double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+mid1_angle2)))-(10*omega3);
      //d->ctrl[LK_actuatorID] = LK_ctrl;

      //Left toe control
      //double LTA_ctrl = (5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTA_actuatorID] = LTA_ctrl;
      //double LTB_ctrl = (-5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTB_actuatorID] = LTB_ctrl;     

      // double RTA_ctrl = (-0*(d->sensordata[RTA_sensor_adr]-RtoeA_pitch));
      // d->ctrl[RTA_actuatorID] = RTA_ctrl;
      // double RTB_ctrl = (-0*(d->sensordata[RTB_sensor_adr]-RtoeB_pitch));
      // d->ctrl[RTB_actuatorID] = RTB_ctrl;  

      //double RTA_ctrl = (50*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      //d->ctrl[RTA_actuatorID] = RTA_ctrl;
      //double RTB_ctrl = (-50*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      //d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      //anchor foot
      // double LTA_ctrl = (500*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      // d->ctrl[LTA_actuatorID] = LTA_ctrl;
      // double LTB_ctrl = (-500*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      // d->ctrl[LTB_actuatorID] = LTB_ctrl;

      double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-traj_des[1])))-(-10*omega1);
      //double LHP_ctrl = (5000*(theta0-traj_des[0]))-(-200*omega1);
      d->ctrl[LHP_actuatorID] = LHP_ctrl;
      double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+traj_des[3])))-(10*omega3);
      d->ctrl[LK_actuatorID] = LK_ctrl;

      //double RHP_ctrl = (-5000*(theta0-traj_des[0]))-(200*omega2);
      double RHP_ctrl = (-500*(d->sensordata[RHP_sensor_adr]+(theta1_mo-traj_des[2])))-(10*omega2);
      d->ctrl[RHP_actuatorID] = RHP_ctrl;
      double RK_ctrl = (-500*(d->sensordata[RK_sensor_adr]+(-theta2_mo+traj_des[4])))-(-10*omega4);
      d->ctrl[RK_actuatorID] = RK_ctrl;

      double LHR_ctrl = -1000*(theta5-ssr[0][0])-(10*omega5); //double LHR_ctrl = -500*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
      double RHR_ctrl = -1000*(theta6-ssr[0][1])-(10*omega6); //double RHR_ctrl = -500*(d->sensordata[RHR_sensor_adr]+20*M_PI/180)-(10*d->sensordata[RHR_vel_sensor_adr]);

      d->ctrl[LHR_actuatorID] = LHR_ctrl;
      d->ctrl[RHR_actuatorID] = RHR_ctrl;

      double Ltoe_pitch;
      parallel_toe(&theta0, &theta1, &theta3, &Ltoe_pitch);
      double Rtoe_pitch;
      parallel_toe(&theta0, &theta2, &theta4, &Rtoe_pitch);

      double LTA_ctrl = (10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1])-1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTA_actuatorID] = LTA_ctrl;
      double LTB_ctrl = (-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1])+1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTB_actuatorID] = LTB_ctrl; 

       // double RTA_ctrl = 15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])+2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);;
       // d->ctrl[RTA_actuatorID] = RTA_ctrl;
       // double RTB_ctrl = -15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])-2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
       // d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      double RTA_ctrl = 1*((-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1]))+1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);;
       d->ctrl[RTA_actuatorID] = RTA_ctrl;
       double RTB_ctrl = 1*((10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1]))-1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);
       d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      if (msd >= stage_time+traj_time2){//-0.027
        stage=3;
        stage_time=msd;
      }
      
    }

    else if(stage==3 && counter==4){
        printf("--------------------------------------------------------------------------------------");
        stanceToe_x = LToe_x;

        //d->qpos[body_x_joint_adr] = 0.0;
        //d->qpos[body_pitch_joint_adr] = 0.0;
        //d->qpos[body_z_joint_adr] = 1.5;

        if (msd < stage_time + traj_time2){

            for(int i=0;i<5;i++){
            ctraj(&ss[0][i],&vv[0][i],&aa[0][i],msd,stage_time,stage_time+traj_time2, s0[0][i], sf[0][i], v0[0][i], vf[0][i], a0[0][i], af[0][i]);
            }
            for(int i=0;i<2;i++){
                ctraj(&ssr[0][i],&vvr[0][i],&aar[0][i],msd,stage_time,stage_time+traj_time2, s0r[0][i], sfr[0][i], v0r[0][i], vfr[0][i], a0r[0][i], afr[0][i]);
            }
        }

        

     // printf("%f", msd);
     // printf("\n");
     // printf("%f,%f,%f \n", ss[0][0],vv[0][0],aa[0][0]);
     // printf("%f,%f,%f \n", ss[0][1],vv[0][1],aa[0][1]);

      traj_des[0]=ss[0][0];
      traj_des[1]=ss[0][1];
      traj_des[2]=ss[0][2];
      traj_des[3]=ss[0][3];
      traj_des[4]=ss[0][4];
      
      traj_des[5]=vv[0][0];
      traj_des[6]=vv[0][1];
      traj_des[7]=vv[0][2];
      traj_des[8]=vv[0][3];
      traj_des[9]=vv[0][4];

      traj_des[10]=aa[0][0];
      traj_des[11]=aa[0][1];
      traj_des[12]=aa[0][2];
      traj_des[13]=aa[0][3];
      traj_des[14]=aa[0][4];

      theta2_des=theta2;
      theta3_des=theta3;
      theta4_des=theta4;//swing leg knee angle

      torso_des=traj_des[0];
      LHP_des=traj_des[1];
      RHP_des=traj_des[2];
      LK_des=traj_des[3];
      RK_des=traj_des[4];

      z[0]=theta0;
      z[1]=omega0;
      z[2]=theta1; //Uncontrolled state
      z[3]=omega1;
      z[4]=theta2;
      z[5]=omega2;
      z[6]=theta3;
      z[7]=omega3;
      z[8]=theta4;
      z[9]=omega4;
   

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

      // d->ctrl[LHP_actuatorID] = -1*uu[0]/16.0;
      // d->ctrl[RHP_actuatorID] = uu[1]/16.0;
      // d->ctrl[LK_actuatorID] = 1*uu[2]/16.0;
      // d->ctrl[RK_actuatorID] = -1*uu[3]/16.0;

      //double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-mid1_angle1)))-(-10*omega1);
      //d->ctrl[LHP_actuatorID] = LHP_ctrl;
      //double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+mid1_angle2)))-(10*omega3);
      //d->ctrl[LK_actuatorID] = LK_ctrl;

      //Left toe control
      //double LTA_ctrl = (5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTA_actuatorID] = LTA_ctrl;
      //double LTB_ctrl = (-5*(d->sensordata[LToe_pitch_sensor_adr]-Ltoe_pitch));
      //d->ctrl[LTB_actuatorID] = LTB_ctrl;     

      // double RTA_ctrl = (500*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      // d->ctrl[RTA_actuatorID] = RTA_ctrl;
      // double RTB_ctrl = (-500*(d->sensordata[RToe_pitch_sensor_adr]+Rtoe_pitch));
      // d->ctrl[RTB_actuatorID] = RTB_ctrl;  

      // //anchor foot
      // double LTA_ctrl = (-100*0*(d->sensordata[LTA_sensor_adr]-LtoeA_pitch));
      // d->ctrl[LTA_actuatorID] = LTA_ctrl;
      // double LTB_ctrl = (-100*0*(d->sensordata[LTB_sensor_adr]-LtoeB_pitch));
      // d->ctrl[LTB_actuatorID] = LTB_ctrl;   

      double LHP_ctrl = (-500*(d->sensordata[LHP_sensor_adr]-(theta1_mo-traj_des[1])))-(-10*omega1);
      //double LHP_ctrl = (5000*(theta0-traj_des[0]))-(-200*omega1);
      d->ctrl[LHP_actuatorID] = LHP_ctrl;
      double LK_ctrl = (-500*(d->sensordata[LK_sensor_adr]-(-theta2_mo+traj_des[3])))-(10*omega3);
      d->ctrl[LK_actuatorID] = LK_ctrl;

      //double RHP_ctrl = (-5000*(theta0-traj_des[0]))-(200*omega2);
      double RHP_ctrl = (-500*(d->sensordata[RHP_sensor_adr]+(theta1_mo-traj_des[2])))-(10*omega2);
      d->ctrl[RHP_actuatorID] = RHP_ctrl;
      double RK_ctrl = (-500*(d->sensordata[RK_sensor_adr]+(-theta2_mo+traj_des[4])))-(-10*omega4);
      d->ctrl[RK_actuatorID] = RK_ctrl;

      double LHR_ctrl = -500*(theta5-ssr[0][0])-(10*omega5); //double LHR_ctrl = -500*(d->sensordata[LHR_sensor_adr]-20*M_PI/180)-(10*d->sensordata[LHR_vel_sensor_adr]);
      double RHR_ctrl = -500*(theta6-ssr[0][1])-(10*omega6); //double RHR_ctrl = -500*(d->sensordata[RHR_sensor_adr]+20*M_PI/180)-(10*d->sensordata[RHR_vel_sensor_adr]);

      d->ctrl[LHR_actuatorID] = LHR_ctrl;
      d->ctrl[RHR_actuatorID] = RHR_ctrl;

      double Ltoe_pitch;
      parallel_toe(&theta0, &theta1, &theta3, &Ltoe_pitch);
      double Rtoe_pitch;
      parallel_toe(&theta0, &theta2, &theta4, &Rtoe_pitch);

      double LTA_ctrl = (10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1])-1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTA_actuatorID] = LTA_ctrl;
      double LTB_ctrl = (-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1])+1.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
      d->ctrl[LTB_actuatorID] = LTB_ctrl; 

       // double RTA_ctrl = 15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])+2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);;
       // d->ctrl[RTA_actuatorID] = RTA_ctrl;
       // double RTB_ctrl = -15*(d->sensordata[LToe_pitch_sensor_adr]+d->sensordata[RToe_pitch_sensor_adr])-2.0*(Ltoe_pitch-d->sensordata[LToe_pitch_sensor_adr]);
       // d->ctrl[RTB_actuatorID] = RTB_ctrl; 

      double RTA_ctrl = 0*((-10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])-1.2*(base_angvel[1]))+1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);;
       d->ctrl[RTA_actuatorID] = RTA_ctrl;
       double RTB_ctrl = 0*((10*(sensor_comx-LToe_x))-0.0*(d->sensordata[LTA_vel_sensor_adr])+1.2*(base_angvel[1]))-1.0*(Rtoe_pitch+d->sensordata[RToe_pitch_sensor_adr]);
       d->ctrl[RTB_actuatorID] = RTB_ctrl; 


      if (msd >= stage_time + traj_time2){
        stage=4;
        stage_time=msd;
        //plotcounter=2;
      }


        ///////////////////////////
  }
        


      else{
        clock_gettime(CLOCK_REALTIME, &spec);

        //time_start_l  = (spec.tv_sec)*1000+(spec.tv_nsec)/1.0e6;
        //time_start = (double)time_start_l;
        //setup_time=-tf;//Remove this to have a setup time beforethe initialization of each loop

        tf=msd;
        counter=0;
        stage=0;
        plotcounter=plotcounter+1;
        
      }


      if (plotcounter<6 && plotbreak==0){
        yy1 = realloc(yy1, (j+1)*sizeof(double));
        yy2 = realloc(yy2, (j+1)*sizeof(double));
        xx = realloc(xx, (j+1)*sizeof(double));
        xx [j]=cont_time;
        //yy1 [j]=x_com; //actual red 
        //yy2 [j]=stanceToe_x; //desired blue
        //yy1 [j]=LHS_defl; //actual red 
        //yy2 [j]=RHS_defl; //desired blue
        yy1 [j]=theta2_des; //actual red 
        yy2 [j]=sf[0][2];//traj_des[2]; //desired blue
        yy3 = realloc(yy3, (j+1)*sizeof(double));
        yy4 = realloc(yy4, (j+1)*sizeof(double));    
        //yy3 [j]=x_com-stanceToe_x; //actual red 
        //yy4 [j]=traj_des[4]; //desired blue
        //yy3 [j]=LShin_defl; //actual red 
        //yy4 [j]=RShin_defl; //desired blue
        yy3 [j]=theta4_des; //actual red 
        yy4 [j]=sf[0][4];//traj_des[4]; //desired blue
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

        Data_csv = realloc(Data_csv, (j+1)*sizeof(double)*11);
        Data_csv[j][0]=msd;
        Data_csv[j][1]=theta0;
        Data_csv[j][2]=theta1;
        Data_csv[j][3]=theta2;
        Data_csv[j][4]=theta3;
        Data_csv[j][5]=theta4;
        Data_csv[j][6]=torso_des;
        Data_csv[j][7]=LHP_des;
        Data_csv[j][8]=RHP_des;
        Data_csv[j][9]=LK_des;
        Data_csv[j][10]=RK_des;



        
      }

      if (plotcounter==6 && plotbreak==0){
        printf("----------------------------------------");
        plotcounter=6;
        plotbreak=1;
        
        //remove("example77.png");

        char file_name[100] = "data_files/test2_csv";

        matcsv(file_name, j, 11, Data_csv);

       

        //matPlot2(xx,yy1,xx,yy2,j,"xcom_vs_stancefootx_0.35_.05_.05_-.01_-.01.png",L"COM (red) vs Foot (blue)", L"time (s)",L"X Position (m)");
        //matPlot2(xx,yy3,xx,yy3,j,"xcom_error_0.35_.05_.05_-.01_-.01.png",L"COM Tracking Error", L"time (s)",L"X Position Error (m)");
        //matPlot2(xx,yy1,xx,yy2,j,"heel_spring_deflection.png",L"Swing Hip (red) Desired final position (blue)", L"time (s)",L"Position (rad)");
        //matPlot2(xx,yy3,xx,yy4,j,"shin_spring_deflection.png",L"Swing Knee (red) Desired final position(blue)", L"time (s)",L"Position (rad)");
                //matPlot2(xx,yt1,xx,yg1,j,"hip_torque_and_gravity_torque.png",L"Hip Control (red) Gravity Torque (blue)",L"time (s)",L"Torque (N-m)");
        //matPlot2(xx,yt2,xx,yg2,j,"knee_torque_and_gravity_torque.png",L"Knee Control (red) Gravity (blue)",L"time (s)",L"Torque (N-m)");
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
  params = (double*)malloc(sizeof(double)*27);
    traj_des = (double *)malloc(sizeof(double)*15);
    z = (double *)malloc(sizeof(double)*10);
    uu = (double *)malloc(sizeof(double)*4);
    get_params(params);

    L=0.9;
    step_angle=0*M_PI/180;
    getphaseangles(&mid1_angle1, &mid1_angle2, params, &L, &step_angle);

    L=0.9;
    step_angle=0*M_PI/180;
    getphaseangles(&mid2_angle1, &mid2_angle2, params, &L, &step_angle);

    L=0.9;
    step_angle=25*M_PI/180;
    getphaseangles(&fs1_angle1, &fs1_angle2, params, &L, &step_angle);

    L=0.7;
    step_angle=0*M_PI/180;
    getphaseangles(&fs2_angle1, &fs2_angle2, params, &L, &step_angle);


  //   double mid1_angle1; //Midstance Hip Pitch of stance leg
  // double mid1_angle2; //Midstance Knee of stance leg
  // double fs1_angle1; //Foot strike Hip pitch of trailing leg
  // double fs1_angle2; //Foot strike Knee of trailing leg
  // double mid2_angle1; //Midstance Hip Pitch of swing leg
  // double mid2_angle2; //Midstance knee of swing leg
  // double fs2_angle1;  //Foot strike Hip Pitch of leading leg
  // double fs2_angle2;  //Foot strike knee of leading leg





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
    //Data_csv = calloc(11, sizeof(double));
    Data_csv=calloc(11, sizeof(double));
  


  //// Above this line was Copied from lowlevel api
    
    // activate software
    mj_activate("../../../mjkey.txt");


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML("../model/digit_test3.xml", 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    //mj_saveLastXML("test_save_xml.xml", m, NULL, 0);

    const char* LHR_joint_name = "left-hip-roll";
        int LHR_jointID = mj_name2id(m, mjOBJ_JOINT, LHR_joint_name);
        int LHR_joint_adr = m->jnt_qposadr[LHR_jointID];
    const char* RHR_joint_name = "right-hip-roll";
        int RHR_jointID = mj_name2id(m, mjOBJ_JOINT, RHR_joint_name);
        int RHR_joint_adr = m->jnt_qposadr[RHR_jointID];
    const char* LHP_joint_name = "left-hip-pitch";
        int LHP_jointID = mj_name2id(m, mjOBJ_JOINT, LHP_joint_name);
        int LHP_joint_adr = m->jnt_qposadr[LHP_jointID];
    const char* RHP_joint_name = "right-hip-pitch";
        int RHP_jointID = mj_name2id(m, mjOBJ_JOINT, RHP_joint_name);
        int RHP_joint_adr = m->jnt_qposadr[RHP_jointID];
    const char* LK_joint_name = "left-knee";
        int LK_jointID = mj_name2id(m, mjOBJ_JOINT, LK_joint_name);
        int LK_joint_adr = m->jnt_qposadr[LK_jointID];
    const char* RK_joint_name = "right-knee";
        int RK_jointID = mj_name2id(m, mjOBJ_JOINT, RK_joint_name);
        int RK_joint_adr = m->jnt_qposadr[RK_jointID];
    const char* body_z_joint_name = "body_z";
        int body_z_jointID = mj_name2id(m, mjOBJ_JOINT, body_z_joint_name);
        int body_z_joint_adr = m->jnt_qposadr[body_z_jointID];
    const char* body_x_joint_name = "body_x";
        int body_x_jointID = mj_name2id(m, mjOBJ_JOINT, body_x_joint_name);
        int body_x_joint_adr = m->jnt_qposadr[body_x_jointID];
    const char* body_pitch_joint_name = "body-pitch";
        int body_pitch_jointID = mj_name2id(m, mjOBJ_JOINT, body_pitch_joint_name);
        int body_pitch_joint_adr = m->jnt_qposadr[body_pitch_jointID];
    const char* LToe_pos_name = "left-toe-pos";
        int LToe_sensorID = mj_name2id(m, mjOBJ_SENSOR, LToe_pos_name);
        int LToe_sensor_adr = m->sensor_adr[LToe_sensorID];
    const char* RToe_pos_name = "right-toe-pos";
        int RToe_sensorID = mj_name2id(m, mjOBJ_SENSOR, RToe_pos_name);
        int RToe_sensor_adr = m->sensor_adr[RToe_sensorID];
    const char* base_joint_name = "base";
        int base_jointID = mj_name2id(m, mjOBJ_JOINT, base_joint_name);
        int base_joint_adr = m->jnt_qposadr[base_jointID];
        d->qpos[base_joint_adr]=0;
        d->qpos[base_joint_adr+1]=0;
        d->qpos[base_joint_adr+2]=1.1;
        //d->qpos[base_joint_adr+3]=0;
        //d->qpos[base_joint_adr+4]=0;
        //d->qpos[base_joint_adr+5]=0;
        //d->qpos[base_joint_adr+6]=1;



    d->qpos[LHR_joint_adr]=20.0/180.0*mjPI; //LHR
    d->qpos[RHR_joint_adr]=-20.0/180.0*mjPI; //RHR

    for(int i=0;i<1;i++){
        d->qpos[LHP_joint_adr]=theta1_mo-mid1_angle1;//30.0/180.0*mjPI; //LHP
        //d->ctrl[4] = -theta2_mo-mid1_angle2;//-1.4;//LK servo
        d->qpos[LK_joint_adr] = -theta2_mo+mid1_angle2;//0.524087;//
        //d->qpos[9] = -0.524487 ;
        d->qpos[RHP_joint_adr]=-theta1_mo+mid1_angle1;//RHP
        //d->ctrl[11] = theta2_mo-mid2_angle2;//RK servo
        d->qpos[RK_joint_adr] = theta2_mo-mid1_angle2;
        //int base = mj_name2id(m, mjOBJ_BODY, "base");
        //d->xpos[base*3+2]=10;
        

        //d->qpos[body_z_joint_adr] = -0.92;
        //d->qpos[body_x_joint_adr] = 0.0;
        //d->qpos[body_pitch_joint_adr] = 0.0;
        
         }

    

    //d->qpos[5]=0+theta1_mo-0.5;//30.0/180.0*mjPI; //LHP
    //d->ctrl[4] = 0-theta2_mo-0.5;//LK servo
    //d->qpos[32]=0-theta1_mo+0.5;//RHP
    //d->ctrl[11] = 0+theta2_mo+0.8;//RK servo

    //d->qpos[0]=0; //
    //d->qpos[1]=0;//
    //d->qpos[2]=0; //
    //d->qvel[3]=0; //LHR
    //d->qvel[30]=0; //RHR
    //d->qvel[5]=0;//30.0/180.0*mjPI; //LHP
    //d->qvel[10]=0; //LK
    //d->qvel[0]=0; //
    //d->qvel[1]=0;//
    //d->qvel[2]=0; //
    //mj_kinematics(m,d);

    //m2=mj_copyData(m);

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
    cam.azimuth = 90;//-160;
    cam.elevation = 0;//-22.8;
    cam.distance = 4;
    cam.lookat[0] = -0.18;
    cam.lookat[1] = 0.2;
    cam.lookat[2] = 0.9;//0.48;
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