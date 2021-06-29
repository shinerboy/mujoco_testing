//------------------------------------------//
//  This file is a modified version of      //
//  basics.cpp, which was distributed as    //
//  part of MuJoCo,  Written by Emo Todorov //
//  Copyright (C) 2017 Roboti LLC           //
//  Modifications by Atabak Dehban          //
//------------------------------------------//

/*****************************************************************************
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/


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
  double tf = 2;
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
  double a1_offset = 0;
  double a2_offset=0;

  //////////////////////
  double theta1;
  double theta2;
  double omega1;
  double omega2; 
  double uu0;
  double uu1;
  double g0;
  double g1;
  ///////////////////////
  double LHP_ctrl;
  double LK_ctrl;

  
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

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

double SETPT = 0;

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
    
   // double M[2][2]={0};
   // M[0][0]=1;
   // M[0][1]=2;
    //M[1][0]=3;
   // M[1][1]=4;


    //double Minv[2][2]={0};

  //matInv2(2, M, Minv);
  //atPrint(2,2,Minv);

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


    //int j = 26;
    //double ctrl = -100*(d->qpos[26]-0.75);
    //double ctrl = -100*(d->qpos[L_elbow_joint_adr]-0.75);
    
    //double ctrl = -100*(d->sensordata[16]-0.5);
    //printf("%f %f %f\n",d->qpos[j],d->sensordata[16],d->sensordata[43]);
    //d->ctrl[15] = ctrl;
   


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

    printf("The knee joint PD torque is " );
    printf("%lf", LK_ctrl*16);
    printf("\n");

    printf("The knee joint start controller torque is " );
    printf("%lf", uu1);
    printf("\n");

    printf("The knee joint gravity torque is " );
    printf("%lf", g1);
    printf("\n");

    printf("The hip joint PD torque is " );
    printf("%lf", LHP_ctrl*16);
    printf("\n");
    printf("\n");

    printf("The hip joint start controller torque is " );
    printf("%lf", uu0);
    printf("\n");

    printf("The hip joint gravity torque is " );
    printf("%lf", g0);
    printf("\n");
    printf("%lf",msd);
    printf("\n");


    //printf("inertia matrix: ");
      //mju_printMat(d->qM,10,10);





    theta1 = -1*d->sensordata[LHP_sensor_adr]+a1_offset*(M_PI/180);
    theta2 = d->sensordata[LK_sensor_adr]-a2_offset*(M_PI/180);
    omega1 = -1*d->sensordata[LHP_vel_sensor_adr];
    omega2 = d->sensordata[LK_vel_sensor_adr];

    if (msd>=tf && counter ==0)
    {
      s0[0][0] = -1*d->sensordata[LHP_sensor_adr]+a1_offset*(M_PI/180);
      s0[0][1] = d->sensordata[LK_sensor_adr]-a2_offset*(M_PI/180);
      v0[0][0] = -1*d->sensordata[LHP_vel_sensor_adr];
      v0[0][1] = d->sensordata[LK_vel_sensor_adr];
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      sf[0][0] = (a1_offset-10)*(M_PI/180);
      //sf[0][0] = s0[0][0];
      sf[0][1] = (a2_offset-15)*(M_PI/180);
      //sf[0][1] = s0[0][1];
      edt[0][0]=0; //accumulated error for integral control
      edt[0][1]=0;

      counter = 1;
      plotcounter=0;
      j=0;
      printf("inertia matrix: ");
      mju_printMat(d->qM,5,5);
      //mjtNum* dense_M = mj_stackAlloc(d, m->nv*m->nv); // Allocate array of specified size on mjData stack. Call mju_error on stack overflow. MJAPI mjtNum* mj_stackAlloc(mjData* d, int size);
    //mj_fullM(m, dense_M, d->qM);//MJAPI void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);
    //mju_printMat(dense_M, m->nv, m->nv);
    }
    if (msd>=2*tf && counter ==1)
    {
      s0[0][0] = -1*d->sensordata[LHP_sensor_adr]+a1_offset*(M_PI/180);
      s0[0][1] = d->sensordata[LK_sensor_adr]-a2_offset*(M_PI/180);
      v0[0][0] = -1*d->sensordata[LHP_vel_sensor_adr];
      v0[0][1] = d->sensordata[LK_vel_sensor_adr];
      a0[0][0]=0; //TODO get accelration from motor
      a0[0][1]=0;
      sf[0][0] = (a1_offset+25)*(M_PI/180);
      //sf[0][0] = s0[0][0];
      sf[0][1] = (a2_offset+15)*(M_PI/180);
      //sf[0][1] = s0[0][1];
      edt[0][0]=0;
      edt[0][1]=0;
  

      counter = 2;
    }
    printf("%d \n", counter);



    

    if(msd<tf){

      //plotcounter=plotcounter+1;
      LHP_ctrl = (-600*(d->sensordata[LHP_sensor_adr]+25*(M_PI/180))/16)-(-1*omega1);
      d->ctrl[LHP_actuatorID] = LHP_ctrl;
      LK_ctrl = (-600*(d->sensordata[LK_sensor_adr]-15*(M_PI/180))/16)-(1*omega2);
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
      c_controller4(uu,z,params,traj_des); 

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

      d->ctrl[LHP_actuatorID] = -1*uu[0]/16.0;
      d->ctrl[LK_actuatorID] = uu[1]/16.0;
      if(j==1){
        uu0 = uu[0];
        uu1=uu[1];
        g0 = d->qfrc_bias[LHP_joint_adr];
        g1 = d->qfrc_bias[LK_joint_adr];
      }


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
      c_controller4(uu,z,params,traj_des); 
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

      d->ctrl[LHP_actuatorID] = -1*uu[0]/16.0;
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

       

        matPlot2(xx,yy1,xx,yy2,j,"closed_chain_control_figures/theta1vstraj_des_KP120_01.png",L"Hip Angle Position", L"time (s)",L"Position (rad)");
        matPlot2(xx,yy3,xx,yy4,j,"closed_chain_control_figures/theta2vstraj_des_KP120_01.png",L"Knee Angle Position",L"time (s)",L"Position (rad)");
        matPlot2(xx,yt1,xx,yg1,j,"closed_chain_control_figures/hip_torque_and_gravity_torque.png",L"Hip Control (red) Gravity Torque (blue)",L"time (s)",L"Torque (N-m)");
        matPlot2(xx,yt2,xx,yg2,j,"closed_chain_control_figures/knee_torque_and_gravity_torque.png",L"Knee Control (red) Gravity (blue)",L"time (s)",L"Torque (N-m)");
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

     // activate software
    mj_activate("../../../mjkey.txt");


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML("../model/closed_chain2.xml", 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // install control callback
    mjcb_control = mycontroller;

    // initial position
    d->qpos[0] = SETPT;

    // run main loop, target real-time simulation and 60 fps rendering
    mjtNum timezero = d->time;
    double_t update_rate = 0.01;

    // making sure the first time step updates the ctrl previous_time
    last_update = timezero-1.0/ctrl_update_freq;

    double arr_view[] = {90,-21.6,2.57,0,0,0.39};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance =arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] =arr_view[4];
    cam.lookat[2] = arr_view[5];

    // use the first while condition if you want to simulate for a period.
//    while( !glfwWindowShouldClose(window) and d->time-timezero < 1.5)
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);

        // 15 ms is a little smaller than 60 Hz.
        usleep(15); //usleep(useconds_t usec);
        //std::this_thread::sleep_for(std::chrono::milliseconds(15));
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
        //printf("%f %f %f %f %f %f\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);


    }

    // free visualization storage
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
