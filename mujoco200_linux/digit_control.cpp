/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include <iostream>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


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


    // printouts for debugging purposes
    //std::cout << "number of position coordinates: " << m->nq << std::endl;
    //std::cout << "number of degrees of freedom: " << m->nv << std::endl;
    //std::cout << "number of joints: " << m->njnt << std::endl;
    //std::cout << "joint position: " << d->qpos[0] << std::endl;
    //std::cout << "joint velocity: " << d->qvel[0] << std::endl;
    //std::cout << "Sensor output: " << d->sensordata[0] << std::endl;

    //printf("%d \n",m->nq);
    //printf("%f \n",d->qpos[17]);
    const char* joint_name = "left-hip-pitch";
    int L_elbow_sensorID = mj_name2id(m, mjOBJ_SENSOR, joint_name);
    int L_elbow_sensor_adr = m->sensor_adr[L_elbow_sensorID];
    int L_elbow_actuatorID = mj_name2id(m, mjOBJ_ACTUATOR, joint_name);
    int L_elbow_jointID = mj_name2id(m, mjOBJ_JOINT, joint_name);
    int L_elbow_joint_adr = m->jnt_qposadr[L_elbow_jointID];
    //int j = 26;
    //double ctrl = -100*(d->qpos[26]-0.75);
    //double ctrl = -100*(d->qpos[L_elbow_joint_adr]-0.75);
    double ctrl = -150*(d->sensordata[L_elbow_sensor_adr]-0.75);
    //double ctrl = -100*(d->sensordata[16]-0.5);
    //printf("%f %f %f\n",d->qpos[j],d->sensordata[16],d->sensordata[43]);
    //d->ctrl[15] = ctrl;
    d->ctrl[L_elbow_actuatorID] = ctrl;

    printf("position = %f \n",d->qpos[L_elbow_joint_adr]);
    //for (int i=0;i<=22;i++)
      printf("%f \n",d->sensordata[L_elbow_sensor_adr]);

      //printf("\n");

/*
    // controller with true values, but it is cheating.
//    ctrl = 3.5*(-d->qvel[0]-10.0*d->qpos[0]);

    // controller with sensor readings
    if (previous_time == 0)
    {
        previous_time = d->time;
        return;
    }
    if (d->time - last_update > 1.0/ctrl_update_freq)
    {
        mjtNum vel = (d->sensordata[0] - position_history)/(d->time-previous_time);
        ctrl = 3.5*(-vel-10.0*d->sensordata[0]);
        last_update = d->time;
        position_history = d->sensordata[0];
        previous_time = d->time;
    }
    d->ctrl[0] = ctrl;

    std::cout << "torque effort: " << ctrl << std::endl;*/
}

// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // activate software
    mj_activate("mjkey.txt");

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
