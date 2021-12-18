// /*
//  * This file belong to https://github.com/snolfi/evorobotpy
//  * Author: Stefano Nolfi, stefano.nolfi@istc.cnr.it

//  * predprey.cpp, include an implementation of the ErPredprey environment

//  * This file is part of the python module ErPredprey.so that include the following files:
//  * predprey.cpp, predprey.h, robot-env.cpp, robot-env.h, utilities.cpp, utilities.h, ErPredprey.pxd, ErPredprey.pyx and setupErPredprey.py
//  * And can be compiled with cython and installed with the commands: cd ./evorobotpy/lib; python3 setupErPredprey.py build_ext –inplace; cp ErPredprey*.so ../bin
//  */

// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>
// #include <time.h>
// #include <ctype.h>
// #include <string.h>
// #include <locale.h>
#include "predprey.h"
// #include "utilities.h"
// #include "robot-env.h"

// // the current step
// int cstep;
// int steps = 1000;
// // read steps from file
// void readStepsFromConfig();

// // Pointer to the observations
// float* cobservation;
// // Pointer to the actions
// float* caction;
// // Pointer to termination flag
// int* cdone;
// // Pointer to world objects to be rendered
// double* dobjects;

// int initCameraPPSensor(struct robot *cro);
// void updateCameraPPSensor(struct robot *cro);
// int initGroundGradSensor(struct robot *cro);
// void updateGroundGradSensor(struct robot *cro);
// /*
//  * env constructor
//  */
// Problem::Problem()
// {

//     struct robot *ro;
//     int r;

//     // set USA local conventions
//     setlocale( LC_ALL, "en-US" );
//     // read task parameters
// 	nrobots = 2;  // number of robots
//     // read steps from .ini file
//     readStepsFromConfig();
// 	// create the environment
// 	initEnvironment();
//     // creates the list of robot structures that contain the robot data
//     // the first is the predator and the second is the prey
//     rob = (struct robot *) malloc(nrobots * sizeof(struct robot));
//     //
//     // the initRobot function is defined in the robot-env.cpp file
// 	for (r=0, ro=rob; r < nrobots; r++, ro++)
//     {
		
// 	   initRobot(ro, r, MarXBot);             // initilize robot variable (e.g. the size of the robots' radius, ext).
//         // set predators and prey properties
//         if (r == 0)
//          {
//             ro->color = 1;  // predators and red
//             ro->maxSpeed = 500.0 * 0.80;  // predator is 20% slower
//          }
//         else
//         {
//             ro->color = 3;  // prey are green
//             ro->maxSpeed = 500.0;
//         }
	
// 	   ninputs = 0;
//        ninputs += initInfraredSensor(ro);     // infrared sensor (8 units)
//        ninputs += initCameraPPSensor(ro);
//        ninputs += initGroundGradSensor(ro);
//        ninputs += initTimeSensor(ro);
//        ninputs += initBiasSensor(ro);
        
// 	   ro->sensorinfraredid = 0; 			  // the id of the first infrared sensors (used for graphic purpose only to visually siaplay infrared activity)

// 	   initRobotSensors(ro, ninputs);        // allocate and initialize the robot->sensor vector that contain net->ninputs values and is passed to the function that update the state of the robots' network
		
//        ro->motorwheels = 2;                   // define the motors used and set the number of motor neurons
//        ro->motorleds = 0;                     // motorleds can be set to 1,2, or 3
// 	   ro->motorwheelstype = 0;               // direct controls of the wheels
//        noutputs = ro->motorwheels + ro->motorleds;
		
//        ro->motorwheelsid = ninputs + 10; // +net->nhiddens;
//        ro->motorledsid = 0;
// 	}
	
// 	rng = new RandomGenerator(time(NULL));
	
// }


// Problem::~Problem()
// {
// }


// /*
//  * set the seed
//  */
// void Problem::seed(int s)
// {
//     	rng->setSeed(s);
// }

// /*
//  * reset the initial condition randomly
//  * when seed is different from 0, reset the seed
//  */
// void Problem::reset()
// {
    
//     struct robot *ro;
//     int r;

//     cstep = 0;
//     // initialize robots position, orientation
//     for (r=0, ro=rob; r < nrobots; r++, ro++)
//     {
//         if (r == 0)
//         {
//             ro->x = worldx / 3.0;
//             ro->y = worldy / 2.0;
//             ro->dir = 0.0;
//         }
//         else
//         {
//             ro->x = worldx / 3.0 * 2.0;
//             ro->y = worldy / 2.0;
//             ro->dir = M_PI;
//         }
//     }

// 	// Get observations
// 	getObs();
	
// }


// void Problem::copyObs(float* observation)
// {
// 	cobservation = observation;
// }

// void Problem::copyAct(float* action)
// {
// 	caction = action;
// }

// void Problem::copyDone(int* done)
// {
// 	cdone = done;
// }

// void Problem::copyDobj(double* objs)
// {
// 	dobjects = objs;
// }

// /*
//  * update observation vector
//  * that contains the predator and prey observation state
//  */
// void Problem::getObs()
// {
    
//     struct robot *ro;
//     int r;
//     int u;
//     int s;
    
//     u = 0;
//     for (r=0, ro=rob; r < nrobots; r++, ro++)
//     {
//         ro->csensors = ro->sensors;
//         updateInfraredSensor(ro);
//         updateCameraPPSensor(ro);
//         updateGroundGradSensor(ro);
//         updateTimeSensor(ro, cstep, steps);
//         updateBiasSensor(ro);
//         for(s=0, ro->csensors = ro->sensors; s < ninputs; s++, u++, ro->csensors++)
//             cobservation[u] = *ro->csensors;
//     }
    
// }

// /*
//  * perform the action, update the state of the environment, update observations, return the predator's reward
//  */
// double Problem::step()
// {

//     double dx, dy;
//     int x, y;
//     double dist;
//     struct robot *ro;
//     int r;
//     double reward;
//     float *cacti;
//     double angle1, angle2;
	
//     cstep++;
//     *cdone = 0;
//     reward = 0.0;
//     cacti = caction;
//     for (r=0, ro=rob; r < nrobots; r++, ro++)
//     {
      
// 	  if (updateRobot(ro, cacti) > 0)
//         {
// 	  	 *cdone = 1;
//          reward = 1.0 - ((double) cstep / (double) steps);
//         }
        
//       if (r < (nrobots - 1))
//          cacti = (cacti + noutputs);
//     }
    
// 	getObs();
    
//     return(reward);

// }

// int Problem::isDone()
// {
// 	return *cdone;
// }

// void Problem::close()
// {
//     	//printf("close() not implemented\n");
// }

// /*
//  * initialize the environment
//  */
// void
// Problem::initEnvironment()

// {
    
//     int cobj=0;
    
//     nenvobjs = 4;    // total number of objects
//     worldx = 3000;
//     worldy = 3000;
//     int f;
    
//     envobjs = (struct envobjects *) malloc(nenvobjs * sizeof(struct envobjects));
    
//     envobjs[cobj].type = WALL;
//     envobjs[cobj].x = 0.0;
//     envobjs[cobj].y = 0.0;
//     envobjs[cobj].x2 = worldx;
//     envobjs[cobj].y2 = 0.0;
//     cobj++;
//     envobjs[cobj].type = WALL;
//     envobjs[cobj].x = 0.0;
//     envobjs[cobj].y = 0.0;
//     envobjs[cobj].x2 = 0.0;
//     envobjs[cobj].y2 = worldy;
//     cobj++;
//     envobjs[cobj].type = WALL;
//     envobjs[cobj].x = worldx;
//     envobjs[cobj].y = 0.0;
//     envobjs[cobj].x2 = worldx;
//     envobjs[cobj].y2 = worldy;
//     cobj++;
//     envobjs[cobj].type = WALL;
//     envobjs[cobj].x = 0.0;
//     envobjs[cobj].y = worldy;
//     envobjs[cobj].x2 = worldx;
//     envobjs[cobj].y2 = worldy;
//     cobj++;
    
//     if (cobj > nenvobjs)
//     {
//         printf("ERROR: you should allocate more space for environmental objects");
//         fflush(stdout);
//     }
    
// }


// /*
//  * create the list of robots and environmental objects to be rendered graphically
//  */
// void Problem::render()
// {
//     int i;
//     int c;
//     struct robot *ro;
//     double scale = 0.12;
    
//     c=0;
//     // robots
//     for (i=0, ro = rob; i < nrobots; i++, ro++)
//     {
//         dobjects[c] = 1.0;
//         dobjects[c+1] = ro->x * scale;
//         dobjects[c+2] = ro->y * scale;
//         dobjects[c+3] = ro->radius * scale;
//         dobjects[c+4] = 0.0;
//         if (i == 0)
//           {
//             dobjects[c+5] = 1.0; //ro->rgbcolor[0];
//             dobjects[c+6] = 0.0; //ro->rgbcolor[1];
//             dobjects[c+7] = 0.0; // ro->rgbcolor[2];
// 		  }
// 		 else
//           {
//             dobjects[c+5] = 0.0; //ro->rgbcolor[0];
//             dobjects[c+6] = 1.0; //ro->rgbcolor[1];
//             dobjects[c+7] = 0.0; // ro->rgbcolor[2];
// 		  }
//         dobjects[c+8] = (ro->x + xvect(ro->dir, ro->radius)) * scale;
//         dobjects[c+9] = (ro->y + yvect(ro->dir, ro->radius)) * scale;
//         c += 10;
//     }
//     for(i=0; i < nenvobjs; i++)
//     {
//         switch(envobjs[i].type)
//         {
//             case SAMPLEDSCYLINDER:
//                 dobjects[c] = 3.0;
//                 dobjects[c+3] = envobjs[i].r;
//                 dobjects[c+4] = 0.0;
//                 dobjects[c+8] = 0.0;
//                 dobjects[c+9] = 0.0;
//                 break;
//             case WALL:
//                 dobjects[c] = 2.0;
//                 dobjects[c+3] = envobjs[i].x2 * scale;
//                 dobjects[c+4] = envobjs[i].y2 * scale;
//                 dobjects[c+8] = 0.0;
//                 dobjects[c+9] = 0.0;
//                 break;
//         }
//         dobjects[c+1] = envobjs[i].x * scale;
//         dobjects[c+2] = envobjs[i].y * scale;
//         dobjects[c+5] = envobjs[i].color[0];
//         dobjects[c+6] = envobjs[i].color[1];
//         dobjects[c+7] = envobjs[i].color[2];
//         c += 10;
//     }
//     dobjects[c] = 0.0;
    
    
// }

// /*
//  * reads parameters from the configuration file
//  */
// void readStepsFromConfig()
// {
//     char *s;
//     char buff[1024];
//     char name[1024];
//     char value[1024];
//     char *ptr;
//     int section;  // 0=before the section 1=in the section 2= after the section
    
//     section = 0;
    
//     FILE* fp = fopen("ErPredprey.ini", "r");
//     if (fp != NULL)
//     {
//         // Read lines
//         while (fgets(buff, 1024, fp) != NULL)
//         {
            
//             //Skip blank lines and comments
//             if (buff[0] == '\n' || buff[0] == '#' || buff[0] == '/')
//                 continue;
            
//             //Parse name/value pair from line
//             s = strtok(buff, " = ");
//             if (s == NULL)
//                 continue;
//             else
//                 copyandclear(s, name);
            
//             s = strtok(NULL, " = ");
//             if (s == NULL)
//                 continue;
//             else
//                 copyandclear(s, value);
            
//             // Copy into correct entry in parameters struct
//             if (strcmp(name, "maxsteps")==0)
//                 steps = (int)strtol(value, &ptr, 10);
//             //else printf("WARNING: Unknown parameter %s \n", name);
//         }
//         fclose(fp);
//     }
//     else
//     {
//         printf("ERROR: unable to open file ErForaging.ini\n");
//         fflush(stdout);
//     }
// }

// /*
//  *    initialize the PP camera sensor
//  */
// int initCameraPPSensor(struct robot *cro)
// {
//     if (cro->idn == 0) printf("Camera[%d]: single color, 8 sectors and distance \n", 9);
//     return(9);
// }


// /*
//  *    update the omidirectional camera sensor
//  *  assume that the environment include a single coloured object constituted by the other robot
//  *  encode angular centre of the color blob produced by the other robot withon 8 sectors and the average fraction of pixel stimulated
//  */
// void updateCameraPPSensor(struct robot *cro)

// {
    
//     double dx, dy, dist, angle;
//     double angleb;
//     int n,r,s;
//     struct robot *ro;
//     double act[8];
//     double adist;
//     double sectoracenter;
//     double pi8;
//     double pi16;
    
//     pi8 = PI2 / 8.0;
//     pi16 = PI2 / 16.0;
//     for (n=0; n < 8; n++)
//         act[n] = 0.0;
//     for (r=0, ro=rob; r < nrobots; r++, ro++)
//     {
//         if (cro->idn != ro->idn)
//         {
//             dx = (cro->x - ro->x);
//             dy = (cro->y - ro->y);
//             dist = sqrt(dx*dx+dy*dy);
//             angle = mangrelr(angv(ro->x, ro->y, cro->x, cro->y), cro->dir);
//             for (s=0, sectoracenter = 0.0 + (PI2 / 16.0); s < 8; s++, sectoracenter += pi8)
//             {
//                 angleb = angle;
//                 if (s == 0 && angle > M_PI)
//                     angleb = angle - PI2;
//                 if (s == 7 && angle < M_PI)
//                     angleb = angle + PI2;
//                 if (fabs(angleb - sectoracenter) < pi8)
//                 {
//                     act[s] = 1.0 - ((fabs(angleb - sectoracenter) / pi8));
//                 }
//                 //printf("s %d ang %.2f secta %.2f fabs %.2f -> %.2f\n", s, angle, sectoracenter, fabs(angleb - sectoracenter), act[s]);
//             }
            
//         }
//     }
//     // perceived angular blob (maximum value il 0.93 radiants for two adjacent marxbots)
//     adist = ((M_PI / 2.0) - atan(dist/cro->radius)) * 2.0;
//     //printf("dist %.2f angular blob %.2f\n", dist, adist);
    
//     // normalize and copy on the imput vectorc
//     for(n=0; n < 8; n++)
//     {
//         *cro->csensors = act[n];
//         //printf("%.2f\n", act[n]);
//         cro->csensors++;
//     }
//     *cro->csensors = adist;
//     //printf("%.2f\n", adist);
//     cro->csensors++;
//     //printf("end\n");
// }

// /*
//  *    initialize the ground gradient sensor
//  */
// int initGroundGradSensor(struct robot *cro)
// {
//     return(5);
// }
// /*
//  *    update the marXbot ground gradient sensors
//  *  assume that the color of the area is white at the centre and progressively darker in the periphery
//  *  the last value encode the average color detected by the four sensors located at the frontal-left/right and at the back-left/right
//  *  the first four values variation of the with respect to the average color
//  */
// void updateGroundGradSensor(struct robot *cro)

// {
//     double act[5];
//     double gcolor[4];
//     double x,y;
//     double dx, dy;
//     double average;
//     double wx, wy;
//     double maxdist;
//     int s, n;
//     double sensordist;
    
//     wx = worldx / 2.0;
//     wy = worldy / 2.0;
//     maxdist = sqrt((wx*wx)+(wy*wy));
//     sensordist = cro->radius * 0.9;
    
//     //front left
//     x = cro->x + xvect(cro->dir - 0.0872 , sensordist);
//     y = cro->y + yvect(cro->dir - 0.0872 , sensordist);
//     dx = x - wx;
//     dy = y - wy;
//     gcolor[0] = (sqrt((dx*dx)+(dy*dy)) / maxdist);
//     //front right
//     x = cro->x + xvect(cro->dir + 0.0872 , sensordist);
//     y = cro->y + yvect(cro->dir + 0.0872 , sensordist);
//     dx = x - wx;
//     dy = y - wy;
//     gcolor[1] = (sqrt((dx*dx)+(dy*dy)) / maxdist);
//     //rear left
//     x = cro->x + xvect(cro->dir - 0.0872 + M_PI, sensordist);
//     y = cro->y + yvect(cro->dir - 0.0872 + M_PI, sensordist);
//     dx = x - wx;
//     dy = y - wy;
//     gcolor[2] = (sqrt((dx*dx)+(dy*dy)) / maxdist);
//     //rear right
//     x = cro->x + xvect(cro->dir + 0.0872 + M_PI, sensordist);
//     y = cro->y + yvect(cro->dir + 0.0872 + M_PI, sensordist);
//     dx = x - wx;
//     dy = y - wy;
//     gcolor[3] = (sqrt((dx*dx)+(dy*dy)) / maxdist);
    
//     // average value
//     act[4] = average = (gcolor[0] + gcolor[1] + gcolor[2] + gcolor[3]) / 4.0;
//     // variations
//     for(s=0; s < 4; s++)
//         act[s] = 0.5 + (gcolor[s] - average) * 10.0;
    
//     // normalize and copy on the imput vector
//     for(n=0; n < 5; n++)
//     {
//         *cro->csensors = act[n];
//         cro->csensors++;
//     }
    
    
// }


/*
 * This is a cleaned version of predprey.cpp
 * This file is part of the python module ErPredprey.so that include the following files:
 * And can be compiled with cython and installed with the commands: cd ./evorobotpy/lib; python3 setupErPredprey.py build_ext –inplace; cp ErPredprey*.so ../bin
 */

#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <string.h>
#include <cmath>
#include <locale.h>
// #include "Flock.h"
#include <vector>
#include "Boid.h"
#include "Vector2D.h"
// Pointer to the observations
float* cobservation;
// Pointer to the actions
float* caction;
// Pointer to termination flag
int* cdone;
// Pointer to world objects to be rendered
double* dobjects;


float x = 0.0;
float y = 0.0;
int window_width = 600;
int window_height = 600;
float max_speed = 6;
float max_force = 1;
float acceleration_scale = 0.3;
float cohesion_weight = 0.75;
float alignment_weight = 0.65;
float separation_weight = 4.5;
float perception = 100;
float separation_distance = 20;
float noise_scale = 0;
bool is_predator = false;

/*
 * env constructor
 */
Problem::Problem()
{


    // set USA local conventions
    setlocale( LC_ALL, "en-US" );
    ninputs = 2;              // number of sensory neurons of each boids REPLACE 5 WITH THE RIGHT VALUE
    noutputs = 3;             // number of motor neurons of each boids REPLACE 2 WITH THE RIGHT VALUE

}


Problem::~Problem()
{

}


/*
 * set the seed
 */
void Problem::seed(int s)
{
    s = 5;
}

/*
 * reset the initial condition randomly
 * 
 */
int stepNum = 0;
int n_bot = 5; 
void Problem::reset()
{
    boids.clear();
    Vector2D acceleration = (Vector2D::random() - 0.5) * 0 * 2;
    add_boid(500,500,acceleration.x,acceleration.y,false);
    acceleration = (Vector2D::random() - 0.5) * 0.0* 2;
    add_boid(550,550,acceleration.x,acceleration.y,false);
    acceleration = (Vector2D::random() - 0.5) * 0.0 * 2;
    add_boid(600,600,acceleration.x,acceleration.y,false);
    acceleration = (Vector2D::random() - 0.5) * 0.0 * 2;
    add_boid(700,700,acceleration.x,acceleration.y,false);

    add_boid(200,450,acceleration.x,acceleration.y,true);
    getObs();
    // for(int i = 0; i < 10; i++){
    //     // caction[i] = 0;
    // }
    // printf("reset=====================================\n");
    stepNum = 0;
}


void Problem::copyObs(float* observation)
{
cobservation = observation;
}

void Problem::copyAct(float* action)
{
caction = action;
printf("copyAct");
}

void Problem::copyDone(int* done)
{
cdone = done;
}

void Problem::copyDobj(double* objs)
{
dobjects = objs;
}



/*
 * perform the action, update the state of the environment, update observations, return the predator's reward
 */
// ==================================================================previous step func
    //                     double Problem::step()
    //                     {
    //                         double reward = 0;
    //                         /*
    //                         use the *caction vector to change 
    //                         the direction and the positions of the boids, 
    //                         you should update the observation vector, 
    //                         you should compute the reward (a scalar 
    //                         floating point value) and return the reward
    //                         0 - nothing, 1 - left, -1 - right
    //                         */
    //                         // for(int i = 0; i < 2; i++){
    //                         //     Boid b1 = boids[i];
    //                         //     for(int j = i+1; j < 3; j++){
    //                         //         Boid b2 = boids[j];
    //                         //         float distance = sqrt(pow((b1.posX() - b2.posX()),2) + pow((b1.posY() - b2.posY()),2));
    //                         //         printf("distance = %f\n",distance);
    //                         //         if(distance < 150){
    //                         //             printf("run\n");
    //                         //             b1.position = Vector2D(b2.posX()+11,b2.posY()+1);
    //                         //         }
    //                         //     }
    //                         // }
    //                         // Boid b1 = boids[0];
    //                         // Boid b2 = boids[1];
    //                         // printf("step = %d, %f %f ",stepNum,b1.posX(),b1.posY());

    //                         // =================================================================================================================
    //                         // printf("distance = %f",distance);
    //                         int predator_idx = 0;
    //                         for(int i = 0; i < 5; i++){
    //                             if(boids[i].is_predator){
    //                                 predator_idx = i;                
    //                             }
    //                         }
    //                         int closest_boid = 0;
    //                         double min_dist = 1000000000;
    //                         for(int i = 0; i < 5; i++){
    //                             for (int j = 0; j < 5; j++){
    //                                 if(!boids[j].is_predator){ // was i
    //                                     float dist = sqrt(pow(boids[predator_idx].posX() - boids[j].posX(), 2) + pow(boids[predator_idx].posY() - boids[j].posY(), 2)); 
    //                                     if(dist < min_dist){
    //                                         min_dist = dist;
    //                                         closest_boid = j;
    //                                     }

    //                                 }       
    //                             }
    //                         }
    // boids[predator_idx].velocity += (Vector2D::random() - 0.5) * boids[predator_idx].max_speed;
    // // Limit the velocity so the boids don't get too fast
    // boids[predator_idx].velocity.limit(boids[predator_idx].max_speed);
    // // Then update the position based on the velocity
    // boids[predator_idx].position += boids[predator_idx].velocity;
    // if (boids[predator_idx].position.x < 0) boids[predator_idx].position.x += boids[predator_idx].max_width;
    // if (boids[predator_idx].position.y < 0) boids[predator_idx].position.y += boids[predator_idx].max_height;
    // if (boids[predator_idx].position.x >= boids[predator_idx].max_width) boids[predator_idx].position.x -= boids[predator_idx].max_width;
    // if (boids[predator_idx].position.y >= boids[predator_idx].max_height) boids[predator_idx].position.y -= boids[predator_idx].max_height;



    //                         // boids[predator_idx].acceleration *= acceleration_scale;
    //                         // boids[predator_idx].acceleration.limit(boids[predator_idx].max_force);
    //                         // boids[predator_idx].velocity += boids[predator_idx].acceleration;
    //                         // if (boids[predator_idx].noise_scale != 0)
    //                         //     boids[predator_idx].velocity += (Vector2D::random() - 0.5) * boids[predator_idx].noise_scale;
    //                         // // Limit the velocity so the boids don't get too fast
    //                         // boids[predator_idx].velocity.limit(boids[predator_idx].max_speed);
    //                         // // Then update the position based on the velocity
    //                         // boids[predator_idx].position += boids[predator_idx].velocity;
    //                         // if (boids[predator_idx].position.x < 0) boids[predator_idx].position.x += boids[predator_idx].max_width;
    //                         // if (boids[predator_idx].position.y < 0) boids[predator_idx].position.y += boids[predator_idx].max_height;
    //                         // if (boids[predator_idx].position.x >= boids[predator_idx].max_width) boids[predator_idx].position.x -= boids[predator_idx].max_width;
    //                         // if (boids[predator_idx].position.y >= boids[predator_idx].max_height) boids[predator_idx].position.y -= boids[predator_idx].max_height;


    //                         double max_distance = 1500;
                            
    //                         float* cacti = caction;
    //                         for(int i = 0; i < 5; i++){
    //                             if(!boids[i].is_predator){
                                    
    //                                 // printf("caction[%d] = %f, caction[%d] = %f " ,2*i,caction[2*i],2*i+1,caction[2*i+1]);
    //                                 // boids[i].velocity.x = 5*cos(caction[i*2]);
    //                                 // boids[i].velocity.y = 5*sin(caction[i*2 + 1]);
    //                                 boids[i].velocity.x = 10*cos(caction[i*2]) + 5*Vector2D::random().x; 
    //                                 boids[i].velocity.y = 10*sin(caction[i*2 + 1]) + 5*Vector2D::random().x; 
    //                                 // printf("%d vel %f %f action %f %f\n",i,boids[i].velocity.x,boids[i].velocity.y, caction[i*2], caction[i*2 +1]);

    //                                 // printf("agnle from step %f", boids[i].angle());
                                    
    //                                 // printf("Velocity %f", boids[i].velocity.x);
    //                                 // boids[i].velocity.limit(boids[i].max_speed);
    //                                 // printf("cactions = %d", size(caction));

    //                                 // // printf("Action %f\n",caction[i]);
    //                                 // if (caction[i] > 0){
    //                                 //     // boids[i].velocity += Vector2D{5,-5};
    //                                 //     boids[i].velocity.x += 0.05;
    //                                 //     boids[i].velocity.y -= 0.05;
    //                                 //     boids[i].velocity.limit(boids[i].max_speed);

    //                                 //     // printf("%d action = %f\n",i,caction[i]);
    //                                 //     // printf("Action1");
    //                                 // }
    //                                 // if (caction[i] <= 0){
    //                                 //     boids[i].velocity.x -= 0.005;
    //                                 //     boids[i].velocity.y += 0.005;
                                        
    //                                 //     boids[i].velocity.limit(boids[i].max_speed);
    //                                 //     // printf("Action2");
    //                                 //     // printf("%d action = %f\n",i,caction[i]);
    //                                 // }
    //                                 // // Limit the velocity so the boids don't get too fast
    //                                 float distance = sqrt(pow((boids[i].posX() - boids[predator_idx].posX()),2) + pow((boids[i].posY() - boids[predator_idx].posY()),2));
    //                                 reward += distance/max_distance;
    //                                 // printf("reward = %f\n",reward);
    //                                 // Then update the position based on the velocity
    //                                 boids[i].position += boids[i].velocity;
    //                                 if (boids[i].position.x < 0) boids[i].position.x += boids[i].max_width;
    //                                 if (boids[i].position.y < 0) boids[i].position.y += boids[i].max_height;
    //                                 if (boids[i].position.x >= boids[i].max_width) boids[i].position.x -= boids[i].max_width;
    //                                 if (boids[i].position.y >= boids[i].max_height) boids[i].position.y -= boids[i].max_height;
                            
    //                             }

    //                         }
    //                         // printf("\n");
    //                         // if(stepNum %200 == 0)
    //                         //     printf("reward = %f\n", reward);
















    //                         // boids[closest_boid].acceleration *= acceleration_scale;
    //                         // boids[closest_boid].acceleration.limit(boids[closest_boid].max_force);
    //                         // boids[closest_boid].velocity += boids[closest_boid].acceleration;
    //                         // if (boids[closest_boid].noise_scale != 0)
    //                         //     boids[closest_boid].velocity += (Vector2D::random() - 0.5) * boids[closest_boid].noise_scale;
    //                         // // Limit the velocity so the boids don't get too fast
    //                         // boids[closest_boid].velocity.limit(boids[closest_boid].max_speed);
    //                         // // Then update the position based on the velocity
    //                         // boids[closest_boid].position += boids[closest_boid].velocity;
    //                         // if (boids[closest_boid].position.x < 0) boids[closest_boid].position.x += boids[closest_boid].max_width;
    //                         // if (boids[closest_boid].position.y < 0) boids[closest_boid].position.y += boids[closest_boid].max_height;
    //                         // if (boids[closest_boid].position.x >= boids[closest_boid].max_width) boids[closest_boid].position.x -= boids[closest_boid].max_width;
    //                         // if (boids[closest_boid].position.y >= boids[closest_boid].max_height) boids[closest_boid].position.y -= boids[closest_boid].max_height;
    //                         // // Set the acceleration to zero before the next update
    //                         // acceleration = 0;







    //                         // for(int i = 0; i < 5; i++){
    //                         //     if(i != closest_boid && !boids[i].is_predator){
    //                         //         if (boids[i].noise_scale != 0)
    //                         //             boids[i].velocity += (Vector2D::random() - 0.5) * boids[i].noise_scale;
    //                         //         // Limit the velocity so the boids don't get too fast
    //                         //         boids[i].velocity.limit(boids[i].max_speed);
    //                         //         // Then update the position based on the velocity
    //                         //         boids[i].position += boids[i].velocity;
    //                         //         if (boids[i].position.x < 0) boids[i].position.x += boids[i].max_width;
    //                         //         if (boids[i].position.y < 0) boids[i].position.y += boids[i].max_height;
    //                         //         if (boids[i].position.x >= boids[i].max_width) boids[i].position.x -= boids[i].max_width;
    //                         //         if (boids[i].position.y >= boids[i].max_height) boids[i].position.y -= boids[i].max_height;
                            
    //                         //     }
    //                         // }











    //                         // if(distance < 150){
    //                         //     // printf("run \n");

    //                         //     // Scale the acceleration then use it to update the velocity
    //                         // // if (is_predator)
    //                         // //     acceleration *= PREDATOR_ACCELERATION_BOOST;
    //                         // boids[0].acceleration *= acceleration_scale;
    //                         // boids[0].acceleration.limit(max_force);
    //                         // boids[0].velocity += boids[0].acceleration;
    //                         // if (noise_scale != 0)
    //                         //     boids[0].velocity += (Vector2D::random() - 0.5) * noise_scale;
    //                         // // Limit the velocity so the boids don't get too fast
    //                         // boids[0].velocity.limit(max_speed);
    //                         // // Then update the position based on the velocity
    //                         // boids[0].position += boids[0].velocity;
    //                         // // printf("accel = %f, %f", boids[0].accX(),boids[0].accY());
    //                         // // Set the acceleration to zero before the next update
    //                         // boids[0].acceleration = 0;
    //                         //     // b1.position = Vector2D(b2.posX()+11,b2.posY()+1);
    //                         //     // boids[0].update(boids);
    //                         // }
    //                         // // boids[0].position = Vector2D{b1.posX() + 1, b1.posY()+1};
    //                         // render();
    //                         stepNum++;
    //                         getObs();
                            
    //                         return(reward);

    //                     }
// float* cacti;
// =============================================================================================================================сбиваются в кучку

                                                                                        // double distance_prev[5] = {1000000,100000,10000,10000,100000};
                                                                                        // double Problem::step()
                                                                                        // {   
                                                                                        //     double reward = 0;
                                                                                        //     // printf("distance = %f",distance);
                                                                                        //     int predator_idx = 0;
                                                                                        //     for(int i = 0; i < 5; i++){
                                                                                        //         if(boids[i].is_predator){
                                                                                        //             predator_idx = i;                
                                                                                        //         }
                                                                                        //     }
                                                                                        //     // printf("\n\n");
                                                                                        //     for(int i = 0; i < 4; i++){
                                                                                        //         if(true){
                                                                                        //             // printf("%f ",caction[i]);
                                                                                        //             boids[i].velocity.x = 6*caction[2*i];
                                                                                        //             boids[i].velocity.y = 6*caction[2*i+1];
                                                                                        //             float distance = sqrt(pow((boids[i].posX() - boids[predator_idx].posX()),2) + pow((boids[i].posY() - boids[predator_idx].posY()),2));
                                                                                        //             if (distance < 900){
                                                                                        //                boids[i].position.x = boids[i].position.x + 8*(caction[2*i] - 0.01);
                                                                                        //                boids[i].position.y = boids[i].position.y + 8*(caction[2*i+1] - 0.01);
                                                                                        //             }           
                                                                                        //             printf("%d %f %f\n",i ,cobservation[2*i], cobservation[2*i+1]);
                                                                                        //             if (boids[i].position.x < 0) boids[i].position.x += boids[i].max_width;
                                                                                        //             if (boids[i].position.y < 0) boids[i].position.y += boids[i].max_height;
                                                                                        //             if (boids[i].position.x >= boids[i].max_width) boids[i].position.x -= boids[i].max_width;
                                                                                        //             if (boids[i].position.y >= boids[i].max_height) boids[i].position.y -= boids[i].max_height; 


                                                                                        //             if (distance_prev[i] > distance){
                                                                                        //                 reward += 1;  
                                                                                        //             }else{
                                                                                        //                 // reward -= 1;
                                                                                        //             }
                                                                                        //             distance_prev[i] = distance;
                                                                                        //         }
                                                                                        //     }
                                                                                        //     // printf("\n\n");
                                                                                        //     // // printf("caction %d\n\n", sizeof(caction));



                                                                                        //     boids[predator_idx].acceleration *= acceleration_scale;
                                                                                        //     boids[predator_idx].acceleration.limit(boids[predator_idx].max_force);
                                                                                        //     boids[predator_idx].velocity += boids[predator_idx].acceleration;
                                                                                        //     if (boids[predator_idx].noise_scale != 0)
                                                                                        //         boids[predator_idx].velocity += (Vector2D::random() - 0.5) * boids[predator_idx].noise_scale;
                                                                                        //     // Limit the velocity so the boids don't get too fast
                                                                                        //     boids[predator_idx].velocity.limit(boids[predator_idx].max_speed);
                                                                                        //     // Then update the position based on the velocity
                                                                                        //     boids[predator_idx].position += boids[predator_idx].velocity;
                                                                                        //     if (boids[predator_idx].position.x < 0) boids[predator_idx].position.x += boids[predator_idx].max_width;
                                                                                        //     if (boids[predator_idx].position.y < 0) boids[predator_idx].position.y += boids[predator_idx].max_height;
                                                                                        //     if (boids[predator_idx].position.x >= boids[predator_idx].max_width) boids[predator_idx].position.x -= boids[predator_idx].max_width;
                                                                                        //     if (boids[predator_idx].position.y >= boids[predator_idx].max_height) boids[predator_idx].position.y -= boids[predator_idx].max_height;


                                                                                        //     // cacti = caction;
                                                                                        //     getObs();
                                                                                        //     return reward;
                                                                                        // }


// double distance_prev[5] = {1000000,100000,10000,10000,100000};
// double Problem::step()
// {   
//     double reward = 0;
//     // printf("distance = %f",distance);
//     int predator_idx = 0;
//     for(int i = 0; i < 5; i++){
//         if(boids[i].is_predator){
//             predator_idx = i;                
//         }
//     }
//     for(int i = 0; i < 5; i ++){
//         if(!boids[i].is_predator){
//             int sign_1 = 0;
//             int sign_2 = 0;
//             if(float(caction[2*i]) >=0){
//                 sign_1 = 1;
//             }
//             else{
//                 sign_1 = -1;
//             }
//             if(float(caction[2*i+1]) >=0){
//                 sign_2 = 1;
//             }
//             else{
//                 sign_2 = -1;
//             }
//         boids[i].velocity.x = sign_1*(caction[2*i])*10;
//         boids[i].velocity.y = sign_2*(caction[2*i+1])*10;
//         boids[i].position.x +=boids[i].velocity.x;
//         boids[i].position.y +=boids[i].velocity.y;
        
//         float distance = sqrt(pow((boids[i].position.x - boids[predator_idx].position.x),2) + pow((boids[i].position.y - boids[predator_idx].position.y),2));
        

//         // if (distance_prev[i] > distance){
//             reward += 1/distance;  
//         // }else{
//             // reward -= 1;
//         // }
//         // distance_prev[i] = distance;
//         if (boids[i].position.x < 0) boids[i].position.x += boids[i].max_width;
//         if (boids[i].position.y < 0) boids[i].position.y += boids[i].max_height;
//         if (boids[i].position.x >= boids[i].max_width) boids[i].position.x -= boids[i].max_width;
//         if (boids[i].position.y >= boids[i].max_height) boids[i].position.y -= boids[i].max_height; 
//         // printf(" %f %f ",caction[2*i], caction[2*i + 1]);
//         }
//     }
//     // printf("\n\n");
//     boids[predator_idx].velocity += (Vector2D::random() - 0.5) * boids[predator_idx].max_speed;
//     // Limit the velocity so the boids don't get too fast
//     boids[predator_idx].velocity.limit(boids[predator_idx].max_speed);
//     // Then update the position based on the velocity
//     boids[predator_idx].position += boids[predator_idx].velocity;
//     if (boids[predator_idx].position.x < 0) boids[predator_idx].position.x += boids[predator_idx].max_width;
//     if (boids[predator_idx].position.y < 0) boids[predator_idx].position.y += boids[predator_idx].max_height;
//     if (boids[predator_idx].position.x >= boids[predator_idx].max_width) boids[predator_idx].position.x -= boids[predator_idx].max_width;
//     if (boids[predator_idx].position.y >= boids[predator_idx].max_height) boids[predator_idx].position.y -= boids[predator_idx].max_height;





//     stepNum++;
//     getObs();
//     return reward;
// }

/*
 * perform the action, update the state of the environment, update observations, return the predator's reward
 */
double distance_prev[5] = {1000000,100000,10000,10000,100000};
double Problem::step()
{   
     int predator_idx = 0;
    for(int i = 0; i < n_bot; i++){
        if(boids[i].is_predator){
            predator_idx = i;                
        }
    }

    double reward = 0;
for(int i = 0; i < n_bot; i ++){
    if(!boids[i].is_predator){
        boids[i].acceleration.x = 5*caction[2*i];
        boids[i].acceleration.y = 5*caction[2*i+1];    
        boids[i].acceleration *= acceleration_scale;
        boids[i].acceleration.limit(max_force);
        boids[i].velocity += boids[i].acceleration;
        if (noise_scale != 0)
            boids[i].velocity += (Vector2D::random() - 0.5) * boids[i].noise_scale;
        // Limit the velocity so the boids don't get too fast
        boids[i].velocity.limit(boids[i].max_speed);
        // Then update the position based on the velocity
        boids[i].position += boids[i].velocity;
        // Set the acceleration to zero before the next update
        boids[i].acceleration = 0;
        // If boid leaves the screen, update position so the boid wraps around
        if (boids[i].position.x < 0) boids[i].position.x += boids[i].max_width;
        if (boids[i].position.y < 0) boids[i].position.y += boids[i].max_height;
        if (boids[i].position.x >= boids[i].max_width) boids[i].position.x -= boids[i].max_width;
        if (boids[i].position.y >= boids[i].max_height) boids[i].position.y -= boids[i].max_height;

        float distance = sqrt(pow((boids[i].position.x - boids[predator_idx].position.x),2) + pow((boids[i].position.y - boids[predator_idx].position.y),2));
        

        // if (distance_prev[i] > distance){
            reward += 1 - 1/distance;  
        // }
            // reward -= 1;
        // }
        distance_prev[i] = distance;



    }
}
    boids[predator_idx].velocity =  3; //boids[predator_idx].max_speed; // (Vector2D::random() - 0.5) *
    // Limit the velocity so the boids don't get too fast
    boids[predator_idx].velocity.limit(boids[predator_idx].max_speed);
    // Then update the position based on the velocity
    boids[predator_idx].position += boids[predator_idx].velocity;
    if (boids[predator_idx].position.x < 0) boids[predator_idx].position.x += boids[predator_idx].max_width;
    if (boids[predator_idx].position.y < 0) boids[predator_idx].position.y += boids[predator_idx].max_height;
    if (boids[predator_idx].position.x >= boids[predator_idx].max_width) boids[predator_idx].position.x -= boids[predator_idx].max_width;
    if (boids[predator_idx].position.y >= boids[predator_idx].max_height) boids[predator_idx].position.y -= boids[predator_idx].max_height;


    // cacti = caction;
    getObs();
    return reward;
    }

Vector2D Problem::alignment(Boid b) 
{
    // const std::vector<Boid *> &boids;
    Vector2D perceived_velocity;
    int n = 0;

    for (int i=0; i<5; i++) {
        if (b.is_predator) {
            return Vector2D{};

            perceived_velocity += 1; //b.velocity;
            ++n;
        }
    }

    if (n == 0)
        return Vector2D{};

    perceived_velocity /= n;
    Vector2D steer = perceived_velocity - b.velocity;
    return steer.normalize();
}

int Problem::isDone()
{
return *cdone;
}

void Problem::close()
{
    //printf("close() not implemented\n");
}

void Problem::render()
{
    int i;
    int c;

    double scale = 1.0;
    
    c = 0;
    // robots
    for (i=0; i<n_bot; i++)
    {   
        dobjects[c] = 3;
        dobjects[c+1] = boids[i].position.x * scale;
        dobjects[c+2] = boids[i].position.y * scale;
        dobjects[c+3] = 1.0* scale;
        dobjects[c+4] = 0.1;
        if (!boids[i].is_predator)
        {
            // boid green
            // printf("Boid %d is here.", i);
            dobjects[c+5] = 0.0;
            dobjects[c+6] = 1.0;
            dobjects[c+7] = 0.0;
        }
        else
        {
            // predator red
            // printf("Predator are here.");
            dobjects[c+5] = 1.0;
            dobjects[c+6] = 0.0;
            dobjects[c+7] = 0.0;            
        }
        dobjects[c+8] = (boids[i].position.x + boids[i].velX()) * scale;//(ro->x + xvect(ro->dir, ro->radius)) * scale;
        dobjects[c+9] = (boids[i].position.y + boids[i].velY()) * scale;//(ro->y + yvect(ro->dir, ro->radius)) * scale;
        c += 10;
    }
    // printf("%d",c);
    dobjects[c] = 0;     
}

float Problem::get_random_float() {
    static std::random_device rd;
    static std::mt19937 engine(rd());
    static std::uniform_real_distribution<float> dist(0, 1);
    return dist(engine);
}

void Problem::add_boid(float x, float y,float ax, float ay, bool is_predator) {
    Boid b = Boid{x, y, ax, ay, (float) window_width, (float) window_height, max_speed, max_force, acceleration_scale,
                  cohesion_weight, alignment_weight, separation_weight, perception, separation_distance, noise_scale,
                  is_predator};

    boids.emplace_back(b);
}

void Problem::getObs()
{
    for(int i = 0; i < n_bot; i++){
        cobservation[2*i] = boids[i].angle_custome();
        cobservation[2*i+1] = sqrt(pow(boids[i].position.x - boids[n_bot-1].position.x,2) + pow(boids[i].position.y - boids[n_bot-1].position.y,2))/1746;
        // cobservation[2*i+1] = boids[i].posY() + (Vector2D::random() - 0.5).y;
        // cobservation[2*i+1] = (boids[i].posY() - 450)/900;
        // cobservation[2*i+1] = (boids[i].posY() - 450)/900;
        // printf("%f %f ",cobservation[2*i],cobservation[2*i+1]);
        // cobservation[2*i] = (boids[i].posX()-750)/1500 + ;
        // cobservation[2*i+1] = (boids[i].posY() - 450)/900 + ;

        // printf("%d angle = %f\n",i,cobservation[i]);
        // printf("%d %f %f %f %f \n",i,boids[i].posX(), boids[i].posY(),boids[i].velX(), boids[i].velY());
    }
    // printf("\n\n");
    // printf("\n");

    // struct robot *ro;
    // int r;
    // int u;
    // int s;
    
    // u = 0;
    // for (r=0, ro=rob; r < nrobots; r++, ro++)
    // {
    //     ro->csensors = ro->sensors;
    //     updateInfraredSensor(ro);
    //     updateCameraPPSensor(ro);
    //     updateGroundGradSensor(ro);
    //     updateTimeSensor(ro, cstep, steps);
    //     updateBiasSensor(ro);
    //     for(s=0, ro->csensors = ro->sensors; s < ninputs; s++, u++, ro->csensors++)
    //         cobservation[u] = *ro->csensors;
    // }
    
}