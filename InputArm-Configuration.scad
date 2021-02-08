// Robot INPUT Arm Configuration File
//  last modified FEB 7 2021 by SrAmo

include <Part-Constants.scad>

// global origin point
origin = [0,0,0]; // inch

typeRobotArm = false;

/*
Arm lengths are the center to center (ctc) distance.
An 9" ctc is about 10.5" total length. This fits rotated 40 deg on bed of Prusa i3.
*/
// length of A-B arm, color = plum
lenAB=2;     // inch
// length of B-C arm, color = blue
lenBC=2;      // inch
// Arm wall thickness throughout
wall_t=0.1;  //inch

// spring attach pt on ground up from A
// NEEDED IN SIMULATION, BUT INPUT DOESN'T HAVE SPRING
spr_dist_ground = 1.0; 

widthAB = 0.5;
widthBC = 0.6;
