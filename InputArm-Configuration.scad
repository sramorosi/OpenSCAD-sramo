// Robot INPUT Arm Configuration File
//  last modified Feb 2022 by SrAmo

include <Part-Constants.scad>

// global origin point
origin = [0,0,0]; // mm

// length of A-B arm, color = plum
lenAB=50;     // mm
// length of B-C arm, color = blue
lenBC=50;      // mm
// Arm wall thickness throughout
wall_t=3;  //mm

widthAB = 15; // mm, 14.5 is min for good print with potentiometer tabs
pot_lug_t = 8; // thickness of the lug over potentiometer shaft
clevis_gap = 0.1; // mm, space between lug and clevis, total
armt = 20;  // thickness of the clevis with potentiometer

pot_shaft_dia = 6.2; // mm, potentiometer shaft dia

calc_forces = false; // NO NEED TO CALCULATE FORCES FOR INPUT ARM

// End offsets from C to grip/load point
LengthEnd=[20,0,0.0];   // mm

wire_dia = 1; //mm
wire_hole_dia = 4; // mm, minimum size for wire holes in parts

// DEFINITION OF WEIGHTS AND SPRINGS
// Maximum payload weight (thing being lifted) (g)
payload=10;  // g     
// weight of end effector with no payload (g)
end_weight=10;  // g

// SPRING TO HELP JOINT A AND TO HELP JOINT B ARE OPTIONAL

// True if there is a sprint helping joint A
Spring_at_A = false;
// True if there is a sprint helping joint B
Spring_at_B = false;

// DEFINITION OF ARM ANGLES
// When looking at the arm in the XY view (Top), Pos X to the right
// When the AB arm is horizontal pointing right from A, is 0 deg (pos CCW)
// When the BC arm is horizontal pointing right from A, is 0 deg (pos CCW)
// angle limits at the B joint. Angle is 0 when BC in line with AB
// ANGLE ranges are defined in the configuration file
max_B_to_A = 45; // max angle of B to AB arm
min_B_to_A = -150; // min angle of B to AB arm

A_range = 160; // max angle range degrees of A servo
B_range = 180; // max angle range degrees of B servo
A_rigging = 75; // middle of A range used for rigging
B_rigging = 85; // middle of B range used for rigging
