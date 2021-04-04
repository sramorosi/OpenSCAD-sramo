// Robot INPUT Arm Configuration File
//  last modified APR 2 2021 by SrAmo

include <Part-Constants.scad>

// global origin point
origin = [0,0,0]; // mm

// length of A-B arm, color = plum
lenAB=50;     // mm
// length of B-C arm, color = blue
lenBC=50;      // mm
// Arm wall thickness throughout
wall_t=3;  //mm

widthAB = 14.5; // mm, 13.5 is min for good print with pot tabs
pot_lug_t = 8; // thickness of the lug over pot shaft
armt = 20;  // thickness of the clevis with pot

//widthBC = 10; // not used

pinSize = 6.2; // mm, pot shaft

calc_forces = false; // NO NEED TO CALCULATE FORCES FOR INPUT ARM

// End offsets from C to grip/load point
LengthEnd=[20,0,0.0];   // mm

