// Robot INPUT Arm Configuration File
//  last modified APR 26 2021 by SrAmo

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
armt = 20;  // thickness of the clevis with potentiometer

//widthBC = 10; // not used

pinSize = 6.2; // mm, potentiometer shaft

calc_forces = false; // NO NEED TO CALCULATE FORCES FOR INPUT ARM

// End offsets from C to grip/load point
LengthEnd=[20,0,0.0];   // mm

wire_hole_dia = 4; // mm, minimum size for wire holes in parts
wire_hole_offset = sqrt((wire_hole_dia/2)*(wire_hole_dia/2)-.3); // space to insert wire
echo(wire_hole_offset=wire_hole_offset);