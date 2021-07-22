// Robot Arm Configuration File
// ALL UNITS ARE IN MM
//  last modified JUNE 16 2021 by SrAmo

include <Part-Constants.scad>

// global origin point
origin = [0,0,0]; 

/*
Arm lengths are the center to center distance.
An 8" ctc is about 9.5" total length. This fits in x bed of Prusa i3.
An 9" ctc is about 10.5" total length. This fits rotated 40 deg on bed of Prusa i3.
*/
// length of A-B arm, color = plum
lenAB=228.6;     // 9 in = 228.6 mm
// length of B-C arm, color = blue
lenBC=228.6;     // 9 in = 228.6 mm
// Arm wall thickness throughout
wall_t=bearing_t;  
// Joint/Pin size throughout
pinSize = hole_p25_inch;

// Number of teeth in AB ARM pulleys
AB_pulley_teeth = 80;
// BC ARM pulley diameter inch
AB_pulley_d = 50.8; // 2 inch = 50.8 mm
// BC ARM pulley thichness mm
AB_pulley_t = 10; // mm, note subtract 2 mm for flanges
AB_boss_t = 7.112; // 0.28 inch = 7.112 mm
a_svo_boss = 7.112; // 0.28 inch = 7.112 mm

// BASE RELATED POSITIONING and dimensions
base_z_top = 45.72;  // 1.8 inch = 45.72 mm
// translation distance for face of A servo
A_servo_y = -31; // 1.22 in = 30.988 mm
// translation distance for face of B servo
B_servo_y = 45.61; // 2.15 in = 45.61 mm
// base servo lug thickness
base_svo_lug_t = 6.35; // 0.25 inch = 6.35 mm

// center A pulley (THIS DRIVES THE JOINT WIDTH)
a_pulley_t = 12.25; // mm

// extra lug y location, USED IN THE BASE
extra_lug_y = (a_pulley_t*mm_inch)/2 + base_svo_lug_t + wall_t + 2*bearing_flange_t+.01;

// A-B arm width. The section is square.
widthAB=a_pulley_t+2*wall_t+2*base_svo_lug_t+4*bearing_flange_t;   
// B-C arm width. The section is square.
widthBC=widthAB;   
// AB Slot inside (calculated)
wAB_inside = widthAB - 2*wall_t;
// BC Slot inside (calculated)
wBC_inside = widthBC - 2*wall_t;
//echo (wall_t=wall_t,widthAB=widthAB,widthBC=widthBC,wAB_inside=wAB_inside,wBC_inside=wBC_inside);

// Horizontal distance from C joint to claw back plate
End_x=50.8; // 2 inch = 50.8 mm
// Horizontal distance from C to pin
End_pin_x = End_x - hole_p25_inch*1.5;
// Vertical distance from C joint to pin
End_pin_y=0;
// End effector interface width. 
End_w = 10;  // mm
// x location of servo relative to pin
claw_servo_x=63 + End_pin_x; // 63 mm relative to claw back plate

// COMPLIANT CLAW parameters
claw_length = 150;
claw_width = 120;
claw_height = 38;
claw_radius = 18;


// SPRING TO HELP JOINT A AND B ARE OPTIONAL

// True if there is a sprint helping joint A
Spring_at_A = true;
// A spring free length (NO SPRING IN SOME CONFIGURATIONS)
A_spr_free_len = 30; // 50 mm
// A spring rate K (force/distance)
A_spr_k = 20; // 17 g/mm
// spring attach pt on base up from A
spr_dist_base = 60; // 2 inch = 50.8 mm
// spring attach pt on AB arm from A
spr_dist_AB = 172.7; // 6.8 inch = 172.72 mm
A_spr_pt_gnd = [0,spr_dist_base,50];   // spring attach point on base

// True if there is a sprint helping joint B
Spring_at_B = true;
// B spring free length
B_spr_free_len = 56; // mm
// B spring rate K (force/distance)
B_spr_k = 45; // 39 g/mm
// B spring attach pt on base up from A
B_spr_dist_base = 130; // mm
// radius of B spring attachment, opposite from C on BC
B_spr_r = 50; // 0.78 inch = 19.812 mm
B_spr_pt_gnd = [0,B_spr_dist_base,0];   // spring attach point on base


// DEFINITION OF ARM ANGLES
// When looking at the arm in the XY view (Top), Pos X to the right
// When the AB arm is horizontal pointing right from A, is 0 deg (pos CCW)
// When the BC arm is horizontal pointing right from A, is 0 deg (pos CCW)
// angle limits at the B joint. Angle is 0 when BC in line with AB
// ANGLE ranges are defined in the configuration file
max_B_to_A = 45; // max angle of B to AB arm
min_B_to_A = -150; // min angle of B to AB arm

A_range = 160; // max angle range degrees of A servo
B_range = 170; // max angle range degrees of B servo
A_rigging = 70; // middle of A range used for rigging
B_rigging = 45; // middle of B range used for rigging


