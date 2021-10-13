// Robot Arm Configuration File
// ALL UNITS ARE IN MM
//  last modified October 13 2021 by SrAmo

include <Part-Constants.scad>

// global origin point
origin = [0,0,0]; 

/*
Arm lengths are the center to center distance.
An 8" ctc is about 9.5" total length. This fits in x bed of Prusa i3.
An 9" ctc is about 10.5" total length. This fits rotated 40 deg on bed of Prusa i3.
*/
// length of A-B arm, color = plum
lenAB=240;     // 9 in = 228.6 mm
// length of B-C arm, color = blue
lenBC=240;     // 9 in = 228.6 mm
// Arm wall thickness throughout
wall_t=4;  
// Joint/Pin size throughout
pinSize = hole_qtr_inch;

// Number of teeth in AB ARM pulleys
AB_pulley_teeth = 80;
// BC ARM pulley diameter inch
AB_pulley_d = 50.8; // 2 inch = 50.8 mm
// BC ARM pulley thichness mm
AB_pulley_t = 10; // mm, note subtract 2 mm for flanges
AB_boss_t = 20; // used for the B servo horn on the pulley

a_svo_boss = 8; // used for the A servo horn

// shoulder RELATED POSITIONING and dimensions
shoulder_z_top = 40;  // mm
// shoulder servo lug thickness
shoulder_svo_lug_t = 7; // 0.25 inch = 6.35 mm
// center thickness (WIDTH OF TORSION SPRING
center_t = 13; // mm
// Y translation distance for lugs
shoulder_y_A = 0; 
shoulder_y_1 = 20;
shoulder_y_2 = 73;
//shoulder_y_3 = shoulder_y_2+center_t+4*shoulder_svo_lug_t+Qtr_bearing_flange_t;
shoulder_y_B = 90; 
echo(shoulder_y_1=shoulder_y_1,shoulder_y_2=shoulder_y_2);
// distance of shoulder plate from zero
shoulder_y_shift = 35;
// shoulder plate size
shoulder_w = 75;
shoulder_l = shoulder_y_B+shoulder_svo_lug_t;
shoulder_t = 7;
// parameters for the 4 shoulder attach bolts to the bearing
hole_space = 58;
x_b = hole_space/2;
y_b = hole_space/2;
// parameters for the 4 gear attach screws
hole_space_gear = 38;
x_g = hole_space_gear/2;
y_g = hole_space_gear/2;
// paramenters for the shoulder gussetts
t_guss = 7.5;
h_guss = 18; // height
x_guss = 15; // offset from center

// A-B arm width. The section is square.
widthAB=center_t+2*wall_t+2*shoulder_svo_lug_t+4*Qtr_bearing_flange_t;   
// B-C arm width. The section is square.
widthBC=widthAB;   
// AB Slot inside (calculated)
wAB_inside = widthAB - 2*wall_t;
// BC Slot inside (calculated)
wBC_inside = widthBC - 2*wall_t;
//echo (wall_t=wall_t,widthAB=widthAB,widthBC=widthBC,wAB_inside=wAB_inside,wBC_inside=wBC_inside);

base_z_top = -60;
base_t = 12;


// Horizontal distance from C joint to claw back plate
End_x=50.8; // 2 inch = 50.8 mm
// Horizontal distance from C to pin
End_pin_x = End_x - hole_qtr_inch*1.5;
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


// SPRINGS TO HELP JOINT A AND B, AND ARE OPTIONAL

// True if there is a sprint helping joint A
//Spring_at_A = true;
// A spring free length (NO SPRING IN SOME CONFIGURATIONS)
A_spr_free_len = 30; // 50 mm
// A spring rate K (force/distance)
A_spr_k = 20; // 17 g/mm
// spring attach pt on shoulder up from A
spr_dist_shoulder = 60; // 2 inch = 50.8 mm
// spring attach pt on AB arm from A
spr_dist_AB = 181; // 6.8 inch = 172.72 mm
A_spr_pt_gnd = [-20,spr_dist_shoulder,50];   // spring attach point on shoulder

// True if there is a sprint helping joint B
//Spring_at_B = true;
// B spring free length
B_spr_free_len = 56; // mm
// B spring rate K (force/distance)
B_spr_k = 45; // 39 g/mm
// B spring attach pt on shoulder up from A
B_spr_dist_shoulder = 130; // mm
// radius of B spring attachment, opposite from C on BC
B_spr_r = 50; // 0.78 inch = 19.812 mm
B_spr_pt_gnd = [0,B_spr_dist_shoulder,0];   // spring attach point on shoulder


// DEFINITION OF ARM ANGLES
// When looking at the arm in the XY view (Top), Pos X to the right
// When the AB arm is horizontal pointing right from A, is 0 deg (pos CCW)
// When the BC arm is horizontal pointing right from A, is 0 deg (pos CCW)
// angle limits at the B joint. Angle is 0 when BC in line with AB
// ANGLE ranges are defined in the configuration file

A_range = 160; // max angle range degrees of A servo
B_range = 170; // max angle range degrees of B servo
A_rigging = 70; // middle of A range used for rigging
B_rigging = 45; // middle of B range used for rigging


