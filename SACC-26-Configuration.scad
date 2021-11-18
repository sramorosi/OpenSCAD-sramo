// Robot Arm Configuration File
// ALL UNITS ARE IN MM
//  last modified October 29 2021 by SrAmo

include <Part-Constants.scad>

// global origin point
origin = [0,0,0]; 

/*
Arm lengths are the center to center distance.
An 8" ctc is about 9.5" total length. This fits in x bed of Prusa i3.
An 9" ctc is about 10.5" total length. This fits rotated 40 deg on bed of Prusa i3.
*/
lenAB=240; // length of A-B arm, color = plum, mm
lenBC=240; // length of B-C arm, color = blue, mm
wall_t=4;  // Arm wall thickness throughout
pinSize = hole_M6; // Joint/Pin size throughout

AB_pulley_teeth = 80; // Number of teeth in AB ARM pulleys
AB_pulley_d = 50.8; // BC ARM pulley diameter, 2 inch = 50.8 mm
AB_pulley_t = 10; // BC ARM pulley thichness mm, note subtract 2 mm for flanges
AB_boss_t = 20; // used for the B servo horn on the pulley

a_svo_boss = 8; // used for the A servo horn

// shoulder RELATED POSITIONING and dimensions
shoulder_z_top = 40;  // offset down from zero, mm
shoulder_svo_lug_t = 7; // shoulder servo lug thickness mm
center_t = 13; // center thickness (WIDTH OF TORSION SPRING, mm
// Y translation distances for lugs
shoulder_y_A = 0; // A servo position
shoulder_y_1 = 18;
shoulder_y_2 = 72;
//shoulder_y_3 = not used
shoulder_y_B = 95; // B servo length, was 93
// distance of shoulder plate from zero
shoulder_y_shift = 45;  // was 38
// shoulder plate size
shoulder_w = 75;
shoulder_l = shoulder_y_B-shoulder_svo_lug_t;
shoulder_t = 7;

// Base plate size
base_x = 130;  // base width
base_y = 150;  // base length
base_z_top = -62; // offset down from zero, mm
base_t = 10;

// parameters for the 4 shoulder attach bolts to the bearing
shoulder_bearing_hole_space = 59;
x_bs = shoulder_bearing_hole_space/2;
y_bs = shoulder_bearing_hole_space/2;
// parameters for the 4 base attach bolts to the bearing
base_bearing_hole_space = 77;
x_bb = base_bearing_hole_space/2;
y_bb = base_bearing_hole_space/2;

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
widthBC=widthAB; // B-C arm width. The section is square.
wAB_inside = widthAB - 2*wall_t; // AB Slot inside (calculated)
wBC_inside = widthBC - 2*wall_t; // BC Slot inside (calculated)

End_x=50.8; // Horizontal distance from C joint to claw back plate, 2 inch = 50.8 mm
End_pin_x = End_x - hole_qtr_inch*1.5; // Horizontal distance from C to pin
End_pin_y=0; // Vertical distance from C joint to pin
End_w = 10; // End effector interface width, mm
// x location of servo relative to pin
claw_servo_x=63 + End_pin_x; // 63 mm relative to claw back plate

// COMPLIANT CLAW parameters
claw_length = 150;
claw_width = 120;
claw_height = 38;  // (initially 1.5 inch) 1.5 inch = 38 mm
claw_radius = 18;
claw_t = 1.73;

// SPRINGS TO HELP JOINT A AND B, AND ARE OPTIONAL

// True if there is a sprint helping joint A
//Spring_at_A = true;
//A_spr_free_len = 30; // A spring free length mm
//A_spr_k = 20; // A spring rate K (force/distance) g/mm
//spr_dist_shoulder = 60; // spring attach pt on shoulder up from A mm
//spr_dist_AB = 181; // spring attach pt on AB arm from A mm
//A_spr_pt_gnd = [-20,spr_dist_shoulder,50];   // spring attach point on shoulder
A_Tspr_rot = 6;  // degrees rotation of torsion spring in installation

// True if there is a sprint helping joint B
//Spring_at_B = true;
//B_spr_free_len = 56; // B spring free length mm
//B_spr_k = 45; // B spring rate K (force/distance) 39 g/mm
//B_spr_dist_shoulder = 130; // B spring attach pt on shoulder up from A mm
//B_spr_r = 50; // radius of B spring attachment, opposite from C on BC 
//B_spr_pt_gnd = [0,B_spr_dist_shoulder,0];   // spring attach point on shoulder

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


