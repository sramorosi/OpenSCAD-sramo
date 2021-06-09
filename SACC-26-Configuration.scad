// Robot Arm Configuration File
// ALL UNITS ARE IN MM
//  last modified JUNE 8 2021 by SrAmo

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

// spring attach pt on ground up from A
spr_dist_ground = 50.8; // 2 inch = 50.8 mm
// spring attach pt on AB arm from A
spr_dist_AB = 172.72; // 6.8 inch = 172.72 mm
spr_pt_gnd = [0,spr_dist_ground,0];   // spring attach point on ground

// radius of B spring attachment, opposite from C on BC
B_spr_r = 19.812; // 0.78 inch = 19.812 mm

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

// extra lug y location
extra_lug_y = (a_pulley_t*mm_inch)/2 + base_svo_lug_t + wall_t + 2*bearing_flange_t+.01;

// A-B arm width. The section is square.
widthAB=a_pulley_t+2*wall_t+2*base_svo_lug_t+4*bearing_flange_t;   
// B-C arm width. The section is square.
widthBC=widthAB;   
// AB Slot inside (calculated)
wAB_inside = widthAB - 2*wall_t;
// BC Slot inside (calculated)
wBC_inside = widthBC - 2*wall_t;
echo (widthAB=widthAB,widthBC=widthBC,wAB_inside=wAB_inside,wBC_inside=wBC_inside);


// End effector offsets from C to grip/load point
LengthEnd=[125,0,0.0];   // mm

// Horizontal distance from C joint to claw back plate
End_x=50.8; // 2 inch = 50.8 mm
// Horizontal distance from C to pin
End_pin_x = End_x - hole_p25_inch*1.5;
// Vertical distance from C joint to pin
End_pin_y=0;
// End effector interface width. 
End_w = 10;  // mm
// x location of servo relative to pin
claw_servo_x=48.26 + End_pin_x; // 1.9 inch = 48.26 mm relative to claw back plate

// COMPLIANT CLAW parameters
claw_length = 150;
claw_width = 120;
claw_height = 38;
claw_radius = 18;

