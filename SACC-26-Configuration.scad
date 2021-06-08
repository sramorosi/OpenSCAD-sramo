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
lenAB=9;     // 9 in = 228.6 mm
// length of B-C arm, color = blue
lenBC=9;     // 9 in = 228.6 mm
// Arm wall thickness throughout
wall_t=bearing_t;  
// Joint/Pin size throughout
pinSize = hole_p25_inch;

// Number of teeth in BC ARM pulleys
bc_pulley_teeth = 80;
// BC ARM pulley diameter inch
bc_pulley_d = 2.0; // 2 inch = 50.8 mm
// BC ARM pulley thichness mm
bc_pulley_t = 10; // mm, note subtract 2 mm for flanges
bc_boss_t = 0.28; // 0.28 inch = 7.112 mm
a_svo_boss = 0.28; // 0.28 inch = 7.112 mm

// Number of teeth in End (wrist) pulleys
end_pulley_teeth = 54;
// B pulley diameter inch
end_pulley_d = 1.353; // 1.353 inch = 34.3662 mm
// B pulley thichness mm
end_pulley_t = 9; // mm, subtract 2 for flanges (reduced from 10)

// spring attach pt on ground up from A
spr_dist_ground = 2.0; // 2 inch = 50.8 mm
// spring attach pt on AB arm from A
spr_dist_AB = 6.8; // 6.8 inch = 172.72 mm
spr_pt_gnd = [0,spr_dist_ground,0];   // spring attach point on ground

// radius of B spring attachment, opposite from C on BC
B_spr_r = 0.78; // 0.78 inch = 19.812 mm

// BASE RELATED POSITIONING and dimensions
base_z_top = 1.8;  // 1.8 inch = 45.72 mm
// translation distance for face of A servo
A_servo_y = -1.22; // 1.22 in = 30.988 mm
// translation distance for face of B servo
B_servo_y = 2.15; // 2.15 in = 45.61 mm
// base servo lug thickness
base_svo_lug_t = 0.25; // 0.25 inch = 6.35 mm

// center A pulley (THIS DRIVES THE JOINT WIDTH)
a_pulley_t = 12.25; // mm

// extra lug y location
extra_lug_y = (a_pulley_t*mm_inch)/2 + base_svo_lug_t + wall_t + 2*bearing_flange_t+.01;

// A-B arm width. The section is square.
widthAB=a_pulley_t*mm_inch+2*wall_t+2*base_svo_lug_t+4*bearing_flange_t+0.0;   
// B-C arm width. The section is square.
widthBC=widthAB;   
// AB Slot inside (calculated)
wAB_inside = widthAB - 2*wall_t;
// BC Slot inside (calculated)
wBC_inside = widthBC - 2*wall_t;
echo (widthAB=widthAB,widthBC=widthBC,wAB_inside=wAB_inside,wBC_inside=wBC_inside);


// End effector offsets from C to grip/load point
LengthEnd=[125,0,0.0];   // mm

// TO BE FIXED (ALSO IN ROBOT ARM PARTS)

// Horizontal distance from C joint to back plate
End_x=2; // 2 inch = 50.8 mm
// Horizontal distance from C to pin
End_pin_x = End_x - hole_p25_inch*1.5;
// Vertical distance from C joint to pin
End_pin_y=0;
End_angle = atan2(End_pin_y,End_pin_x);
// End effector interface width. 
End_w = end_pulley_t*mm_inch;  // inch
// y location of servo relative to claw
claw_servo_y=1.9; // 1.9 inch = 48.26 mm
claw_y_parts = claw_servo_y+End_pin_x-.6;


