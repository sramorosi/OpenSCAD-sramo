// Robot Arm Configuration File
//  last modified 8/17/2020 by SrAmo

// conversions
mm_inch = 1/25.4;
// global origin point
origin = [0,0,0]; 

// 1/4 ID bearing OD if using a bearing (PART FR4-ZZ C3)
bearing_od=0.628; // inch
// 1/4 ID bearing flange OD (with clearance)
bearing_flange_od=0.71; // inch
// 1/4 ID bearing flange thickness
bearing_flange_t=0.05; // inch
// 1/4 ID bearing thickness (under the flange)
bearing_t=0.155; // inch

/*
Arm lengths are the center to center distance.
An 8" ctc is about 9.5" total length. This fits in x bed of Prusa i3.
An 9" ctc is about 10.5" total length. This fits rotated 40 deg on bed of Prusa i3.
*/
// length of A-B arm, color = plum
lenAB=9;     
// length of B-C arm, color = blue
lenBC=9;      
// Arm wall thickness throughout
wall_t=bearing_t;  

// Number of teeth in BC ARM pulleys
bc_pulley_teeth = 80;
// BC ARM pulley diameter inch
bc_pulley_d = 2.0; // inch
// BC ARM pulley thichness mm
bc_pulley_t = 10; // mm, note subtract 2 mm for flanges
bc_boss_t = 0.28; // inch
a_svo_boss = 0.28; // inch

// Number of teeth in End (wrist) pulleys
end_pulley_teeth = 54;
// B pulley diameter inch
end_pulley_d = 1.353; // inch
// B pulley thichness mm
end_pulley_t = 9; // mm, subtract 2 for flanges (reduced from 10)

// spring attach pt on ground up from A
spr_dist_ground = 2.0; 
// spring attach pt on AB arm from A
spr_dist_AB = 6.8; 
spr_pt_gnd = [0,spr_dist_ground,0];   // spring attach point on ground

// radius of B spring attachment, opposite from C on BC
B_spr_r = 0.78; 

// round belt diameter throughout, if used
belt_d=0.19;   

// hole for 0.25 inch joint/bolt
hole_p25_inch=0.255;   // inch
// hole start diameter for number 6 screw
hole_no6_screw = 0.095; // inch
// hole for M3 (3 mm) joint/bolt
hole_M3_inch=3.1*mm_inch;  // inch
// hole for M5 (5 mm) joint/bolt
hole_M5_inch=5.1*mm_inch;  // inch
// hole for servo bushing
hole_servo_bushing=0.15;   // inch

// STANDARD 40MM X 20MM X 40MM SERVO DIMENSIONS
// Servo Length for openings in parts
svo_l = 40.66 * mm_inch; 
// Servo Width for openings in parts
svo_w = 20.3 * mm_inch;
// Servo Depth from Horn interface to bottom of part
svo_d = 42 * mm_inch;
// Servo dist from shaft to edge of body in length direction
svo_shaft = 10.2 * mm_inch;
// Servo screw hole length between
svo_screw_l = 49.5 * mm_inch;
// Servo screw hole width between
svo_screw_w = 10 * mm_inch;
// Servo flange length
svo_flange_l = 55 * mm_inch;
// Servo flange thickness
svo_flange_t = 3 * mm_inch;
// Servo depth from Horn interface to top of flange
svo_flange_d = 10 * mm_inch;

// BASE RELATED POSITIONING and dimensions
base_z_top = 1.8;  // inch    
// translation distance for face of A servo
A_servo_y = -1.22; 
// translation distance for face of B servo
B_servo_y = 2.15; 
// base servo lug thickness
base_svo_lug_t = 0.25; // inch
// C servo center relative to base
c_svo_x = 2.6;
c_svo_y = 0.35;
c_svo_z = -1.2;
c_servo_ctr = [c_svo_x,c_svo_z,c_svo_y];

// center A pulley (THIS DRIVE THE JOINT WIDTH)
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


// Interface at C Pulley (End Effector)
// Horizontal distance from C joint to back plate
End_x=1.2; 
// Horizontal distance from C to pin
End_pin_x = End_x - hole_p25_inch*1.5;
// Vertical distance from C joint to pin
End_pin_y=1.0;
End_angle = atan2(End_pin_y,End_pin_x);
// End effector interface width. 
End_w = end_pulley_t*mm_inch;  // inch
// End effector offsets from C to grip/load point
LengthEnd=[5,0,0.0];   

// y location of servo relative to claw
claw_servo_y=1.9; 
// big claw radius
claw_corner = 0.7;
claw_y_parts = claw_servo_y+End_pin_x-.6;



