// MK2-EEZY Robot Arm Configuration File
//  last modified 5/20/2020 by SrAmo

// conversions
mm_inch = 1/25.4;

// length of A-B arm, color = plum
lenAB=9;
lenDE=lenAB;
// A-B arm width. The section is square.
widthAB=1;   
// length of B-C arm, color = blue
lenBC=9;
// B-C arm width. The section is square.
widthBC=1;
// length of A-D arm
lenAD = 3.0;
lenBE = lenAD;
// angle of EBC
angEBC = 180;
// Arm wall thickness throughout
wall_t=0.12;  
// Slot inside (calculated)
wBC_inside = widthBC - 2*wall_t;
//echo (wBC_inside=wBC_inside);

// spring attach pt on ground up from A
spr_dist_ground = 2.0; 
// spring attach pt on AB arm from A
spr_dist_AB = 6.8; 
spr_pt_gnd = [0,spr_dist_ground,0];   // spring attach point on ground

// bearing OD if using a bearing
bearing_od=0.628; // inch
// bearing flange OD (with clearance)
bearing_flange_od=0.71; // inch
// bearing flange thickness
bearing_flange_t=0.05; // inch

// hole for 0.25 inch joint/bolt
hole_p25_inch=0.255;   // inch
// hole start diameter for number 6 screw
hole_no6_screw = 0.09; // inch
// hole for M3 (3 mm) joint/bolt
hole_M3_inch=3.1*mm_inch;  // inch
// hole for servo bushing
hole_servo_bushing=0.15;   // inch

// translation distance for face of A servo
A_servo_z = -.95; 
// translation distance for face of B servo
B_servo_z = 2.0; 
flange_z = -.45;



