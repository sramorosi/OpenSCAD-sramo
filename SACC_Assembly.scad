// Robot Arm Assembly
//  Started on 3/24/2020 by SrAmo
//  last modified December 2021 by SrAmo

use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
use <Pulley-GT2_2.scad>
use <gears_involute.scad>
use <arduino.scad>
include <SACC-26-Configuration.scad>

// Draw the Robot Arm Assembly
display_assy = false;
// Section cut at X = 0?
clip_yz = true;
// Section cut at Z = 0?
clip_xy = false;
// Draw Final shoulder

gear_space_adjustment = 1; // mm

if (display_assy) {
    difference () {
        draw_assy(A_angle=90,B_angle=120,C_angle=0,base_ang=0);
        if (clip_yz) // x = xcp cut 
            translate ([0,-lenAB*2,-lenAB*2]) cube (lenAB*4,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-lenAB*2,-lenAB*2,0]) cube (lenAB*4,center=false);
    }
}   
module torsion_spring_spacer() {
    translate([0,0,9271K619_t/2]) 
        spacer(d=9271K619_ID*0.9,t=9271K619_t,d_pin=hole_M6*1.03);
}
*torsion_spring_spacer();

module AB_offset_link (length=50,w=15,offset=7,d_pin=5) {
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,0], Second joint is at [length,0]
    // link width is w (y dimension), constant along length of part
    // offset is the length of the second. IF NEGATIVE IT REVERSES THE SHAPE
    // Method of making a dog leg link, using law of sines:
    c_ang=135;
    A = (offset > 0) ? offset : -offset;
    a_angle = law_sines_angle (C=length,a=A,c_ang=c_ang);
    b_ang = 180 - a_angle - c_ang;
    L1 = law_sines_length (C=length,c_ang=c_ang,b_ang=b_ang);
    a_ang = (offset > 0) ? a_angle : -a_angle;
    c_angle = (offset > 0) ? c_ang : -c_ang;
    echo("AB",offset=offset,a_ang=a_ang,b_ang=b_ang,L1=L1);
    rotate([0,0,a_ang]) {
        
        // collection of operations for the first leg
        difference () {
            union() {
                simple_link (l=L1,w=w,t=w,d=0,cored=w*.75); 
                *simple_link (l=L1,w=w,t=w,d=0,cored=0); 
                // add the servo horn pad
                rotate([0,0,-90]) translate([0,widthAB*.25,widthAB-a_svo_boss/2])
                    rounded_cube(size=[widthAB,widthAB*1.5,a_svo_boss],r=widthAB/2,center=true);
            }
    *translate([lenAB/2,0,w/2]) cube([lenAB*2,25.5,25.5],center=true);
            // remove the servo horn shape
            translate([0,0,widthAB-a_svo_boss/2-1]) servo_horn ();
            // remove a screw hole for the horn
            translate([servo_horn_l*0.85,0,widthAB-3]) rotate([90,0,0]) cylinder(h=100,d=3,center=true);
            
            // remove screw holes, used to hold two halfs together
            translate([0,0,w]) hole_pair (x = length*.18,y=w*0.8,d=hole_M3,h=w*.7);
            translate([0,0,w]) hole_pair (x = length*.4,y=w*0.8,d=hole_M3,h=w*.7);
            translate([0,0,w]) hole_pair (x = length*.6,y=w*0.8,d=hole_M3,h=w*.7);
            translate([0,0,w]) hole_pair (x = length*.82,y=w*0.8,d=hole_M3,h=w*.7);
        }
        // collection of operations for the second leg
        translate([L1,0,0]) rotate([0,0,c_angle-180]) 
            simple_link (l=A,w=w,t=w,d=0,cored=0); 
    }
}

module final_AB_arm () {
    // CREATE TWO .STL PARTS BY DISABLING * BIG CUBES
    $fn=$preview ? 64 : 128; // minimum angle fragment
    // Adds operations to the link/arm

    color("plum",1) difference () {
        AB_offset_link (length=lenAB,w=widthAB,offset=-widthAB/2.25,d_pin=pinSize,$fn=48);
            
        // remove the A hole and donut
        cylinder(h=4*widthAB,d=M6_bearing_od,center=true);
        
        translate([0,0,widthAB/2])
            filled_donut(t=widthAB*.55,d=widthAB*1.4,r=widthAB*.2);
        
        // remove the spring leg hole
        translate([0,0,22]) rotate([0,180,90-A_Tspr_rot])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        
        // remove the B hole and end donut
        translate([lenAB,0,widthAB/2]) {
            cylinder(h=4*widthAB,d=pinSize,center=true);
            filled_donut(t=widthAB*.75,d=widthAB*1.8,r=widthAB*.1);
        }
        //  BIG CUBES TO MAKE TWO PARTS, FOR PRINTING
        split_offset=11;
        // USE ONE OR THE OTHER
        *translate([0,0,widthAB+split_offset]) cube([lenAB*4,lenAB*2,widthAB],center=true);
        *translate([0,0,split_offset]) cube([lenAB*4,lenAB*2,widthAB],center=true);
   }
    //  DON'T RENDER WHEN MAKING .STL FILES
    rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   
    translate([0,0,wAB_inside]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   
    // torsion spring at A
    translate([0,0,22]) rotate([0,180,90-A_Tspr_rot]){
        torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH);
        torsion_spring_spacer();
    }
    // // END DON'T RENDER
}
*final_AB_arm();

module final_7462_arm () {
    // Arm version for FTC 7462 Robotics that can hold a 1" x 1" tube
    $fn=$preview ? 64 : 128; // minimum angle fragment
    // Adds operations to the link/arm

    color("blue",1) difference () {
        AB_offset_link (length=lenAB,w=widthAB,offset=widthAB/2.25,d_pin=pinSize,$fn=48);
            
        // remove the A hole and donut
        cylinder(h=4*widthAB,d=M6_bearing_od,center=true);
        
        translate([0,0,widthAB/2])
            filled_donut(t=widthAB*.64,d=widthAB*1.4,r=widthAB*.2);
        
        // remove the spring leg hole
        translate([0,0,22]) rotate([0,180,90-A_Tspr_rot])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        
        // remove the B hole and end donut
        *translate([lenAB,0,widthAB/2]) {
            cylinder(h=4*widthAB,d=pinSize,center=true);
            filled_donut(t=widthAB*.75,d=widthAB*1.8,r=widthAB*.1);
        }
        
        translate([lenAB,0,widthAB/2]) cube([lenAB,lenAB*2,widthAB*2],center=true);

        //  BIG CUBES TO MAKE TWO PARTS, FOR PRINTING
        split_offset=11;
        // USE ONE OR THE OTHER
        *translate([0,0,widthAB+split_offset]) cube([lenAB*4,lenAB*2,widthAB],center=true);
        *translate([0,0,split_offset]) cube([lenAB*4,lenAB*2,widthAB],center=true);
   }
    *rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   
    *translate([0,0,wAB_inside]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   
    // torsion spring at A
    *translate([0,0,22]) rotate([0,180,90-A_Tspr_rot]){
        torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH);
        torsion_spring_spacer();
    }
}
*final_7462_arm();

module BC_offset_link (length=50,w=15,offset=7,d_pin=5) {
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,0], Second joint is at [length,0]
    // link width is w (y dimension), constant along length of part
    // offset is the length of the second. IF NEGATIVE IT REVERSES THE SHAPE
    // Method of making a dog leg link, using law of sines:
    c_ang=135;
    A = (offset > 0) ? offset : -offset;
    a_angle = law_sines_angle (C=length,a=A,c_ang=c_ang);
    b_ang = 180 - a_angle - c_ang;
    L1 = law_sines_length (C=length,c_ang=c_ang,b_ang=b_ang);
    a_ang = (offset > 0) ? -a_angle : a_angle;
    c_angle = (offset > 0) ? c_ang : -c_ang;
    echo("BC",offset=offset,a_ang=a_ang,b_ang=b_ang,L1=L1);
       
    // collection of operations for the first leg
    difference () {
        union() {
            // collection of operations for the first leg
            rotate([0,0,180-c_angle]) 
                    simple_link (l=A,w=w,t=w,d=0,cored=0); 
            
            // second leg, the long part
            translate([length,0,0]) {
                translate([0,0,w-4]) cylinder(h=8,d=w/2,center=true); // for the bolt
                
                rotate([0,0,a_ang]) 
                translate([-L1,0,0]) {
                    difference() {
                        simple_link (l=L1,w=w,t=w,d=0,cored=0); 
                        // remove a cube core, for better printing
                        w_scale = w*.8;
                        translate([0,-w_scale/2,(w-w_scale)/2]) 
                            cube([length*2,w_scale,w_scale],center=false);
                        // remove screw holes, used to hold two halfs together
                        hole_pair (x = length*.15,y=w*0.9,d=2.2,h=w*.7);
                        hole_pair (x = length*.35,y=w*0.9,d=2.2,h=w*.7);
                        hole_pair (x = length*.57,y=w*0.9,d=2.2,h=w*.7);
                        hole_pair (x = length*.78,y=w*0.9,d=2.2,h=w*.7);
                    }
                    // add the servo horn pad
                    translate([L1-widthAB/4,0,0]) rotate([0,0,90])
                    rounded_cube(size=[widthAB,widthAB*1.5,a_svo_boss],r=widthAB/2,center=true);
                }
            }
        }

        // remove the servo horn shape
        *translate([0,0,widthAB-a_svo_boss/2]) servo_horn ();
    }
}
*BC_offset_link (length=lenBC,w=widthAB,offset=-widthAB/2.25,d_pin=pinSize,$fn=48);

module final_BC_arm () {
    // CREATE TWO .STL PARTS BY DISABLING * BIG CUBES
    $fn=$preview ? 64 : 128; // minimum angle fragment
    
    hex_h = AB_pulley_t;  // height offset for hex

    color("lightblue",1) difference () {
        union () {
            BC_offset_link (length=lenBC,w=widthAB,offset=-widthAB/2.25,d_pin=pinSize,$fn=48);

            // union HEX for pulley
            translate ([ 0,0,-hex_h+M6_bearing_flange_t])
                hex (size=22.86,l=hex_h);
            
        }
        // c-bore for B bearing
        cylinder(h=3*widthBC,d=M6_bearing_od,center=true);
        
        // remove donut at C
        translate([0,0,widthAB/2])
            filled_donut(t=widthAB*.8,d=widthAB*2,r=widthAB*.05);
        
        // remove the B hole and end donut
        translate([lenBC,0,widthAB/2]) {
            cylinder(h=4*widthAB,d=hole_qtr_inch,center=true);
            filled_donut(t=widthAB*.8,d=widthAB*1.4,r=widthAB*.1);
        }

        // subtract C servo interface
        translate([lenBC,0,svo_flange_d-4])
            rotate([0,0,0])
                servo_body (vis=false);

        //  BIG CUBES TO MAKE TWO PARTS, FOR PRINTING
        split_offset=-16.1;
        // USE ONE OR THE OTHER BIG CUBE WHEN CREATING .STL FILES
        *translate([0,0,widthAB+split_offset]) cube([lenAB*4,lenAB*2,widthAB],center=true);
        *translate([0,0,split_offset+.2]) cube([lenAB*4,lenAB*2,widthAB],center=true);
    } 
 //  DON'T RENDER WHEN MAKING .STL FILES
   translate([0,0,wall_t]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   translate([0,0,-AB_pulley_t+Qtr_bearing_flange_t]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   translate([0,0,wBC_inside+2*wall_t]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
    
    rotate ([180,0,0]) B_Drive_at_B_Pulley ();
// // END DON'T RENDER
}
*final_BC_arm ();

module B_Drive_at_B_Pulley () {
    color ("green") difference () {
       pulley_gt2_2 ( teeth = AB_pulley_teeth , pulley_t_ht = AB_pulley_t);
        // add hex bore
       hex (size=22.94,l=AB_pulley_t+.5);
    }
}

module B_drive_at_A_Pulley () {
    // This pulley is on the outside of the AB arm at A
    // CREATE TWO .STL PARTS BY DISABLING * PART A OR B
    $fn=$preview ? 64 : 128; // minimum angle fragment
    
    pulley_OD = tooth_spacing (AB_pulley_teeth,2,0.254) +2;
    
    difference () {
        union () {
            // PART A
            pulley_gt2_2(teeth=AB_pulley_teeth,pulley_t_ht=AB_pulley_t ,motor_shaft=hole_qtr_inch);
            translate([0,0,13]) 
                cylinder(h=6,d=pulley_OD,center=true); // big boss A side
            
            // PART B
            translate([0,0,-3]) 
                cylinder(h=6,d=pulley_OD,center=true); // big boss B side
            translate([0,0,-AB_boss_t/2]) 
                rotate([0,0,-10]) 
                    rotate_extrude(angle=60,convexity = 10)
                        translate([25, 0, 0]) 
                            square([15,AB_boss_t],center=true);

            }
        // remove the servo horn shape and screw hole for holding screw
        rotate([0,0,20]) translate([0,0,-AB_boss_t-1]) {
            servo_horn();
            rotate([90,0,0]) 
                translate([servo_horn_l*.85,servo_horn_t/2,0]) 
                    cylinder(h=100,d=3,center=true);
        }
        // remove the spring
        translate([0,0,-3]) rotate([0,0,-90+45]) // ADJUSTMENT ANGLE
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd*1.05,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        translate([-30,-50,10]) cube([40,40,20],center=false);
        translate([-50,-11,10]) cube([40,40,20],center=false);
        translate([0,0,11.5]) cube([21,21,3],center=true);
        // remove the A hole!
        cylinder(h=10*AB_boss_t,d=M6_bearing_od,center=true);
        // remove the screw holes that hold the two parts together
        rotate([0,0,70]) translate([16,0,-4]) Cbore_Screw_Hole(d=3,h=21,cb_d=7,cb_h=2);
        rotate([0,0,160]) translate([16,0,-4]) Cbore_Screw_Hole(d=3,h=21,cb_d=7,cb_h=2);
        rotate([0,0,250]) translate([16,0,-4]) Cbore_Screw_Hole(d=3,h=21,cb_d=7,cb_h=2);
        rotate([0,0,330]) translate([16,0,-4]) Cbore_Screw_Hole(d=3,h=21,cb_d=7,cb_h=2);

        // remove the outside bearing flange cylinder
        *translate([0,0,-AB_pulley_t])
            cylinder(h=AB_pulley_t,d=Qtr_bearing_flange_od+.5,center=false);
    }
    //  don't render when creating .stl files
   translate([0,0,-6]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   translate([0,0,AB_pulley_t+6]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
    translate([0,0,-3]) rotate([0,0,-90+45]) {
        color("blue") torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH);
        color("red") torsion_spring_spacer();
    }
    // // end don't render
}
*B_drive_at_A_Pulley ();

module final_C_horn(){
    // Pulley at joint C
    // This is the arm end-effector interface
    // center is 0,0,0 on xy plane, length is +x
    $fn=$preview ? 64 : 128; // minimum number of fragements

    offx = 5;
    t_at_C=wBC_inside-2*Qtr_bearing_flange_t-offx; // thickness at C
    
    difference () {
        union () { 
            translate([0,0,offx/2]) cylinder (t_at_C,d=30,center=true);
            ear();
            mirror ([0,0,1]) ear() ;// make a mirror copy
        }
        // remove bore bearing
        cylinder(2*t_at_C,d=Qtr_bearing_od,center=true);
        translate([0,0,-6]) cylinder(h=10,d=Qtr_bearing_flange_od+4,center=true);
        
        // remove the servo horn
        translate([0,0,-t_at_C/2-.01]) servo_horn();
        
        // remove End attach pin
        translate ([claw_radius*1.5,0,0])
            hole_pair (x = 0,y=claw_height*0.7,d=hole_M3,h=100);
    }
    module ear () {
        End_angle = atan2(End_pin_y,End_pin_x);
        translate ([0,-claw_height/2,End_w/2]) 
            rotate([0,0,End_angle])
            cube([claw_radius*2,claw_height,End_w/2],center=false);
    }
}
*final_C_horn();

module claw_servo_bracket() {
    $fn=$preview ? 64 : 128; // minimum number of fragements
    servo_plate_t = 8;
    shim_t = 7;
    servo_plate_l = 55;
    back_plate_w = 12+claw_width - 4*claw_radius;
    difference() {
        // interface top
        union() {
            translate ([-servo_plate_l/2-svo_w+5,0,-back_plate_w/2+10]) {
                cube([servo_plate_l,servo_plate_t,back_plate_w],center=false);
                translate([0,-shim_t,0]) 
                    cube([servo_plate_l/3.3,shim_t,back_plate_w],center=false);
            }
        }
        // remove the servo interface
        translate([0,servo_plate_t-svo_flange_d,0]) 
            rotate([0,90,-90]) servo_body(vis=false,$fn=32);
        
        // remove the screw holes
        translate ([-servo_plate_l/2-4,0,0]) rotate([90,0,0]) 
            hole_pair (x = 0,y=claw_radius*1.7,d=hole_M3,h=100);
        
        translate([-38,0,0]) rotate([90,0,0]) cylinder(h=100,d=hole_M3,center=true);
    }
}
*claw_servo_bracket();

module final_claw(){
    // DRAW THE COMPLIANT CLAW
    $fn=$preview ? 32 : 64; // minimum angle fragment

    servo_plate_t = 8;
    back_plate_w = claw_width - 4*claw_radius;
    shim_t = 7;
    difference () {
        translate([claw_radius,-claw_height/2,0]) 
            rotate([0,-90,-90])     
                compliant_claw2 (len=claw_length,width=claw_width,t1=claw_t,t2=claw_height,r=claw_radius,pre_angle=15);
        // remove attach pins (screw holes)
        translate ([claw_radius/2,0,0])
            hole_pair (x = 0,y=claw_height*0.7,d=hole_M3,h=100);
        // remove the screw holes
        translate ([2*claw_radius,0,0]) rotate([90,0,0]) 
            hole_pair (x = 0,y=claw_radius*1.7,d=hole_M3,h=100);
        translate([2*claw_radius-6,0,0]) rotate([90,0,0]) cylinder(h=100,d=hole_M3,center=true);
    }
    
    claw_servo_x = 2*claw_radius+32;
    // Remove Servo Bracket for claw print
    color("green") translate([claw_servo_x,claw_height/2+shim_t,0]) claw_servo_bracket();
    
    // Remove Servo for claw print
    color ("red",.7) translate([claw_servo_x,svo_flange_d+shim_t,0]) 
        rotate([0,90,-90]) servo_body();


}
*final_claw();
*compliant_claw2 (len=claw_length,width=claw_width,t1=claw_t,t2=claw_height,r=claw_radius,pre_angle=15);

module shoulder_servo_lug() {
    $fn=$preview ? 64 : 128; // minimum number of fragements
    difference () {
        lug (r=x_guss,w=shoulder_w/1.3,h=shoulder_z_top+12,t=shoulder_svo_lug_t,d=0);    
        translate([0,shoulder_z_top,svo_flange_d])
            rotate([0,0,90])
                servo_body (vis=false);
        translate([x_guss+t_guss/2,h_guss/2,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=20);
        translate([-(x_guss+t_guss/2),h_guss/2,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=20);
    }
}
*shoulder_servo_lug();

module shoulder_lug() {
    $fn=$preview ? 64 : 128; // minimum number of fragements
    difference() {
        lug (r=x_guss,w=x_guss*2,h=shoulder_z_top,t=shoulder_svo_lug_t,d=hole_M6);
        translate([0,h_guss/2,shoulder_svo_lug_t/2]) 
            rotate([0,90,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=x_guss*3);
   }
}
*shoulder_lug();

module shoulder_assy () {
    // shoulder of the arm
    // XY = HORIZON
    $fn=$preview ? 64 : 128; // minimum number of fragements

    difference () {
        union () {
            // shoulder plate
            translate([0,shoulder_l/2,-shoulder_z_top - shoulder_t/2])
                rounded_cube(size=[shoulder_w,shoulder_l,shoulder_t],r=10,center=true);
            translate([0,shoulder_y_shift,-shoulder_z_top - shoulder_t - 5])
                cylinder(h=10,d=60,center=true);

            // A Servo support (lug)
            color("yellow") translate([0,shoulder_y_A-shoulder_svo_lug_t,-shoulder_z_top])
                rotate([90,0,180]) 
                    shoulder_servo_lug();    
            // First lug
            color("lime") translate([0,shoulder_y_1,-shoulder_z_top])
                rotate([90,0,0]) 
                    shoulder_lug();    
            // Second lug
            color("turquoise") translate([0,shoulder_y_2,-shoulder_z_top])
                rotate([90,0,0]) 
                    shoulder_lug();
            // B Servo support (lug)
            color("blueviolet") translate([0,shoulder_y_B,-shoulder_z_top])
                rotate([90,0,0]) 
                    shoulder_servo_lug();    
               //*/     
            
            // shoulder gussets
            color("blue") translate([x_guss,shoulder_y_A,-shoulder_z_top])
                cube([t_guss,shoulder_l,h_guss],center=false);
            color("blue") translate([-x_guss-t_guss,shoulder_y_A,-shoulder_z_top])
                cube([t_guss,shoulder_l,h_guss],center=false);
            *color("skyblue") translate([-x_guss,y1,-shoulder_z_top])
                cube([2*x_guss,extra_lug_y-y1+shoulder_svo_lug_t,h_guss],center=false);
                
        }
        // subtract the wire hole
        translate([0,shoulder_y_shift,-shoulder_z_top])
                cylinder(h=shoulder_t*5,d=15,center=true);
        
        // subtract the 4 shoulder mounting bolt holes
        translate([x_bs,y_bs+shoulder_y_shift,-shoulder_z_top])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        translate([x_bs,-y_bs+shoulder_y_shift,-shoulder_z_top])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        translate([-x_bs,-y_bs+shoulder_y_shift,-shoulder_z_top])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        translate([-x_bs,y_bs+shoulder_y_shift,-shoulder_z_top])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        
        // subtract the 4 gear mounting screw holes
        translate([0,y_g+shoulder_y_shift,-shoulder_z_top-12])
                cylinder(h=shoulder_t*2,d=hole_no6_screw,center=true);
        translate([x_g,shoulder_y_shift,-shoulder_z_top-12])
                cylinder(h=shoulder_t*2,d=hole_no6_screw,center=true);
        translate([-x_g,shoulder_y_shift,-shoulder_z_top-12])
                cylinder(h=shoulder_t*2,d=hole_no6_screw,center=true);
        translate([0,-y_g+shoulder_y_shift,-shoulder_z_top-12])
                cylinder(h=shoulder_t*2,d=hole_no6_screw,center=true);
        
        // screw holes long ways in gussets
        translate([x_guss+t_guss/2,3,-shoulder_z_top + h_guss/2]) 
            rotate([90,0,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=10);
        translate([x_guss+t_guss/2,shoulder_y_B-10,-shoulder_z_top + h_guss/2]) 
            rotate([90,0,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=10);
        translate([-(x_guss+t_guss/2),3,-shoulder_z_top + h_guss/2]) 
            rotate([90,0,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=10);
        translate([-(x_guss+t_guss/2),shoulder_y_B-10,-shoulder_z_top + h_guss/2]) 
            rotate([90,0,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=10);
            
        // screw holes perpendicular to gussets
        translate([0,shoulder_y_1-shoulder_svo_lug_t/2,-shoulder_z_top + h_guss/2]) 
            rotate([90,0,90]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=x_guss*4);
        translate([0,shoulder_y_2-shoulder_svo_lug_t/2,-shoulder_z_top + h_guss/2]) 
            rotate([90,0,90]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=x_guss*4);
        
        // subtract the B pulley arm torsion spring leg hole
        translate([0,shoulder_y_1+9271K619_t+4,0]) rotate([90,180,0])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        // subtract the AB torsion spring leg hole
        translate([0,shoulder_y_2-8,0]) rotate([90,180-A_Tspr_rot,0])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
    }
}
*shoulder_assy ();

module Electronics_Board () {
    $fn=$preview ? 16 : 64; // minimum number of fragements
    board_l = 180;
    board_shift = 60;
    y_w = (base_x-20)/2;
    x_w = (board_l-20)/2;
    difference() {
        translate([board_shift,0,-4]) rounded_cube([board_l,base_x,4],r=10,center=true);
        scale([1.01,1.01,1]) Power_Energy_Meter();
        translate([50,-30,0]) rotate([0,0,90]) scale([1.01,1.01,1]) Rocker_Switch();
        translate([90,30,-8]) rotate([0,0,90]) Current_Shunt();
        
        // subtract the 4 2X4 screw mounting holes
        translate([x_w+board_shift,y_w,0]) cylinder(h=base_t*3,d=3,center=true);
        translate([x_w+board_shift,-y_w,0]) cylinder(h=base_t*3,d=3,center=true);
        translate([-x_w+board_shift,-y_w,0])cylinder(h=base_t*3,d=3,center=true);
        translate([-x_w+board_shift,y_w,0]) cylinder(h=base_t*3,d=3,center=true);
        
        // arduino board holes (see arduino.scad)
        translate([150,10,10]) rotate([180,0,-90]) holePlacement()
            union() {
                cylinder(d=4, h = 30, $fn=32);
              };
    }
    //   Comment out these items when printing the electronics board
    Power_Energy_Meter();
    translate([50,-30,0]) rotate([0,0,90]) Rocker_Switch();
    translate([90,30,-8]) rotate([0,0,90]) Current_Shunt();
    translate([150,10,-8]) rotate([180,0,-90]) arduino(); //  end comment
}
*Electronics_Board();

module base_assy() {
    $fn=$preview ? 32 : 72; // minimum angle fragment
    // location for the holes that screw the base to the wood
    x_w = (base_x-20)/2;
    y_w = (base_y-20)/2;
    
    difference() {
        union() {
         rounded_cube(size=[base_x,base_y,base_t],r=12,center=true);
            translate([base_x/2-15,0,-svo_flange_d/2]) 
            cube([svo_flange_l*1.2,svo_w*1.4,svo_flange_d+base_t],center=true);
        }
        // big middle hole
        cylinder(h=shoulder_t*2.,d=94,center=true); 
        
        // hole for small gear on servo
        translate([(64+32)/2*(2.54/3.14159)-gear_space_adjustment,0,1]) 
            cylinder(h=shoulder_t*2.1,d=30,center=true);
        
        // the distance between gears is the teeth*pitch/pi
        translate([(64+32)/2*(2.54/3.14159)-gear_space_adjustment,0,-3]) 
            rotate([0,0,180]) 
            servo_body(vis=false);  // shoulder servo
        
        // subtract the 4 bearing mounting screw holes
        translate([x_bb,y_bb,0]) cylinder(h=base_t*3,d=2.5,center=true);
        translate([x_bb,-y_bb,0]) cylinder(h=base_t*3,d=2.5,center=true);
        translate([-x_bb,-y_bb,0])cylinder(h=base_t*3,d=2.5,center=true);
        translate([-x_bb,y_bb,0]) cylinder(h=base_t*3,d=2.5,center=true);
        
        // subtract the 4 2X4 screw mounting holes
        translate([x_w,y_w,0]) cylinder(h=base_t*3,d=4,center=true);
        translate([x_w,-y_w,0]) cylinder(h=base_t*3,d=4,center=true);
        translate([-x_w,-y_w,0])cylinder(h=base_t*3,d=4,center=true);
        translate([-x_w,y_w,0]) cylinder(h=base_t*3,d=4,center=true);
    }
    // Representation of bearing
    translate([0,0,9+base_t/2]) Bearing(t=9,od=120,id=70);
    
    // Representation of 2x4 wood
    l_wood = 400;
    color("Khaki") {
        translate([-base_x/2,base_x/2-15,-base_t/2]) rotate([0,90,0]) cube([3.5*25.4,1.5*25.4,l_wood]);
        translate([-base_x/2,-base_x/2+15-1.5*25.4,-base_t/2]) rotate([0,90,0]) cube([3.5*25.4,1.5*25.4,l_wood]);
    }
    
    // Representation of electronics board
    translate([150,0,0]) Electronics_Board();
}
*base_assy();

module base_and_shoulder_assy(base_ang=0,A_angle=0,B_angle=0){
    // shoulder, adjust z translation as required
    rotate([-90,base_ang,0]) 
        translate([0,-shoulder_y_shift,0]) { 
            color("green",.5) shoulder_assy (); 
            // B Servo (was A)
            color ("red",.5) rotate([-90,-90,0])
                translate([0,0,shoulder_y_A+shoulder_svo_lug_t/2]) {
                    servo_body();
                    rotate([0,0,B_angle-120])
                        servo_horn();
                }
            // A Servo  (was B)
            color ("darkred",.5) 
                translate([0,shoulder_y_B-svo_flange_d,0]) 
                rotate([90,-90,0]) {
                    servo_body();
                    rotate([0,0,A_angle-90])
                        servo_horn();
                }
        }
    // sholder big gear  64 tooth
    translate([0,-1.42*shoulder_z_top,0]) rotate([90,0,0]) 64T_32P_Actobotics();
    // the distance between gears is the teeth*pitch/pi
    color ("red",.5) 
        translate([-(64+32)/2*(2.54/3.14159)+gear_space_adjustment,base_z_top-2,0])
            rotate([-90,0,0]) {
                servo_body();  // shoulder servo
                32T_32P_Actobotics();   // servo gear 32 tooth
    }
        
    // Base
    translate([0,base_z_top,0]) rotate([-90,180,0]) base_assy();
}
*base_and_shoulder_assy(base_ang=0,A_angle=0,B_angle=0);

module end_effector_assy() {
    color("SpringGreen") final_C_horn (); 
    translate([claw_radius,0,0]) final_claw();
}
*end_effector_assy();

module BC_arm_and_End (C_angle=0) {
    
    final_BC_arm ();
    
    // Draw the end effector
    translate([lenBC,0,0]) {
        
        translate([0,0,6])color ("red",.5) servo_body();  // C Servo
        
        rotate ([0,0,-C_angle]) {
            translate([0,0,widthBC/2]) end_effector_assy();
            translate([0,0,6])color ("darkred",.5) servo_horn();
        }
    }
}
*BC_arm_and_End (C_angle=90);

module draw_assy (A_angle=0,B_angle=0,C_angle=0,base_ang=0) {
    // XZ = HORIZON
    // calculate b and c positions from angles
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  // B location
    c = [(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),0];
    // lower arm vector
    //vecAB=[b[0]/lenAB,b[1]/lenAB,b[2]/lenAB];
    rotate([0,base_ang,0]) {
        translate([0,0,5]) {     // There are a few unfortunate fixed rotations 
            // draw the upper and lower arms
            rotate([0,0,A_angle]) {
                // Draw the AB link
                translate ([0,0,-1]) rotate([180,0,0]) final_AB_arm ();
                
                // Draw the BC link and End
                translate([lenAB,0,5])
                    rotate([0,0,B_angle-A_angle]) rotate([180,0,0]) BC_arm_and_End(C_angle);
            }
            
            // Draw the B drive pulley at A
            color("navy",0.5)  
                rotate([0,0,(B_angle-45)]) 
                    translate([0,0,16]) rotate([180,0,0]) B_drive_at_A_Pulley ();
                    
            // B drive belt (displayed with shoulder assembly)
            belt_z = 11;
            color("blue") 
                pt_pt_belt([0,0,belt_z],[b[0],b[1],belt_z],d=6,r_pulley=AB_pulley_d/2,round=false);
        }
    }
    // Draw Base and Shoulder assembly adjust z translation as required
    //rotate([0,0,0]) 
    base_and_shoulder_assy(base_ang=base_ang,A_angle=A_angle,B_angle=B_angle);
    
    // A joint shaft
    translate([0,0,-1]) cylinder(h=66,d=5.5,center=true);
} 
draw_assy (A_angle=120,B_angle=-30,C_angle=-40,base_ang=0);