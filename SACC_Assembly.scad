// Robot Arm Assembly
//  Started on 3/24/2020 by SrAmo
//  last modified September 29 2021 by SrAmo

use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
use <Pulley-GT2_2.scad>
include <SACC-26-Configuration.scad>

// Draw the Robot Arm Assembly
display_assy = true;
// Section cut at X = 0?
clip_yz = false;
// Section cut at Z = 0?
clip_xy = false;
// Draw Final Base
display_base = false;
// Draw Final AB arm
display_ABarm = false;
// Draw Final BC arm
display_BCarm = false;
// Draw Final Claw 
display_claw= false;
// Draw Final Claw Attachment
display_claw_shooter= false;
// Is the shooter curved or linear?
curved_shooter=false;
// Draw Final Forks (option to claw)
display_fork= false;
// BC Arm Pulley at A
display_B_drive_pulley_at_A= false;
// BC Arm Pulley at B
display_B_drive_pulley_at_B= false;
// Draw Final End (C) Horn
display_C_horn= false;
// Draw B Spring Pulley
display_B_spring_pulley= false;

if (display_assy) {
    difference () {
        draw_assy(A_angle=90,B_angle=0,C_angle=0);
        if (clip_yz) // x = xcp cut 
            translate ([0,-lenAB*2,-lenAB*2]) cube (lenAB*4,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-lenAB*2,-lenAB*2,0]) cube (lenAB*4,center=false);
        }
    }

if (display_ABarm) final_AB_arm ();
if (display_BCarm) final_BC_arm ();
if (display_base) {
    difference () {
        robot_arm_base ();
        if (clip_yz) // x = 0 cut 
            translate ([-20,-10,-10]) cube (20,center=false);
    }
}
if (display_B_drive_pulley_at_A) B_drive_at_A_Pulley ();
if (display_B_drive_pulley_at_B) B_Drive_at_B_Pulley ();
if (display_C_horn) final_C_horn ();
if (display_claw_shooter)  {
    color ("purple") claw_shooter(curved=curved_shooter);
}
if (display_claw)  final_claw();
if (display_fork)  final_fork();
if (display_B_spring_pulley) final_B_Spring_pulley();
    
module final_B_Spring_pulley() {
    $fn=$preview ? 60 : 100; // number of fragements
    B_Spr_width = wBC_inside - 2*wall_t-1;
    B_Spr_OD = widthBC*1.6;
    //echo(B_Spr_width=B_Spr_width,B_Spr_OD=B_Spr_OD);
    color ("skyblue") 
        translate ([0,0,B_Spr_width/2])
        difference () {
            // Starting cylinder
            cylinder(h=B_Spr_width,d=B_Spr_OD+6,center=true);
            // Subtract the outer donut
            difference () {
                // this leaves the flanges, 1 mm each
                cylinder(h=B_Spr_width-2,d=B_Spr_OD+8,center=true);
                cylinder(h=B_Spr_width,d=B_Spr_OD,center=true);
            }
            // subtract the hole through the middle
            cylinder(h=B_Spr_width+1,d=Qtr_bearing_od,center=true);
            // subtract a rectangle on one side for clevis clearance
            translate([-widthBC,widthBC/4,-wBC_inside]) 
            cube([2*widthBC,widthBC,2*wBC_inside],center=false);
            // subtract a rectangle on one side for clevis clearance
            translate([widthBC/4,-widthBC,-wBC_inside]) 
            cube([widthBC,2*widthBC,2*wBC_inside],center=false);
            // subtract a cylinder at the middle for wires
            cylinder(h=B_Spr_width/2,d=B_Spr_OD/1.3,center=true);
    }
}
   

module end_effector_assy() {
    color("SpringGreen") final_C_horn (); 
    color("green") final_claw();
    //color ("purple") claw_shooter(curved=curved_shooter);
    color ("red",.5) translate([claw_servo_x,claw_height/2-svo_flange_d,0]) 
        rotate([0,90,-90]) servo_body();
}
module zip_tie_AB (x = 50,y=5) {
    // holes are parallel to the Y axis
    zip_hole_d = hole_M3;
    rotate([90,0,0]) {
        translate ([x,-y,0]) 
            cylinder(h=100,d=zip_hole_d,center=true,$fn=16);
        translate ([x,y,0]) 
            cylinder(h=100,d=zip_hole_d,center=true,$fn=16);
    }
}

module AB_offset_link (length=50,w=15,offset=7,d_pin=5,t=2) {
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,0], Second joint is at [length,0]
    // link width is w (y dimension), constant along length of part
    // Method of making a dog leg link, using law of sines:
    c_ang=135;
    a_ang = law_sines_angle (C=length,a=offset,c_ang=c_ang);
    b_ang = 180 - a_ang-c_ang;
    L1 = law_sines_length (C=length,c_ang=c_ang,b_ang=b_ang);
    echo(a_ang=a_ang,b_ang=b_ang,L1=L1);
    rotate([0,0,a_ang]) {
        
        // collection of operations for the first leg
        simple_link (l=L1,w=w,t=w,d=0,cored=w*.75); 
        // add the servo horn pad
        difference () {
            rotate([0,0,-90]) translate([0,widthAB*.25,widthAB+a_svo_boss/2])
                rounded_cube(size=[widthAB,widthAB*1.5,a_svo_boss],r=widthAB/2,center=true);
            // remove the servo horn shape
            translate([0,0,widthAB-.01]) servo_horn ();
        }
        translate([L1,0,0]) rotate([0,0,c_ang-180]) 
            // collection of operations for the second leg
            simple_link (l=offset,w=w,t=w,d=0,cored=0); 
    }
}

module final_AB_arm () {
    $fa=$preview ? 6 : 1; // minimum angle fragment

    difference () {
        AB_offset_link (length=lenAB,w=widthAB,offset=widthAB/2.5,d_pin=pinSize,t=wall_t,$fn=48);
            
        // remove the A hole and donut
        cylinder(h=4*widthAB,d=Qtr_bearing_od,center=true);
        
        translate([0,0,widthAB/2]) {
            zip_tie_AB (x=46,y=4);
            filled_donut(t=widthAB*.75,d=widthAB*1.4,r=widthAB*.2);
        }
        
        // remove the B hole and end donut
        translate([lenAB,0,widthAB/2]) {
            cylinder(h=4*widthAB,d=pinSize,center=true);
            filled_donut(t=widthAB*.75,d=widthAB*1.4,r=widthAB*.2);
        }
   }
}
module final_BC_arm () {
    $fa=$preview ? 6 : 1; // minimum angle fragment
    
    hex_h = AB_pulley_t;  // height offset for hex

    difference () {
        union () {
            hollow_offset_link(length=lenBC,d_pin=pinSize,w=widthBC,t=widthBC,offset=widthBC/2.5,ang=45,wall=wall_t,$fn=48); 
            // union HEX for pulley
            translate ([ 0,0,-hex_h+Qtr_bearing_flange_t])
                hex (size=22.86,l=hex_h);
            
            // boss for servo
            translate([lenBC,0,-1])
                rotate([0,0,-45])
                    translate([-10,0,0])
                    rounded_cube(size=[svo_screw_l+5,svo_w+5,5],r=7,center=true);
        }
        // c-bore for bearing
        cylinder(h=3*widthBC,d=Qtr_bearing_od,center=true);
            
        // subtract A servo interface
        translate([lenBC,0,svo_flange_d-3.5])
            rotate([0,0,-45])
                servo_body (vis=false);

    }    
   translate([0,0,wall_t]) Bearing_Flanged (t=Qtr_bearing_t,flange_t=Qtr_bearing_flange_t,od=Qtr_bearing_od,id=hole_qtr_inch,flange_od=Qtr_bearing_flange_od);
   translate([0,0,-AB_pulley_t+Qtr_bearing_flange_t]) rotate([180,0,0]) Bearing_Flanged (t=Qtr_bearing_t,flange_t=Qtr_bearing_flange_t,od=Qtr_bearing_od,id=hole_qtr_inch,flange_od=Qtr_bearing_flange_od);
   translate([0,0,wBC_inside+2*wall_t]) Bearing_Flanged (t=Qtr_bearing_t,flange_t=Qtr_bearing_flange_t,od=Qtr_bearing_od,id=hole_qtr_inch,flange_od=Qtr_bearing_flange_od);
}

module tube_arm (length=50, w=10, t=1) { // NOT USED PRESENTLY
    $fa=$preview ? 6 : 1; // minimum angle fragment

    color("blue") difference () {
        translate([-w/2,-w/2,-w/2]) cube([length+w,w,w],center=false);
        // remove bore for bearings
        cylinder(h=3*w,d=Qtr_bearing_od,center=true);
        translate([length,0,0]) cylinder(h=3*w,d=Qtr_bearing_od,center=true);
        // remove center of tube
        offset=w/2-t/2;
        translate([-w,-offset,-offset]) cube([length+2,2*offset,2*offset],center=false);
    }    
    if (display_all) {
       translate([0,0,-w]) B_Drive_at_B_Pulley ();
       translate([0,0,w/2]) bearing_flng_qtr ();
       translate([0,0,-w/2]) rotate([180,0,0]) bearing_flng_qtr ();
    }
}
module B_Drive_at_B_Pulley () {
    color ("green") difference () {
       pulley_gt2_2 ( teeth = AB_pulley_teeth , pulley_t_ht = AB_pulley_t);
        // add hex bore
       hex (size=22.94,l=AB_pulley_t+.5);
    }
}
module plastic_screw() {
    translate([0,0,20/2])cylinder(h=21,d=2.8,center=true);
    translate([0,0,-2.5/2]) cylinder(h=2.5,d=7,center=true);
}

module B_drive_at_A_Pulley () {
    // This pulley is on the outside of the AB arm at A
    // PRINT THIS IN TWO PARTS
    $fn=$preview ? 64 : 128; // minimum angle fragment
    
    pulley_OD = tooth_spacing (AB_pulley_teeth,2,0.254) +2;
    
    difference () {
        union () {
            // PART A
            pulley_gt2_2(teeth=AB_pulley_teeth,pulley_t_ht=AB_pulley_t ,motor_shaft=hole_qtr_inch);
            translate([0,0,AB_pulley_t*1.5]) 
                cylinder(h=AB_pulley_t,d=pulley_OD,center=true); // big boss A side
            
            // PART B
            translate([0,0,-3]) 
                cylinder(h=6,d=pulley_OD,center=true); // big boss B side
            translate([0,0,-AB_boss_t/2]) 
                rotate([0,0,-10]) 
                    rotate_extrude(angle=60,convexity = 10)
                        translate([20, 0, 0]) 
                            square([15,AB_boss_t],center=true);

            }
        // remove the servo horn shape
        rotate([0,0,20]) translate([0,0,-AB_boss_t-1]) servo_horn();
        // remove the spring
        translate([0,0,-2]) rotate([0,0,-90])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd*1.2,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        translate([-30,-50,10]) cube([40,40,20],center=false);
        translate([-50,-11,10]) cube([40,40,20],center=false);
        translate([0,0,13.5]) cube([21,21,7],center=true);
        // remove the A hole!
        cylinder(h=10*AB_boss_t,d=M6_bearing_od,center=true);
        // remove the screw holes that hold the two parts together
        rotate([0,0,70]) translate([16,0,-4]) plastic_screw();
        rotate([0,0,160]) translate([16,0,-4]) plastic_screw();
        rotate([0,0,250]) translate([16,0,-4]) plastic_screw();
        rotate([0,0,335]) translate([16,0,-4]) plastic_screw();

        // remove the outside bearing flange cylinder
        *translate([0,0,-AB_pulley_t])
            cylinder(h=AB_pulley_t,d=Qtr_bearing_flange_od+.5,center=false);
    }
   translate([0,0,-6]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   translate([0,0,AB_pulley_t*2]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
}

module final_C_horn(){
    // Pulley at joint C
    // This is the arm end-effector interface
    // center is 0,0,0 on xy plane, length is +x
    // The belt can be slipped over the pulley
    // The interface is a standard width
    $fa=$preview ? 3 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)

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
        translate ([End_pin_x,End_pin_y,0])
            cylinder(2*t_at_C,d=hole_qtr_inch,center=true);
    }
    //translate([0,0,-t_at_C/2+10]) rotate([180,0,0]) bearing_flng_qtr ();
    //translate([0,0,t_at_C/2+offx/2]) bearing_flng_qtr ();
    module ear () {
        End_angle = atan2(End_pin_y,End_pin_x);
        translate ([End_pin_x,End_pin_y,End_w/2+End_w/4]) 
            cube([hole_qtr_inch*3,hole_qtr_inch*3,End_w/2],center=true);
        translate ([0,-hole_qtr_inch*2,End_w/2]) 
            rotate([0,0,End_angle])
            cube([End_x,hole_qtr_inch*4,End_w/2],center=false);
    }
}
module final_fork (t_forks=1.3,l_fork=2,d_fork=0.2) {
    // Fork that attaches to End Effector
    // center is 0,0,0 on xy plane, length is +x
    // t_end is the end effector inteface thickness
    $fa=$preview ? 6 : 1; // minimum angle fragment

    difference () {
        union () { 
            // interface center
            translate ([End_x-End_pin_x,-End_pin_y,-End_w/2])
                cube([End_pin_x,End_pin_y,End_w],center=false);
            right_fork();
            // make a mirror copy
            mirror ([0,0,1]) right_fork();
        }
        // remove attach pin
        translate ([End_pin_x,-End_pin_y/2,0])
            cylinder(2,d=hole_qtr_inch,center=true);
        // remove cylinder to clear pulley
        cylinder(End_w*2,d=1.1*end_pulley_d,center=true);
    }
    module right_fork() {
        // interface back
        translate ([End_x,-End_pin_y,0]) {
            cube([0.2,End_pin_y,t_forks/2],center=false);
            rotate ([-90,0,0]) 
                translate ([d_fork/2,t_forks/2,End_pin_y])
                    cylinder(2*End_pin_y,d=d_fork,center=true);
        }
        // forks 
        translate([End_x+l_fork/2,-End_pin_y+d_fork/2,t_forks/2-d_fork/2])
            rotate([0,90,-5])
                fork(l_fork,d_fork);
}
}
module final_claw(){
    // DRAW THE COMPLIANT CLAW
    servo_plate_t = 8;
    back_plate_w = claw_width - 4*claw_radius;
    $fa=$preview ? 6 : 1; // minimum angle fragment
    union () {
        difference () {
            union () { 
                // interface center
                translate ([End_pin_x,End_pin_y,0])
                    cube([hole_qtr_inch*3,hole_qtr_inch*3,End_w],center=true);
                // interface top
                translate ([End_x+claw_radius,claw_height/2-servo_plate_t,-back_plate_w/2+5]) 
                    cube([50,servo_plate_t,back_plate_w+10],center=false);
            }
            // remove attach pin
            translate ([End_pin_x,End_pin_y,0])
                cylinder(2*End_w,d=hole_qtr_inch,center=true,$fn=32);

            // remove the servo interface
            translate([claw_servo_x,claw_height/2-svo_flange_d,0]) rotate([0,90,-90]) servo_body(vis=false,$fn=32);
                        
        }
    }
    translate([End_x,-claw_height/2,0]) rotate([0,-90,-90])     
    compliant_claw2 (len=claw_length,width=claw_width,t1=1.73,t2=claw_height,r=claw_radius,pre_angle=15);
}
module claw_shooter (curved=true) {
    union () { 
        // top plate attachment
        translate ([End_x+1.8,.7,-.8]) 
            rotate([0,0,-15])
                difference () {
                    cube([.12,1,1.6],center=false);
                    translate([0,.5,.4]) 
                        rotate([0,90,0]) 
                            cylinder(h=1,d=hole_M3_inch,center=true,$fn=16);
                    translate([0,.5,1.2]) 
                        rotate([0,90,0]) 
                            cylinder(h=1,d=hole_M3_inch,center=true,$fn=16);
                }
                if (curved) {
                    translate ([3.25,-4.41,0]) rotate([0,0,55]) 
                    rotate_extrude(angle=35,$fn=156) 
                    polygon([[6,.8],[6.08,.8],[6.08,-.8],[6,-.8],[6,.8]]);
                } else {
                    translate([5.,1.6,0])
                    cube([3.5,.08,1.6],center=true);
                }
    }
}
module robot_arm_base () {
    // Base of the arm
    // XY = HORIZON
    //   Nothing elegant about this code.
    $fa=$preview ? 6 : 1; // minimum angle fragment

    base_w = 102;
    base_l = 102;
    base_t = 7;
    
    base_y_shift = 7.6;
    
    // parameters for the 4 attach bolts
    hole_space = 81.28;
    x_b = hole_space/2;
    y_b = hole_space/2;

    // paramenters for the base gussetts
    t_guss = 7.5;
    h_guss = 12.7;
    x_guss = 17.8;
    
    // Inside AB Link, B side
    y1=center_t/2+base_svo_lug_t+Qtr_bearing_flange_t;
    
    // Inside AB link, A side
    y2 = -center_t/2-Qtr_bearing_flange_t;
    
    // extra lug y location, USED IN THE BASE
//extra_lug_y = y1+center_t+2*base_svo_lug_t+Qtr_bearing_flange_t+AB_pulley_t;
extra_lug_y = 47;
echo(y1=y1,extra_lug_y=extra_lug_y);
    
    difference () {
        union () {
            translate([0,base_y_shift,-base_z_top - base_t/2])
                rounded_cube(size=[base_w,base_l,base_t],r=hole_qtr_inch,center=true);
            
            // B Servo Side
            color("blueviolet") translate([0,B_servo_y,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=13,w=base_w/1.6,h=base_z_top+12,t=base_svo_lug_t);      

            // The Third lug
            color("fuchsia") translate([0,extra_lug_y,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=widthAB/3,w=widthAB/1.5,h=base_z_top,t=base_svo_lug_t);
            
            // Inside AB Link, B side
            color("turquoise") translate([0,y1,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=widthAB/2,w=widthAB,h=base_z_top,t=base_svo_lug_t);
          
            // Inside AB link, A side
            color("lime") translate([0,y2,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=widthAB/2,w=widthAB,h=base_z_top,t=base_svo_lug_t);      
            
            // A Servo Side
            color("yellow") translate([0,A_servo_y,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=13,w=base_w/1.6,h=base_z_top+12,t=base_svo_lug_t);    
            
            // base gussetts
            translate([x_guss,A_servo_y,-base_z_top])
                cube([t_guss,B_servo_y-A_servo_y,h_guss],center=false);
            translate([-x_guss-t_guss,A_servo_y,-base_z_top])
                cube([t_guss,B_servo_y-A_servo_y,h_guss],center=false);
            translate([-x_guss,y1,-base_z_top])
                cube([2*x_guss,extra_lug_y-y1+base_svo_lug_t,h_guss],center=false);
                
        }
        // subtract joint A bore
        translate([0,0,0])
            rotate([90,0,0])
                cylinder(h=base_l*2,d=hole_qtr_inch,center=true);
        
        // subtract A servo interface
        translate([0,A_servo_y-base_svo_lug_t+svo_flange_d,0])
            rotate([-90,-90,0])
                servo_body (vis=false);
        
        // subtract B servo interface
        translate([0,B_servo_y-svo_flange_d,0])
            rotate([90,-90,0])
                servo_body (vis=false);
        
        
        // subtract the 4 base mounting bolt holes
        translate([x_b,y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_qtr_inch,center=true);
        translate([x_b,-y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_qtr_inch,center=true);
        translate([-x_b,-y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_qtr_inch,center=true);
        translate([-x_b,y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_qtr_inch,center=true);
    }
}

module draw_assy (A_angle=0,B_angle=0,C_angle=0) {
    // XZ = HORIZON
    // calculate b and c positions from angles
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  // B location
    c = [(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),0];
    // lower arm vector
    vecAB=[b[0]/lenAB,b[1]/lenAB,b[2]/lenAB];
        
    // draw the upper and lower arms
    rotate([0,0,A_angle]) {
        // Draw the AB link
        color("plum",1) 
            translate ([0,0,-widthAB/2]) {
                final_AB_arm ();
       translate([0,0,wall_t]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
       translate([0,0,wAB_inside+wall_t]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
            }
        
        // Draw the BC link
        translate([lenAB,0,-widthBC/2-wall_t-Qtr_bearing_flange_t]) {
            rotate([0,0,B_angle-A_angle]) final_BC_arm ();
            rotate ([180,0,0]) B_Drive_at_B_Pulley ();
            //translate([0,0,3*wall_t]) rotate([0,0,180]) final_B_Spring_pulley();

        }
    }
    // Draw the end effector
    translate([c[0],c[1],c[2]-wall_t-Qtr_bearing_flange_t]) 
        rotate ([0,0,C_angle])  end_effector_assy();
    
    // Draw the B drive pulley at A
    yb=-(center_t/2+2*base_svo_lug_t+Qtr_bearing_flange_t+AB_pulley_t*1.4);
    color("navy",1)  
        rotate([0,0,(B_angle)]) 
            translate([0,0,yb]) B_drive_at_A_Pulley ();
    
    // A Servo
    color ("red",.5) rotate([0,180,-90])
        translate([0,0,A_servo_y+base_svo_lug_t/2]) {
            servo_body();
            rotate([0,0,90-A_angle])
                servo_horn();
        }
    // B Servo
    color ("red",.5) rotate([0,0,90])
        translate([0,0,-B_servo_y+svo_flange_d]) {
            servo_body();
            rotate([0,0,B_angle-60])
                servo_horn();
        }
    // C Servo
    color ("red",.5) 
        translate([c[0],c[1],c[2]-widthBC/2+1.5]) {
            rotate([0,0,-45]) servo_body();
            rotate([0,0,C_angle])
                servo_horn();
        }
            
    // B drive belt (displayed with base assembly)
    color("blue") 
        pt_pt_belt([0,0,-widthAB/1.3],[b[0],b[1],-widthAB/1.3],d=6,r_pulley=AB_pulley_d/2,round=false);

    // BASE    
    color("green",.5)
        rotate([-90,0,0])
                robot_arm_base (); 

    // Draw springs
        
    // torsion spring at A
    translate([0,0,-center_t/2]) torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH);
        
    translate([0,0,yb]) torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH);
    
    //spr_pt_AB = spr_dist_AB*vecAB;
    //echo(spr_pt_AB=spr_pt_AB);
    //B_spr_pt = [B_spr_r*cos(B_angle),B_spr_r*sin(B_angle),-widthAB ];

} 
