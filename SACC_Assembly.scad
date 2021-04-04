// Robot Arm Assembly
//  Started on 3/24/2020 by SrAmo
//  last modified FEB 7 2021 by SrAmo

use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
use <Pulley-GT2_2.scad>
//###### USE ONE CONFIG FILE AT A TIME #######//
//include <SACC-26-Configuration.scad>
include <InputArm-Configuration.scad>

// Draw the Robot Arm Assembly
display_assy = true;
// Draw the End effector, belts, springs 
display_all = false;
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
curved_shooter=true;
// Draw Final Forks (option to claw)
display_fork= false;
// Draw Final A Pulley
display_A_pulley= false;
// BC Arm Pulley at A
display_B_drive_pulley= false;
// BC Arm Pulley at B
display_B_drive_at_B_pulley= false;
// Draw Final B double wide Pulley
display_B_double_pulley= false;
// Draw Final End (C) Pulley
display_C_pulley= false;
// Draw Final D Pulley
display_D_pulley= false;
// Draw Belts for Export (TBD)
display_belts= false;

xcp = -20 + 0;   // x clipping plane

if (display_assy) {
    difference () {
        rotate ([90,0,0]) draw_assy(90,0,0,full=display_all);
        if (clip_yz) // x = xcp cut 
            translate ([xcp,-10,-10]) cube (20,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-10,-10,-20]) cube (20,center=false);
        }
    }

if (display_ABarm) rotate([-90,0,0]) final_AB_arm ();
if (display_BCarm) {
    rotate([-90,0,0])
        final_BC_arm ();
        if (display_all && typeRobotArm) color ("green") rotate([90,0,0])
        B_Drive_at_B_Pulley ();
}
if (display_base) {
    difference () {
        robot_arm_base ();
        if (clip_yz) // x = 0 cut 
            translate ([-20,-10,-10]) cube (20,center=false);
    }
}
if (display_B_drive_pulley) rotate([180,0,0])final_B_drive_pulley ();
if (display_B_drive_at_B_pulley) B_Drive_at_B_Pulley ();
if (display_C_pulley) final_C_pulley ();
if (display_claw_shooter)  {
    color ("purple") claw_shooter(curved=curved_shooter);
}
if (display_claw)  final_claw();
if (display_fork)  final_fork();
if (display_B_double_pulley) final_bc_pulley (); 
if (display_A_pulley) final_A_pulley (); 
if (display_D_pulley) final_D_pulley (); 
if (display_belts) {
    // subtract 1/2% from overall belt length for good fit
    // 1.1 mm thick belts seem to work good with slicer
    // PETG works.  TPA is too soft.
    // C-servo Belt
    c_pulley_d = sqrt(c_svo_x*c_svo_x+c_svo_z*c_svo_z)/mm_inch;
    c_belt_teeth = floor(0.995*(c_pulley_d+end_pulley_teeth));
    belt_circle(teeth=c_belt_teeth,ht=6,thk=1.1); 

    // A-B-C end Belts (need two)
    end_belt_teeth = floor(0.995*(lenAB/mm_inch+end_pulley_teeth));
    //belt_circle(teeth=end_belt_teeth,ht=6,thk=1.1); 

    // B-servo Belt
    b_svo_teeth = floor(0.995*(lenAB/mm_inch+bc_pulley_teeth));
    //belt_circle(teeth=b_svo_teeth,ht=6,thk=1.1); 

    echo(c_belt_teeth=c_belt_teeth,end_belt_teeth=end_belt_teeth,b_svo_teeth=b_svo_teeth);
}
   
module draw_assy (A_angle=0,B_angle=0,C_angle=0,full=true) {
    // calculate b and c positions from angles
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  // B location
    c = [(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),0];
    // lower arm vector
    vecAB=[b[0]/lenAB,b[1]/lenAB,b[2]/lenAB];
        
    // draw the upper and lower arms
    rotate([0,0,A_angle]) {
        // Draw the AB link
        color("plum",1) 
            translate ([0,0,-widthAB/2])
                final_AB_arm ();
        // Draw the BC link
        color("blue",1) 
            translate([lenAB,0,-widthBC/2-wall_t-bearing_flange_t-.01]) 
                rotate([0,0,B_angle-A_angle]) {
                    final_BC_arm ();
                    if(typeRobotArm) rotate ([180,0,0]) B_Drive_at_B_Pulley ();
                }
    }
    // Draw the end effector
    if (full) translate([c[0],c[1],c[2]-wall_t-bearing_flange_t-.01]) 
        rotate ([0,0,C_angle])  end_effector_assy();
    
    if (typeRobotArm) {
        // Draw the B drive pulley
        yb=-(extra_lug_y+base_svo_lug_t+bc_pulley_t*mm_inch+bearing_flange_t);
        color("navy",1)  
            rotate([0,0,(B_angle+180)]) 
                translate([0,0,yb])
                    final_B_drive_pulley ();
        
        // Draw the shared pulley at joint B
        color("lime") 
            translate ([ b[0],b[1],-.5])
                final_bc_pulley ();
                
        // Draw the pulley at joint A
        if (full) color("olive") 
            rotate ([0,0,-45+C_angle])
                translate ([0,0,-a_pulley_t*mm_inch/2])
                    final_A_pulley ();
        // A Servo
        color ("red",.5) rotate([0,180,-90])
            translate([0,0,A_servo_y+base_svo_lug_t/2]) {
                servo_body();
                rotate([0,0,45-A_angle])
                    servo_horn();
            }
        // B Servo
        color ("red",.5) rotate([0,0,90])
            translate([0,0,-B_servo_y+svo_flange_d]) {
                servo_body();
                rotate([0,0,B_angle+90])
                    servo_horn();
            }
        // C  Servo
        if (full) color ("red",.5) translate(c_servo_ctr)
            rotate([180,0,-20]){
                servo_body();
                final_D_pulley ();
            }
        
        // End effector belts
        if (full) {
            color("black") 
                pt_pt_belt([b[0],b[1],-0.25],[c[0],c[1],-0.25],d=belt_d,r_pulley=end_pulley_d/2,round=false);
            color("black") 
                pt_pt_belt([0,0,0],[b[0],b[1],0],d=belt_d,r_pulley=end_pulley_d/2,round=false);
            color("black") 
                pt_pt_belt([0,0,.2],[c_svo_x,c_svo_z,.2],d=belt_d,r_pulley=end_pulley_d/2,round=false);
        }
            
        // B drive belt (displayed with base assembly)
        color("navy") pt_pt_belt([0,0,-1.2],[b[0],b[1],-1.2],d=belt_d,r_pulley=bc_pulley_d/2,round=false);
    } else {
        // type is an input arm
        translate([0,0,-12]) P090S_pot (); // joint A pot
        translate([b[0],b[1],-12]) P090S_pot (); // joint B pot
    }

    // BASE    
    color("green",.5)
        rotate([-90,0,0])
            if (typeRobotArm) {
                robot_arm_base (); 
            } else {
                // type is input arm
                input_arm_base ();
            }

    if (full && typeRobotArm) {
        // Draw springs
        spr_pt_AB = spr_dist_AB*vecAB;
        B_spr_pt = [B_spr_r*cos(B_angle),B_spr_r*sin(B_angle),-1.6 ];
        // Draw the A spring (latex tube with pulleys)
        y_a_p = -A_servo_y-.35;
        color ("grey") 
        pt_pt_belt ([spr_pt_gnd[0],spr_pt_gnd[1],y_a_p+.15],[spr_pt_AB[0],spr_pt_AB[1],y_a_p+.15],d=.2,r_pulley=0.3,round=true);
        translate ([spr_pt_gnd[0],spr_pt_gnd[1],y_a_p]) GT2_2_idle_pulley ();
        translate ([spr_pt_gnd[0],spr_pt_gnd[1],y_a_p]) M5_RHS (length=20); 
        translate([spr_pt_AB[0],spr_pt_AB[1],widthAB/2+.05]) GT2_2_idle_pulley();
        translate ([spr_pt_AB[0],spr_pt_AB[1],widthAB/2+.4]) 
            rotate([0,180,0]) M5_RHS(length=55); 
        
        // Draw the B spring (latex tube with pulleys)
        y_b_p = -B_servo_y+.28;
        color ("grey") 
        pt_pt_belt ([spr_pt_gnd[0],spr_pt_gnd[1],y_b_p+.15],B_spr_pt,d=.2,r_pulley=0.3,round=true);
        translate ([spr_pt_gnd[0],spr_pt_gnd[1],y_b_p]) GT2_2_idle_pulley ();
        translate ([spr_pt_gnd[0],spr_pt_gnd[1],y_b_p+.35]) 
            rotate([0,180,0]) M5_RHS (length=20); 
    }
} 
module end_effector_assy() {
    if(typeRobotArm) {
        color("SpringGreen") final_C_pulley (); 
        color("green") final_claw();
        color ("purple") claw_shooter(curved=curved_shooter);
        color ("red",.5) translate([claw_y_parts+.3,.3,0]) rotate([0,90,-90]) servo_body();
    }
}
module final_AB_arm () {
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.01; // minimum size of fragment (default is 2)

    difference () {
        union () {
            hollow_offset_link(length=lenAB,d_pin=pinSize,w=widthAB,t=widthAB,offset=widthAB/2.5,ang=45,wall=wall_t,right=true,$fn=48); 
            
            if (typeRobotArm) {
                // add the spring attachment boss
                translate([spr_dist_AB,0,widthAB-wall_t])
                    cylinder(h=2*wall_t,d=2.5*hole_M5_inch,center=true);
                translate([spr_dist_AB,0,wall_t])
                    cylinder(h=2*wall_t,d=2.5*hole_M5_inch,center=true);
                
                // add the servo horn pad
                rotate([0,0,-45])
                    translate([0,widthAB*.25,widthAB+a_svo_boss/2])
                rounded_cube(size=[widthAB,widthAB*1.5,a_svo_boss],r=widthAB/2,center=true);
            }
        }
        if (typeRobotArm) {
            // remove the servo horn shape
            rotate([0,0,45])
                translate([0,0,widthAB-.01])
                    servo_horn ();
            // remove the spring bolt hole
            translate([spr_dist_AB,0,widthAB/2])
                cylinder(h=2*widthAB,d=hole_M5_inch,center=true);
            // remove the A hole!
            translate([0,0,0])
                cylinder(h=4*widthAB,d=bearing_od,center=true);
        }
        
        zip_tie_holes (arm_l=lenAB,arm_w=widthAB);
   }
   if (display_all && typeRobotArm) {// add bearings
       translate([0,0,wall_t]) bearing_flng_qtr ();
       translate([0,0,wAB_inside+wall_t]) rotate([180,0,0]) bearing_flng_qtr ();
   }
}

module final_BC_arm () {
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.01; // minimum size of fragment (default is 2)

    difference () {
        union () {
        hollow_offset_link (length=lenBC,d_pin=pinSize,w=widthBC,t=widthBC,offset=widthBC/2.5,ang=45,wall=wall_t,right=false,$fn=48); 
        if (typeRobotArm) {
            // union HEX for pulley
            hex_h = bc_pulley_t*mm_inch;
            translate ([ 0,0,-hex_h+bearing_flange_t])
                hex (size=.9,l=hex_h);
             // triangular support for printing
             rotate([0,90,0])
                translate([-wall_t/3,bearing_od/2+wall_t,0])
                linear_extrude(height = 0.036)
                    polygon(points=[[0,0],[hex_h,0],[0,hex_h],[0,0]]);
            }
        }
        // c-bore for bearing
        if (typeRobotArm) {
            cylinder(h=3*widthBC,d=bearing_od,center=true);
            zip_tie_holes (arm_l=lenBC,arm_w=widthBC);
        }
    }    
    if (display_all && typeRobotArm) {// add bearings
       translate([0,0,wall_t]) bearing_flng_qtr ();
       translate([0,0,-bc_pulley_t*mm_inch+bearing_flange_t]) rotate([180,0,0]) bearing_flng_qtr ();
       translate([0,0,wBC_inside+2*wall_t]) bearing_flng_qtr();
   }
}
module B_Drive_at_B_Pulley () {
    difference () {
        scale ([mm_inch,mm_inch,mm_inch])
           pulley_gt2_2 ( teeth = bc_pulley_teeth , pulley_t_ht = bc_pulley_t);
        // add hex bore
        hex (size=.903,l=bc_pulley_t*mm_inch+.1);

        // c-bore for bearing
        //cylinder(h=3*widthBC,d=bearing_od,center=true);
    }
}
module zip_tie_holes (arm_l = 10,arm_w=1) {
    rotate([90,0,0]) {
        translate ([.15*arm_l,.35*arm_w,-arm_w]) 
            cylinder(h=arm_w,d=hole_M3_inch,center=true);
        translate ([.15*arm_l,.65*arm_w,-arm_w]) 
            cylinder(h=arm_w,d=hole_M3_inch,center=true);
        translate ([.5*arm_l,.35*arm_w,-arm_w]) 
            cylinder(h=arm_w,d=hole_M3_inch,center=true);
        translate ([.5*arm_l,.65*arm_w,-arm_w]) 
            cylinder(h=arm_w,d=hole_M3_inch,center=true);
        translate ([.85*arm_l,.35*arm_w,-arm_w]) 
            cylinder(h=arm_w,d=hole_M3_inch,center=true);
        translate ([.85*arm_l,.65*arm_w,-arm_w]) 
            cylinder(h=arm_w,d=hole_M3_inch,center=true);
    }
}
module B_drive_pulley_at_B () {
    // pulley
    translate ([ 0,0,-bc_pulley_t*mm_inch])
    scale ([mm_inch,mm_inch,mm_inch])
        pulley_gt2_2(teeth=bc_pulley_teeth,pulley_t_ht= bc_pulley_t,motor_shaft=20);

}
module final_B_drive_pulley () {
    // This pulley is on the outside of the AB arm at A
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=$preview ? 0.05 : 0.02; // minimum size of fragment (default is 2)

    difference () {
        union () {
            scale ([mm_inch,mm_inch,mm_inch])
            pulley_gt2_2(teeth=bc_pulley_teeth,pulley_t_ht=bc_pulley_t ,motor_shaft=hole_p25_inch/mm_inch );
            intersection() {
                translate([0,-.5,-bc_boss_t])
                // extra boss for the servo to push on
                    cube([1.5,1,bc_boss_t],center=false);
                 translate([0,0,-bc_boss_t])
                   cylinder (h=bc_boss_t,d=bc_pulley_d,center=false );
            }
        }
        // remove the servo horn shape
        translate([0,0,-bc_pulley_t*mm_inch]) servo_horn();
        // remove the A hole!
        translate([0,0,0])
            cylinder(h=4*widthAB,d=bearing_od,center=true);
        // remove the outside bearing flange cylinder
        translate([0,0,-.5])
            cylinder(h=.5,d=bearing_flange_od+.01,center=false);
        // remove the spring hole
        translate([-B_spr_r,0,0]) 
            cylinder(h=2*widthAB,d=hole_M5_inch,center=true); // bolt hole
        // remove the spring nut hex hole
        translate([-B_spr_r,0,bc_pulley_t*mm_inch-.05]) 
            hex(size=0.312,l=.2); // nut hole
    }
   if (display_all) {// add bearings, pulley and screw
       rotate([180,0,0]) bearing_flng_qtr ();
       translate([0,0,bc_pulley_t*mm_inch]) bearing_flng_qtr ();
       translate([-B_spr_r,0,-(bc_pulley_t+2)*mm_inch]) GT2_2_idle_pulley();
       translate([-B_spr_r,0,-(bc_pulley_t+2)*mm_inch]) M5_RHS(length=20);
    }
}

module final_C_pulley(){
    // Pulley at joint C
    // This is the arm end-effector interface
    // center is 0,0,0 on xy plane, length is +x
    // The belt can be slipped over the pulley
    // The interface is a standard width
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)

    t_at_C=wBC_inside-2*bearing_flange_t; // thickness at C
    
    difference () {
        union () { 
            cylinder (t_at_C,d=end_pulley_d/1.5,center=true);
            translate ([0,0,-(end_pulley_t*mm_inch)/2])
                scale ([mm_inch,mm_inch,mm_inch])
                    pulley_gt2_2 ( teeth = end_pulley_teeth , pulley_t_ht = end_pulley_t,retainer=0,idler=0);
            ear();
            mirror ([0,0,1]) ear() ;// make a mirror copy
        }
        // remove bore bearing
        cylinder(2*t_at_C,d=bearing_od,center=true);
        
        // remove End attach pin
        translate ([End_pin_x,End_pin_y,0])
            cylinder(2*t_at_C,d=hole_p25_inch,center=true);
    }
    if (display_all) {// add bearings
       translate([0,0,-t_at_C/2]) rotate([180,0,0]) bearing_flng_qtr ();
       translate([0,0,t_at_C/2]) bearing_flng_qtr ();
    }
    module ear () {
        translate ([End_pin_x,End_pin_y,End_w/2+.1]) 
            cube([hole_p25_inch*3,hole_p25_inch*3,.2],center=true);
        translate ([0,-.3,End_w/2]) 
            rotate([0,0,End_angle])
            cube([1.2*End_x,.6,.2],center=false);
    }
}
module final_fork (t_forks=1.3,l_fork=2,d_fork=0.2) {
    // Fork that attaches to End Effector
    // center is 0,0,0 on xy plane, length is +x
    // t_end is the end effector inteface thickness
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)

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
            cylinder(2,d=hole_p25_inch,center=true);
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
    // Claw that attaches to End Effector
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)
    union () {
        difference () {
            union () { 
                // interface center
                translate ([End_pin_x,End_pin_y,0])
                    cube([hole_p25_inch*3,hole_p25_inch*3,End_w],center=true);
                // interface top
                translate ([End_x,.5,-0.8]) cube([1.8,.25,2.4],center=false);
                // top plate attachment (angled extention)
                translate ([End_x+1.65,.6,-.8]) 
                    rotate([0,0,-15]) 
                    difference () {
                        cube([.12,1,1.6],center=false);
                        translate([0,.6,.4]) 
                            rotate([0,90,0]) 
                                cylinder(h=1,d=hole_M3_inch,center=true,$fn=16);
                        translate([0,.6,1.2]) 
                            rotate([0,90,0]) 
                                cylinder(h=1,d=hole_M3_inch,center=true,$fn=16);
                    }
            }
            // remove attach pin
            translate ([End_pin_x,End_pin_y,0])
                cylinder(2,d=hole_p25_inch,center=true,$fn=32);
            // remove cylinder to clear pulley
            cylinder(End_w*2,d=1.1*end_pulley_d,center=true);
            // remove the servo interface
            translate([claw_y_parts+.3,.35,0]) rotate([0,90,-90]) servo_body(vis=false,$fn=32);
            // remove the top chamfer
            rotate([90,0,0])
                translate([1.2,.9,-1])
                linear_extrude(height = 1)
                        polygon(points=[[0,0],[.8,.8],[-.7,.7],[0,0]]);
            // remove second top chamfer
            rotate([90,0,0])
                translate([3,1.3,-1])
                linear_extrude(height = 1)
                        polygon(points=[[0,0],[.8,.8],[-.7,.7],[0,0]]);
            
        }
    }
    translate([.5,-.8,0]) rotate([0,-90,-90]) 
    compliant_claw2 (l=6,w=4.6,t1=0.068,t2=1.5,pre_angle=15);
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
module input_arm_base () {
    // Base of the input arm
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=$preview ? 0.05 : 0.03; // minimum size of fragment (default is 2)

    base_w = 30;
    base_l = 30;
    base_t = 2;
    base_z_top = 10;
    
    base_y_shift = 1;
    
    // parameters for the 4 attach bolts
    hole_space = 20;
    x_b = hole_space/2;
    y_b = hole_space/2;

    difference () {
        union () {
            translate([0,base_y_shift,-base_z_top- base_t/2])
                rounded_cube(size=[base_w,base_l,base_t],r=2,center=true);
            
            // B  Side
            color("blueviolet") translate([0,10,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=5,w=base_w/1.6,h=base_z_top,t=2);      

            
            // A  Side
            color("yellow") translate([0,-10,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=5,w=base_w/1.6,h=base_z_top,t=2);    
            
        }
        // subtract joint A bore
        translate([0,0,0])
            rotate([90,0,0])
                cylinder(h=base_l*2,d=4,center=true);
    
        // subtract A servo interface
        
        // subtract the 4 base mounting bolt holes
        translate([x_b,y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
        translate([x_b,-y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
        translate([-x_b,-y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
        translate([-x_b,y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
    }
}
module robot_arm_base () {
    // Base of the arm
    //   Nothing elegant about this code.
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=$preview ? 0.05 : 0.03; // minimum size of fragment (default is 2)

    base_w = 4;
    base_l = 4;
    base_t = 0.27;
    
    base_y_shift = .3;
    
    // parameters for the 4 attach bolts
    hole_space = 3.2;
    x_b = hole_space/2;
    y_b = hole_space/2;

    // paramenters for the base gussetts
    t_guss = 0.3;
    h_guss = 0.5;
    x_guss = 0.7;
    // Inside AB Link, B side
    y1=a_pulley_t*mm_inch/2+base_svo_lug_t+bearing_flange_t;
    // Inside AB link, A side
    y2 = -a_pulley_t*mm_inch/2-bearing_flange_t;
    //echo(y1=y1,y2=y2);
    
    difference () {
        union () {
            translate([0,base_y_shift,-base_z_top - base_t/2])
                rounded_cube(size=[base_w,base_l,base_t],r=hole_p25_inch,center=true);
            
            // B Servo Side
            color("blueviolet") translate([0,B_servo_y,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=.5,w=base_w/1.6,h=base_z_top+spr_dist_ground,t=base_svo_lug_t+.02);      

            // The Third lug
            color("fuchsia") translate([0,extra_lug_y+base_svo_lug_t,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=end_pulley_d/3,w=end_pulley_d/1.5,h=base_z_top,t=base_svo_lug_t);
            
            // Inside AB Link, B side
            color("turquoise") translate([0,y1,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=end_pulley_d/2,w=bc_pulley_d,h=base_z_top,t=base_svo_lug_t-.01);
          
            // Inside AB link, A side
            color("lime") translate([0,y2,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=end_pulley_d/2,w=bc_pulley_d,h=base_z_top,t=base_svo_lug_t);      
            
            // A Servo Side
            color("yellow") translate([0,A_servo_y,-base_z_top])
                rotate([90,0,0]) 
                    lug (r=.5,w=base_w/1.6,h=base_z_top+spr_dist_ground,t=base_svo_lug_t+.02);    
            
            // base gussetts
            translate([x_guss,A_servo_y,-base_z_top])
                cube([t_guss,B_servo_y-A_servo_y,h_guss],center=false);
            translate([-x_guss-t_guss,A_servo_y,-base_z_top])
                cube([t_guss,B_servo_y-A_servo_y,h_guss],center=false);
            translate([-x_guss,y1,-base_z_top])
                cube([2*x_guss,extra_lug_y-y1+base_svo_lug_t,h_guss],center=false);
                
            // add C servo plate
            translate([x_guss,y2-.47,-base_z_top-base_t])
                rotate([90,0,0])
                linear_extrude(base_svo_lug_t,convexity=10)
                polygon([[-2*x_guss,0],[2.4,0],[2.7,.8],[2.7,1.2],[1.3,1.8],[.5,1.8],[-2*x_guss,0]]);
            // add C bearing plate
            translate([x_guss,y1,-base_z_top-base_t])
                rotate([90,0,0])
                linear_extrude(base_svo_lug_t,convexity=10)
                polygon([[0,0],[2.1,0],[2.4,1.1],[.9,1.8],[0,1.8],[0,0]]);
        }
        // subtract joint A bore
        translate([0,0,0])
            rotate([90,0,0])
                cylinder(h=base_l*2,d=hole_p25_inch,center=true);
        
        // subtract spring point
        translate([0,0,spr_dist_ground])
            rotate([90,0,0])
                cylinder(h=base_l*2,d=7*mm_inch,center=true);
        
        // subtract A servo interface
        translate([0,A_servo_y-base_svo_lug_t+svo_flange_d,0])
            rotate([-90,-90,0])
                servo_body (vis=false);
        
        // subtract B servo interface
        translate([0,B_servo_y-svo_flange_d,0])
            rotate([90,-90,0])
                servo_body (vis=false);
        
        // subtract C servo interface
        translate([c_svo_x,-.59,c_svo_z])
            rotate([-90,20,0])
                servo_body(vis=false);

        // subtract C servo bearing support (add hex for nut)
        translate([c_svo_x,c_svo_y,c_svo_z])
            rotate([90,0,0])
                cylinder(h=base_l,d=hole_p25_inch,center=true);
         translate([c_svo_x,y1-.1,c_svo_z])
            rotate([90,0,0]) hex(size=0.435,l=.2); // nut hole
        
        // subtract the 4 base mounting bolt holes
        translate([x_b,y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_p25_inch,center=true);
        translate([x_b,-y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_p25_inch,center=true);
        translate([-x_b,-y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_p25_inch,center=true);
        translate([-x_b,y_b+base_y_shift,-base_z_top])
                cylinder(h=base_t*3,d=hole_p25_inch,center=true);
    }
}
module final_bc_pulley () {
    // double wide pulley at joint B
    // width is hard coded (future improvement)
    scale ([mm_inch,mm_inch,mm_inch])
    pulley_gt2_2 ( teeth = end_pulley_teeth , pulley_t_ht = 20 ,motor_shaft=bearing_od/mm_inch );
    
   if (display_all) {  // add bearings
       rotate([180,0,0]) bearing_flng_qtr ();
       translate([0,0,20*mm_inch]) bearing_flng_qtr ();
   }

}
module final_A_pulley () {
    // Pulley located at the middle of the A joint,
    // that drives the end effector wrist rotation (C)
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)

    difference () {
        scale ([mm_inch,mm_inch,mm_inch])
            pulley_gt2_2(teeth = end_pulley_teeth,pulley_t_ht=a_pulley_t+1,motor_shaft=bearing_od/mm_inch,retainer=0,idler=0);
        cylinder(h=1,d=bearing_od,center=true);
        translate ([0,0,(a_pulley_t+1)*mm_inch])
            cylinder(h=2*bearing_flange_t,d=bearing_flange_od,center=true);
        cylinder(h=2*bearing_flange_t,d=bearing_flange_od,center=true);

    }
   if (display_all) {// add bearings
       rotate([180,0,0]) bearing_flng_qtr ();
       translate([0,0,a_pulley_t*mm_inch]) bearing_flng_qtr ();
   }

}
module final_D_pulley () {
    // Bearing at the D servo, mouted to the base
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)

    ply_thk = 10; // mm
    union () {
        difference () {
        scale ([mm_inch,mm_inch,mm_inch])
            // remove outer flange on pulley for arm down 
            pulley_gt2_2 (teeth = end_pulley_teeth ,pulley_t_ht = ply_thk ,motor_shaft=9,retainer=1 );
        // four holes for attachment to horn
        for (a=[0:90:270]) rotate([0,0,a]) translate([7.32*mm_inch,0,0]) cylinder(h=1,d=2*mm_inch,center=true,$fn=16);
        }
        // add the boss on the ouside for the bearing
        translate([0,0,(ply_thk-2)*mm_inch])
            difference () {
                cylinder(h=bearing_t+.05,d=bearing_od+.25,center=false);
                translate ([0,0,-.02]) 
                    cylinder(h=bearing_t+.1,d=bearing_od,center=false);
            }
    }
   if (display_all) {// add bearings
       translate([0,0,(ply_thk-2)*mm_inch+bearing_t+.05]) bearing_flng_qtr ();
   }

}

