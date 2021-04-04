// Input Arm Assembly
//  Design for Human Input Arm that drives Robot Arm
//  last modified APR 2 2021 by SrAmo
include <InputArm-Configuration.scad>
use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
use <Pulley-GT2_2.scad>

// Draw the Input Arm Assembly
display_assy = false;
// Draw the extra?? stuff
display_all = false;
// Section cut Assy at X = 0?
clip_yz = false;
// Section cut Assy at Z = 0?
clip_xy = false;
// Draw Final Base
display_base = false;
// Draw Final arm (shared)
display_arm = true;
// Draw Final Claw 
display_claw= false;

if (display_assy) {
    difference () {
        draw_assy(90,0,0,full=display_all);
        if (clip_yz) // x = xcp cut 
            translate ([-200,-100,-100]) cube (200,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-100,-100,-200]) cube (200,center=false);
        }
    }

if (display_arm)  final_arm(length=lenAB);
if (display_base) input_arm_base();
if (display_claw)  final_claw();
    
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
                final_arm(length=lenAB);
        // Draw the BC link
        color("blue",1) 
            translate([lenAB,0,-widthAB/2]) 
                rotate([0,0,B_angle-A_angle]) {
                    final_arm(length=lenBC);
                    translate([0,0,1])
                        rotate([0,0,-45])
                            P090S_pot(negative=false);

                }
    }
    // Draw the end effector
    if (full) translate([c[0],c[1],c[2]]) 
        rotate ([0,0,C_angle]) end_effector_assy();

    // BASE    
    color("green",.5) rotate([-90,0,0]) input_arm_base();
} 
module end_effector_assy() {
    final_claw();
    }

module offset_link (length=100,d_pin=2,w=15,t=15,offset=5,ang=45,wall=2) {
    // Create a Offset Link on xy plane along the X axis 
    // First joint is at [0,0], Second joint is at [length,0]
    // The LUG will fit a pin of diameter d_pin
    // link width is w (y dimension), constant along length of part
    // thickness is t, positive z from zero
    // offset is the distance down from the x axis
    // ang is the dog leg angle
    // wall is the wall thickness of the link
    
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    d2 = offset/sin(ang);   // length of second segment
    d1 = length/2 - d2*cos(ang); // length of first segment
    fillet_r = t/4;        
    difference () {
        translate ([length/2,0,0])
            union () {  
                dog_leg (d1,ang,d2,w,t);
                mirror ([1,0,0]) // make a mirror copy
                    dog_leg (d1,ang,d2,w,t);
                translate([length/2,0,0]) mirror([0,0,1]) 
                    rotate([0,0,-45])
                        translate([0,-4,0])
                        cube([w,13,15],center=true); // pot bracket
            };
        translate([-w/1.4,-w*1.1,t/2+pot_lug_t/2]) 
            cube([1.4*w,2.2*w,t],center=false); // lug removal
        translate([-w/1.4,-w*1.1,t/2-pot_lug_t/2]) mirror([0,0,1]) 
            cube([2*w,2.2*w,t],center=false); // lug removal, pot side

        translate([length-w/1.4,-w*1.1,t/2-pot_lug_t/2-.3]) 
            cube([1.4*w,2.2*w,pot_lug_t+.6],center=false); // clevis remove
        translate([length,0,t/2])
           cylinder(h=t,d=d_pin,center=false); // pin remove 
    }
}

module final_arm (length=10) {
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.01; // minimum size of fragment (default is 2)

    mirror([0,1,0]) 
    difference () {
        offset_link(length=length,d_pin=pinSize,w=widthAB,t=armt,offset=widthAB/2,ang=45,wall=wall_t,$fn=48); 
        translate([0,0,1]) rotate([0,0,45]) P090S_pot(negative=true);
        translate([length,0,1]) rotate([0,0,135]) P090S_pot(negative=true);
        //zip_tie_holes(arm_l=length,arm_w=armt,dholes=2);
   }
}

module final_claw(){
    // Claw that attaches to End Effector
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)
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
