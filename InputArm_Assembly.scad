// Input Arm Assembly
//  Design for Human Input Arm that drives Robot Arm
//  last modified APR 4 2021 by SrAmo
include <InputArm-Configuration.scad>
use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
use <Pulley-GT2_2.scad>

// Draw the Input Arm Assembly
display_assy = true;
// Draw the extra?? stuff
display_all = false;
// Section cut Assy at X = 0?
clip_yz = false;
// Section cut Assy at Z = 0?
clip_xy = false;
// Draw Final Base
display_base = false;
// Draw Final arm (shared)
display_arm = false;
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
if (display_claw)  final_hand();
    
module draw_assy (A_angle=0,B_angle=0,C_angle=0,full=true) {
    // calculate b and c positions from angles
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  // B location
    c = [(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),-pot_lug_t];
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
                        rotate([0,0,135])
                            P090S_pot(negative=false);

                }
    }
    // Draw the end effector
    translate([c[0],c[1],c[2]]) 
        rotate ([0,0,C_angle]) end_effector_assy();

    // BASE    
    color("green") translate([0,0,pot_lug_t]) rotate([-90,0,0]) input_arm_base();
} 
module end_effector_assy() {
    final_hand();
    final_finger();
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
                    rotate([0,0,-90+ang])
                        translate([0,-4,0])
                        cube([w,13,15],center=true); // pot bracket
                // zip tie boss
                translate([3,0,t/2]) cylinder(h=t,d=5.8,center=true);
                
            };
        translate([-w/1.4,-w*1.1,t/2+pot_lug_t/2]) 
            cube([1.4*w,2.2*w,t],center=false); // lug removal
        translate([-w/1.4,-w*1.1,t/2-pot_lug_t/2]) mirror([0,0,1]) 
            cube([2*w,2.2*w,t],center=false); // lug removal, pot side

        translate([length-w/1.4,-w*1.1,t/2-pot_lug_t/2-.3]) 
            cube([1.4*w,2.2*w,pot_lug_t+.6],center=false); // clevis remove
        translate([length,0,t/2])
           cylinder(h=t,d=d_pin,center=false); // pin remove 
        // zip tie hole
        //translate([length/2-3,0,t/2.1]) cylinder(h=t*2,d=6,center=true);
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
        //zip_tie_holes
        translate([length/2+3,0,widthAB]) cylinder(h=3*widthAB,d=3,center=true);
        translate([length/2+3,-6,widthAB]) cylinder(h=3*widthAB,d=3,center=true);
   }
}

module final_hand(length=35,dog_angle=15){
    // Claw that attaches to End Effector
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)
    difference () {
        offset_link(length=length,d_pin=pinSize,w=widthAB,t=armt,offset=widthAB/6,ang=dog_angle,wall=wall_t,$fn=48); 
        translate([0,0,1]) rotate([0,0,dog_angle]) P090S_pot(negative=true);
        translate([length,0,1]) rotate([0,0,dog_angle+90]) P090S_pot(negative=true);
        //zip_tie_holes
        translate([length/2,0,widthAB]) cylinder(h=3*widthAB,d=3,center=true);
        translate([length/2,-6,widthAB]) cylinder(h=3*widthAB,d=3,center=true);
   }
   translate([length+5,-20,0]) difference() {
       cylinder(h=armt,d=20,center=false);
       translate([0,0,-armt/2]) cylinder(h=2*armt,d=16,center=false);
   }
   translate([length-5,-8,0]) rotate([0,0,-90+dog_angle]) cube([7,10,armt],center=false);
}
module final_finger() {
}
module input_arm_base () {
    // Base of the input arm
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=$preview ? 0.05 : 0.03; // minimum size of fragment (default is 2)

    base_w = 60;
    base_l = 30;
    base_t = 3;
    base_z_top = 22;
    
    // parameters for the 4 attach bolts
    x_b = base_w/2-5;
    y_b = base_l/2-5;

    difference () {
        union () {
            translate([0,0,-base_z_top- base_t/2])
                rounded_cube(size=[base_w,base_l,base_t],r=2,center=true);
            
           rotate([-90,-45,0]) 
            translate([-lenAB,0,-base_l/2+armt/2]) 
                final_arm(length=lenAB);
        }
        // remove the extra part of the arm
        translate([-base_w,-base_l,-30-base_z_top-base_t]) 
            cube([2*base_w,2*base_l,30],center=false);
        // subtract the 4 base mounting bolt holes
        translate([x_b,y_b,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
        translate([x_b,-y_b,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
        translate([-x_b,-y_b,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
        translate([-x_b,y_b,-base_z_top])
                cylinder(h=base_t*3,d=4,center=true);
    }
}
