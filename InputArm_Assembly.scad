// Input Arm Assembly
//  Design for Human Input Arm that drives Robot Arm
//  last modified June 18 2021 by SrAmo
include <InputArm-Configuration.scad>
use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
use <Pulley-GT2_2.scad>

// Draw the Input Arm Assembly
display_assy = false;
// Draw the extra?? stuff
display_all = true;
// Section cut Assy at X = 0?
clip_yz = false;
// Section cut Assy at Z = 0?
clip_xy = false;
// Draw Final Base
display_base = false;
// Draw Final arm (shared)
display_arm = true;
// Draw Final hand 
display_hand= false;
// Draw Final finger 
display_finger= false;

if (display_assy) {
    difference () {
        draw_assy(90,90,0,full=display_all);
        if (clip_yz) // x = xcp cut 
            translate ([-200,-100,-100]) cube (200,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-100,-100,-200]) cube (200,center=false);
        }
    }

if (display_arm)  final_arm(length=lenAB);
if (display_base) input_arm_base();
if (display_hand)  final_hand();
if (display_finger)  final_finger();
    
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
        
            translate([lenAB,0,-widthAB/2]) 
                rotate([0,0,B_angle-A_angle]) {
                    color("blue",1) final_arm(length=lenBC);
                    translate([0,0,1])
                        rotate([0,0,90-lug_hook_ang])
                            color("red",1) P090S_pot(negative=false);

                }
    }
    // Draw the end effector
    translate([c[0],c[1],c[2]]) 
        rotate ([0,0,C_angle]) end_effector_assy();

    // BASE    
    color("green") translate([0,0,-pot_lug_t/2]) rotate([-90,180,0]) input_arm_base();
} 
module end_effector_assy() {
    translate([0,0,armt]/2) rotate([90,0,-90]) final_hand();
    translate([29,pot_lug_t/2,0]) rotate([90,0,0]) final_finger();
    }

module offset_link (length=100,d_pin=2,w=15,t=15,offset=4,ang=lug_hook_ang,wall=2) {
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
                
                // pot bracket
                translate([length/2,0,0]) mirror([0,0,1]) 
                    rotate([0,0,-90+ang])
                        translate([0,-5,0]) cube([w,14,15],center=true); 
                
            };
        rotate([0,0,-(90-ang)/1])
        translate([-w/1.1,-w*1.1,t/2+pot_lug_t/2]) 
            cube([1.5*w,2.2*w,t],center=false); // lug removal
        translate([-w/1.2,-w*1.1,t/2-pot_lug_t/2]) mirror([0,0,1]) 
            cube([1.5*w,2.2*w,t],center=false); // lug removal, pot side

        translate([length,0,t/2])
           cylinder(h=t,d=d_pin,center=false); // pin remove 
        translate([length-w/1.4,-w*1.1,t/2-pot_lug_t/2-.1]) 
            cube([1.4*w,2.2*w,pot_lug_t+.2],center=false); // clevis remove
    }
}

module final_arm (length=10) {
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.01; // minimum size of fragment (default is 2)

    difference () {
        mirror([0,1,0]) offset_link(length=length,d_pin=pinSize,w=widthAB,t=armt,offset=widthAB/2.1,ang=lug_hook_ang,wall=wall_t,$fn=48); 
        // remove the potentiometer interfaces
        translate([0,0,1]) rotate([0,0,90+lug_hook_ang]) P090S_pot(negative=true);
        translate([length,0,1]) rotate([0,0,90-lug_hook_ang]) P090S_pot(negative=true);
        // remove wire holes
        //   long hole
        translate([length/2,widthAB-wire_hole_offset,armt/2]) 
            rotate ([0,92,0]) cylinder(h=length*2,d=wire_hole_dia,center=true);
        //   diagonal hole 1
        translate([length/4,widthAB-wire_hole_offset,armt/1.9]) 
            rotate ([0,120,0]) 
                translate([0,0,length/4]) cylinder(h=length/2,d=wire_hole_dia,center=true);
        //   diagonal hole 1
        translate([length/1.3,widthAB-wire_hole_offset,armt/2]) 
            rotate ([0,-135,0]) 
                translate([0,0,length/2]) cylinder(h=length,d=wire_hole_dia,center=true);
        //   donut hole
        translate([0,0,armt/2-1]) rotate_extrude(convexity = 10, $fn = 48) {
            translate([widthAB/1.3, 0, 0]) circle(d=wire_hole_dia*1.2, $fn = 48);
            translate([widthAB/1.7, 0, 0]) circle(d=wire_hole_dia*1.2, $fn = 48);
        }
        // twist tie hole
        //translate([widthAB,-widthAB/2,0]) cylinder(h=4*widthAB,d=wire_hole_dia,center=true);
   }
}
module finger_ring(length=20,height=10,inside_dia=16) {
   // Finger Ring
   finger_width =6;
   difference() {
       cylinder(h=height,d=inside_dia+4,center=false);
       translate([0,0,-height/2]) cylinder(h=2*height,d=inside_dia,center=false);
   }
   translate([-finger_width/2,-length,0]) cube([finger_width,length-9,height],center=false);
}
module final_hand(length=15){
    // Claw that attaches to End Effector
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)
    difference () {
            union () {  
                // LUG
                translate ([0,pot_lug_t/2,-length]) rotate([90,0,0]) 
                    lug (r=widthAB/2,w=widthAB,h=length,t=pot_lug_t,d=.1);
                
                // CLEVIS
                 translate ([armt/2,0,-length+1]) rotate([-90,0,90]) 
                    lug (r=widthAB/2,w=widthAB,h=length,t=armt,d=.1);
                // pot bracket
                translate([length,0,-length*1.6]) cube([10,widthAB,widthAB],center=true); 
            };
        // clevis remove slot
        translate([0,0,-length*2]) 
           cube([pot_lug_t+.6,length*2,length+4],center=true); 
        // remove lug end Potentiometer
        translate([0,8,0]) rotate([90,0,0]) P090S_pot(negative=true);
        // remove clevis end Potentiometer
        translate([armt/2,0,-length*2+1]) 
            rotate([90,0,-90]) P090S_pot(negative=true);
        // pin remove 
        translate([0,0,-2*length+1])
           rotate([0,-90,0]) cylinder(h=armt,d=pinSize,center=false); 
        // remove donut hole for wire
        rotate([90,0,0]) rotate_extrude(convexity = 10, $fn = 48) 
            translate([widthAB/1.3, 0, 0]) circle(d=wire_hole_dia*1.2, $fn = 48);

        } 
    // ADD the finger loop
    translate([-armt/2,25,-length*2.4]) 
        rotate([90,135,90]) finger_ring(30,armt,inside_dia=20);

}
module final_finger() {
    translate([0,-12,0]) rotate([0,0,180]) finger_ring(12,pot_lug_t);
    difference() {
        lug (r=widthAB/2,w=widthAB/2,h=10,t=pot_lug_t,d=.1);
        // remove lug end Potentiometer
        translate([0,10,pot_lug_t*1.5]) 
        rotate([0,180,0]) P090S_pot(negative=true);
    }
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
            
           //rotate([-90,lug_hook_ang-90,0]) // rotate link so that lug points up
           rotate([-90,-45,0]) // rotate link so that lug points up
            translate([-lenAB,0,-base_l/2+armt/2]) 
                difference () {
                    mirror([0,1,0]) offset_link(length=lenAB,d_pin=pinSize,w=widthAB,t=armt,offset=widthAB/2,ang=lug_hook_ang,wall=wall_t,$fn=48); 
                    // remove the potentiometer interfaces
                    translate([lenAB,0,1]) rotate([0,0,90-lug_hook_ang]) P090S_pot(negative=true);
                }
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
        // remove wire donut hole
        translate([0,4 ,0]) rotate([90,0,0]) 
            rotate_extrude(convexity = 10, $fn = 48) 
            translate([widthAB, 0, 0]) circle(d=wire_hole_dia*1.5, $fn = 48);
       // twist tie hole
        translate([-10,0,-base_z_top+4]) rotate([90,0,0])
            cylinder(h=4*widthAB,d=wire_hole_dia,center=true);

    }
}
