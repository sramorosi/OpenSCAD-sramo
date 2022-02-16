// Input Arm Assembly
//  Design for Human Input Arm that drives Robot Arm
//  last modified January 2022 by SrAmo
include <InputArm-Configuration.scad>
use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
//use <Pulley-GT2_2.scad>

AA = 5; // [0:130.0]
BB = 0; // [-145:1:130.0]

fascets = 40; // use 140 for printing

// Draw the Input Arm Assembly
display_assy = false;
// Section cut Assy at X = 0?
clip_yz = false;
// Section cut Assy at Z = 0?
clip_xy = false;

// number of fragments for display, ==> DISPLAY PERFORMANCE
donut_fragments = 16; // recommend 16 for Preview,  48 for Render

if (display_assy) {
    difference () {
        draw_assy(120,60,0,full=true);
        if (clip_yz) // x = xcp cut 
            translate ([-200,-100,-100]) cube (200,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-100,-100,-200]) cube (200,center=false);
        }
    }
    
module end_effector_assy() {
    translate([0,0,armt/2]) rotate([-90,0,90]) final_hand();
    translate([39,-4,10]) 
        rotate([90,90,180]) final_finger();
    }
module wire_path(length=30){
    cylinder(h=length*2,d=wire_hole_dia,center=true);
    translate([0,wire_hole_dia/1.9,0])
        cube([wire_dia,wire_hole_dia,length*2.2],center=true);
}
module single_offset_link (length=50,w=15,offset=7,d_pin=5,pot_short=false) {
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,-offset], Second joint is at [length,0]
    // link width is w (y dimension), constant along length of part
    // offset is the distance down from the x axis
    bend_ang = 45; // the angle of the bend
    long_leg = length-offset;
    short_leg = sqrt(2*offset*offset);
    echo(long_leg=long_leg,short_leg=short_leg);
    wire_hole_offset = offset+wire_hole_dia; // wire  hole offset
    difference () {
        union () {
            // Draw the main link
            dog_leg2 (d1=long_leg,ang=bend_ang,d2=short_leg,w=w,t=w);
            // pot bracket
            if(pot_short) { // short = AB arm
                translate([0,-offset,0]) 
                    rotate([0,0,bend_ang]) 
                        translate([w/3,0,w/1.2]) 
                            cube([w,w,w/1.2],center=true);
                } else { // BC arm
                    translate([length-w/3,0,w/1.2]) cube([w,w,w/1.2],center=true);
                }
            };
            // remove wire holes (same for both)
            //   diagonal hole 1
            translate([length/4,widthAB-wire_hole_offset,1]) 
                rotate ([0,30,0]) 
                    translate([0,0,length/4]) 
                        wire_path(length/4.2);
            //   diagonal hole 1
            translate([length/1.3,widthAB-wire_hole_offset,1]) 
                rotate ([0,-60,0]) 
                    translate([0,0,length/3.5]) 
                        wire_path(length/4.5);
            if(pot_short) {  // potentiometer on short end, AB arm
                // remove long wire hole
                translate([length/3.3,widthAB-wire_hole_offset,1.5]) 
                    rotate ([0,87,0]) 
                        wire_path(length/2);
                // remove either side of lug on long end
                translate([length-w*0.7,-w/1.1,pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                translate([length-w*0.7,-w/1.1,-w-pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                // remove clevis on short end
                translate([0,-offset,0]) {
                    rotate([0,0,bend_ang]) 
                        cube([1.4*w,2.2*w,pot_lug_t+clevis_gap],center=true);
                   cylinder(h=2*w,d=d_pin,center=true);
                }
                // remove the potentiometer interfaces
                translate([0,-offset,widthAB/2+1]) 
                    rotate([180,0,180-45]) 
                           P090S_pot(negative=true);
                translate([length,0,widthAB/2]) 
                    rotate([180,0,90]) 
                        P090S_pot(negative=true);
                // donut holes on long end for wires
                translate([length,0,pot_lug_t/2.5]) 
                    rotate_extrude(convexity = 10, $fn = donut_fragments) {
                        translate([widthAB/1.3, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                        translate([widthAB/1.8, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                    }
            } else { // potentiometer on long end, BC arm
                // remove long wire hole
                translate([length/3.3,widthAB-wire_hole_offset,1.5]) 
                    rotate ([0,90,0]) 
                        wire_path(length/2);
                // remove either side of lug on short end
                translate([0,-offset,0]) rotate([0,0,bend_ang]) {
                translate([-w*0.7,-w/1.1,pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                translate([-w*0.7,-w/1.1,-w-pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                }
                // remove clevis on short end
                translate([length,0,0]) {
                        cube([1.4*w,2.2*w,pot_lug_t+clevis_gap],center=true);
                   cylinder(h=2*w,d=d_pin,center=true);
                }
                // remove the potentiometer interfaces
                translate([0,-offset,-widthAB/2]) 
                    rotate([0,0,135]) 
                           P090S_pot(negative=true);
                translate([length,0,widthAB/2+1]) 
                    rotate([180,0,-90]) 
                        P090S_pot(negative=true);
                // donut holes on short end for wires
                translate([0,-offset,pot_lug_t/2.5]) 
                    rotate_extrude(convexity = 10, $fn = donut_fragments) {
                        translate([widthAB/1.3, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                        translate([widthAB/1.8, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                    }
            }
    }
}
module AB_arm_assy(B_angle = 0){
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,-offset], Second joint is at [length,0]
    offset = widthAB/2.1;
    hook_ang=atan2(offset,lenAB); // hook angle
//    echo("AB",length=length,hook_ang=hook_ang,offset=offset);
    translate ([lenAB,0,0])
    rotate([0,180,hook_ang]) 
        translate([0,offset,0]) { 
            //single_offset_link
            color("plum",1) 
single_offset_link(length=lenAB,w=widthAB,offset=offset,d_pin=pot_shaft_dia,pot_short=true,$fn=donut_fragments); 
//P090S_pot(negative=false);
        }
// DRAW THE BC ARM 
translate([lenAB,0,0]) 
rotate([0,0,B_angle]) {
    color("lightblue",1) BC_arm_assy(length=lenBC,offset=widthAB/2.1);
    translate([0,0,-widthAB/2-1])
        rotate([0,0,52])
            P090S_pot(negative=false);
        }

}
*AB_arm_assy();

module BC_arm_assy(length=10,offset=2){
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,-offset], Second joint is at [length,0]
//    $fa=$preview ? 6 : 1; // minimum angle fragment
//    $fs=0.01; // minimum size of fragment (default is 2)
    hook_ang=atan2(offset,length); // hook angle
    echo("BC",length=length,hook_ang=hook_ang,offset=offset);
    rotate([0,0,-hook_ang]) 
        translate([0,offset,0])  
            //single_offset_link 
            single_offset_link(length=length,w=widthAB,offset=offset,d_pin=pot_shaft_dia,pot_short=false,$fn=donut_fragments); 
}
module finger_ring(length=20,height=10,inside_dia=16) {
   // Finger Ring
   finger_width =6;
   lug (r=(inside_dia+4)/2,w=height,h=length,t=finger_width,d=inside_dia);
}
module final_hand(length=14){
    // Claw that attaches to End Effector
//    $fa=$preview ? 6 : 1; // minimum angle fragment
//    $fs=0.05; // minimum size of fragment (default is 2)
    difference () {
        union () {  
            // LUG
            translate ([0,pot_lug_t/2,-length]) rotate([90,0,0]) 
                lug (r=widthAB/2,w=armt,h=length,t=pot_lug_t,d=.1);
            
            // CLEVIS
             translate ([armt/2,0,-length+1]) rotate([-90,0,90]) 
                lug (r=widthAB/2,w=widthAB,h=length,t=armt,d=.1);
            // pot bracket
            translate([length,0,-length*1.6]) cube([10,widthAB,widthAB],center=true); 
            // ADD the finger loop
            translate([-armt/2,5,-length]) 
                rotate([-55,0,0])
                rotate([0,90,0]) finger_ring(30,height=10,inside_dia=20);
            };
        // clevis remove slot
        translate([0,0,-length*2]) 
           cube([pot_lug_t+clevis_gap,length*2,length+4],center=true); 
        // remove lug end Potentiometer
        translate([0,-pot_lug_t,0]) 
               rotate([90,0,180]) P090S_pot(negative=true);
        // remove clevis end Potentiometer
        translate([armt/2,0,-length*2+1]) 
            rotate([90,0,-90]) P090S_pot(negative=true);
        // pin remove 
        translate([0,0,-2*length+1])
           rotate([0,-90,0]) cylinder(h=armt,d=pot_shaft_dia,center=false); 
        // remove donut hole for wire
        rotate([90,0,0]) rotate_extrude(convexity = 10, $fn = donut_fragments) 
            translate([widthAB/1.8, pot_lug_t/2, 0]) circle(d=wire_hole_dia*1.8, $fn = donut_fragments);
        } 
}
module final_finger() {
    y_offset = 10;
    union() {
        difference() {
            lug (r=widthAB/2,w=10,h=y_offset,t=pot_lug_t,d=.1);
            // remove lug end Potentiometer
            translate([0,y_offset,pot_lug_t*1.5]) 
                rotate([0,180,0]) 
                    P090S_pot(negative=true);
        }
        translate([0,0,0])rotate([0,0,180]) 
            finger_ring(length=2*y_offset,height=10,inside_dia=20);
    }
}
module pot_joint(pot=true,lug_two = true) {
    // If pot = true then model the side that holds the pot
    // Else model the lug that goes on the shaft
    dbody = 26;
    zbody = 13;
    dlug = 18;
    difference () {
       // lug
        union() {
            if (pot) {
                translate([0,0,-zbody/2+1]) washer(d=dbody,t=zbody+2,d_pin=1,$fn=fascets);
                translate([0,0,2.4]) washer(d=dbody-10,t=5,d_pin=1,$fn=fascets);
                
                // lug opposite pot.  Large hole for access
                if (lug_two) translate([0,0,5+4+8]) washer(d=dbody,t=8,d_pin=7,$fn=fascets); 
            } else { // lug for shaft
                translate([0,0,5+4]) washer(d=dlug,t=8,d_pin=1,$fn=fascets);
            }
        }
        // remove potentiometer interfaces
        P090S_pot(negative=true);
    }
}
*pot_joint();
*translate([30,0,0]) {
    pot_joint(pot=false);
    color("red",1) P090S_pot(negative=false);
}
base_t = 6;
base_w = 60;
A_joint_z_shift = 11;

module base_turntable_model () {
    // Model of turntable Base, for the input arm

    // Potentiometer support for Joint A
    translate([-5,0,A_joint_z_shift+base_t/2]) rotate([0,90,0]) pot_joint();

    difference() {
        cylinder(h=base_t,d=base_w,center=true,$fn=fascets); // turntable
        
        // remove turntable T potentiometer interface
        translate([0,0,-7]) P090S_pot(negative=true);
        
        // Wire access holes
        rotate([0,0,90]) Rotation_Pattern(number=2,radius=base_w/4,total_angle=360)
                cylinder(h=base_t*3,d=12,center=true,$fn=fascets);
    }
}
module base_model (part_one = true) {
    // Model of Base, for the input arm (THE FIXED PART)
    // TWO PART MODEL. 1 = MAIN BASE,  2 = SCREW ON TOP
    
    bottom_h = 25;
    top_h = 4;
    add_d = 12;
    
    if (part_one) {
        difference() { 
            union () { // PART 1
                // Potentiometer support for Joint T
                translate([0,0,-9]) rotate([0,0,0]) pot_joint(pot=true,lug_two = false);
            
                translate([0,0,-bottom_h/2-base_t/2])
                    washer(d=base_w+add_d,t=bottom_h,d_pin=base_w-10,$fn=fascets); // bottom layer
                
                translate([0,0,-bottom_h])
                    washer(d=base_w-8,t=6,d_pin=20,$fn=fascets); // pot support
            
                // mid layer, outside of turn table
                washer(d=base_w+add_d,t=base_t+0.4,d_pin=base_w*1.01,$fn=fascets); 
            } // END UNION
            
            screw_holes(); // subtract top cap screw holes
            // subtract a big hole for the wires
            translate([0,-30,-8]) rotate([90,0,0]) cylinder(h=46,r=14,center=true,$fn=60);
        // bottom attach screw holes
        translate([0,0,-20]) Rotation_Pattern(number=4,radius=20,total_angle=360)
                cylinder(h=base_t*3,d=3,center=true,$fn=12);
            }
            
    } else { // PART TWO = top layer
        difference() { 
            translate([0,0,top_h/2+base_t/2+0.2])
                washer(d=base_w+add_d,t=top_h,d_pin=base_w-10,$fn=fascets); // top cover
            screw_holes();
        }
    }

    module screw_holes () {
        // top cover screw holes
        rotate([0,0,90]) Rotation_Pattern(number=5,radius=base_w/1.8,total_angle=360)
                cylinder(h=base_t*3,d=2.5,center=true,$fn=12);
    }
        
}

*difference () { // DIFFERENCE FOR VIEWING SECTION CUT
    base_assy(T_angle = 0,draw_arm=true);
    
    translate([-50,0,-50]) cube([100,100,100],center=false); // SECTION CUT
}

module base_assy(T_angle=0) {
    // Fixed models
    color("blue") base_model ();
    color("lightblue") base_model (part_one=false);
    translate([0,0,-9]) P090S_pot(negative=false);

    // Moving turntable models
    rotate([0,0,T_angle]) {
        base_turntable_model ();
        translate([-5,0,A_joint_z_shift+base_t/2]) 
            rotate([0,90,0]) P090S_pot(negative=false);
    }
}
module draw_dummy_arm(a=[0,0,0],b=[0,0,100],c=[100,0,100],d=[100,0,0]) {
    color("silver") pt_pt_cylinder (from=a,to=b, d = 2,$fn=12);
    color("grey") pt_pt_cylinder (from=b,to=c, d = 2,$fn=12);
    color("black") pt_pt_cylinder (from=c,to=d, d = 2,$fn=12);
}
function inverse_arm_kinematics (c=[0,10,0]) = 
    // calculate the angles from pt C ***Inverse Kinematics***
    //  ASSUMES THAT c is on the YZ plane (x is ignored)
    // returns an array with [A_angle,B_angle]
    let (vt = norm(c))  // vector length from A to C
    let (sub_angle1 = atan2(c[2],c[1]))  // atan2 (Y,X)!
    let (sub_angle2 = acos((vt*vt+lenAB*lenAB-(lenBC*lenBC))/(2*vt*lenAB)) )
    echo(vt=vt,sub_angle1=sub_angle1,sub_angle2=sub_angle2)
    [sub_angle1 + sub_angle2,acos((lenBC*lenBC+lenAB*lenAB-vt*vt)/(2*lenBC*lenAB))] ;

module draw_assy (A_angle=0,B_angle=0,D_angle=0,T_angle = 0) {
    // Display the input arm assembly.
    //  This module is not for printing.
    // Input parameters are the angle from the input arm
    //  A and B are the angles of the potentiometers
    // The C angle is calculated so that the claw is vertical pointing down
    //  D is the claw angle
    //  T is the turntable angle
    // A larger wireframe robot arm is drawn with the computed angles
    Arob = A_angle;  // A robot arm = A input arm
    Brob = Arob+B_angle; // B robot arm = A + B input arm
    Crob = (Brob > 45) ? Brob - 135 : -90; // limit the claw from contacting the arm
    echo(Brob=Brob,Crob=Crob);
    
    a=[20,0,13]; // location of A

    // calculate b and c positions from angles
    br=[0,lenAB*cos(Arob),lenAB*sin(Arob)];  // B relative location
    b=a+br; // B absolute
    cr = [0,cos(Brob)*lenBC,sin(Brob)*lenBC];
    c=b+cr;  // C absolute
    
    angles = (c[2] > 0) ? [Arob,Brob] : inverse_arm_kinematics([0,c[1],0]); 

    base_assy(T_angle = T_angle);
    
    translate([0,0,13]) // translate to base joint location
        rotate([A_angle,0,T_angle]) // A rotation
            rotate([90,0,90]) 
                // Draw the AB arm assembly
                AB_arm_assy(B_angle);
                
    // calculate NEW b and c positions from angles
    br2=[0,lenAB*cos(angles[0]),lenAB*sin(angles[0])];  // B relative location
    b2=a+br2; // B absolute
    cr2 = [0,cos(angles[1])*lenBC,sin(angles[1])*lenBC];
    c2=b2+cr2;  // C absolute
    cdLen = 20;  // TEMPORARY DEFINE CLAW LENGTH
    dr=[0,cos(Crob)*cdLen,sin(Crob)*cdLen];
    d=c2+dr; // D absolute
   
    rotate([0,0,T_angle]) draw_dummy_arm(a,b2,c2,d);
}
draw_assy(A_angle=AA,B_angle=BB,T_angle=0);
    
*translate([30,0,40]) rotate([0,90,0]) ruler(100);
