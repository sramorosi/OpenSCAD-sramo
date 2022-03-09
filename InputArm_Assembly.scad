// Input Arm Assembly
//  Design for Human Hand to drive a Robot Arm
//  last modified February 2022 by SrAmo
//use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
include <Part-Constants.scad>

// Joint A angle
AA = 5; // [0:130.0]
// Joint B angle
BB = -10; // [-145:1:0.0]
// Turntable angle
TT = 0; // [-60:60]
// use 140 for printing, 40 for display
fascets = 140; // [40,140]

// Draw the Input Arm Assembly?
display_assy = true;
// Section cut Assy at X = 0?
clip_yz = false;
// Section cut Assy at Z = 0?
clip_xy = false;

// length of A-B arm, color = plum
lenAB=50;     // mm
// length of B-C arm, color = blue
lenBC=60;      // mm
widthAB = 15; // mm

// A joint shift in Z direction
A_Z_shift = 13; // mm
A_joint_z_shift = A_Z_shift-2;
base_t = 6;
base_w = 60;

// CLAW LENGTH on Real arm, for dummy model
cdLen = 20;  

if (display_assy) {
    difference () {
        draw_assy(AA,BB,TT);
        if (clip_yz) // x = xcp cut 
            translate ([-201,-100,-100]) cube (200,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-100,-100,-200]) cube (200,center=false);
        }
    }
module pot_joint(pot=true,lug_two = true,tlug = 8) {
    // If pot = true then model the side that holds the pot
    // Else model the lug that goes on the shaft
    dbody = 26;
    zbody = A_Z_shift;
    dlug = 18;
    difference () {
       // lug
        union() {
            if (pot) {
                translate([0,0,-zbody/2+1]) washer(d=dbody,t=zbody+2,d_pin=1,$fn=fascets);
                translate([0,0,2.4]) washer(d=dbody-10,t=5,d_pin=1,$fn=fascets);
                
                // lug opposite pot.  Large hole for access
                if (lug_two) translate([0,0,5+4+tlug]) washer(d=dbody,t=8,d_pin=7,$fn=fascets); 
            } else { // lug for shaft
                translate([0,0,5+3.8]) washer(d=dlug,t=tlug+0.4,d_pin=1,$fn=fascets);
            }
        }
        // remove potentiometer interfaces
        translate([0,0,0.01]) rotate([0,0,-90]) P090S_pot(negative=true);
        // screw holes
        rotate([0,0,90]) Rotation_Pattern(number=2,radius=dbody/2.5,total_angle=360)
                cylinder(h=27,d=2.5,center=true,$fn=fascets);

    }
}
*pot_joint();
*translate([30,0,0]) {
    pot_joint(pot=false);
    P090S_pot(negative=false);
}
module Pot_Cover_model() {
    dbody = 26;
    difference() {
        washer(d=dbody,t=3,d_pin=1,$fn=fascets);
        // screw holes
        rotate([0,0,90]) Rotation_Pattern(number=2,radius=dbody/2.5,total_angle=360)
                cylinder(h=6,d=3,center=true,$fn=fascets);
    }
    translate([18,0,0]) cube([16,16,3],center=true);
    translate([27,0,8.5]) cube([3,16,20],center=true);

}
*Pot_Cover_model();

module C_End_Knob_model() {
    pot_joint(pot=false,lug_two=false,tlug=7,$fn=fascets);
    // finger point rounded_cube(size=[x,y,z],r=rad,center=true)
    translate([12,0,9]) rounded_cube([19,6,7],r=2.9,center=true,$fn=fascets);
}
*C_End_Knob_model();

module Input_Arm_model(len=100,width=10,overtravel=false) {
    difference() {
        union() {
            // Preview F5 display problems, but Render F6 works
            rotate([0,0,90]) translate([-width/2,-len+10,-26])  
                linear_extrude(height=26) 
                    U_section(Lbase=width,Lleg=15,Tbase=3,Tleg=3);   
            translate([0,0,-5]) pot_joint(pot=false);
            translate([len,0,-13]) pot_joint(pot=true,lug_two = true);
        }
        // remove channel for wire
        translate([len-10,0,-25]) cube([10,6,6],center=true);
        // remove notch for mating arm
        translate([len,0,-4]) rotate([0,0,45]) 
        translate([0,4,0]) cube([60,8,8],center=true);
        
    }
    // Cube that connects the two ends
    translate([5,-width/2,0]) cube([len-10,width,8],center=false);
    // overtravel stop
    if (overtravel) {
        translate([-15,width/2,0]) cube([30,width/2,8],center=false);
    }
}
*Input_Arm_model(len=lenAB,width=widthAB,overtravel=false);
*Input_Arm_model(len=lenBC,width=widthAB,overtravel=true);

module Input_Arm_Assembly(B_angle = 0,C_angle=0){
    // Display the Input Arm Assembly from the AB arm and on
    // A joint is at [0,0], Second joint is at [length,0]
    
    // DRAW THE AB ARM
    color("plum",1) Input_Arm_model(len=lenAB,width=widthAB);
    color("plum",.5) translate([lenAB,0,-28]) rotate([0,0,180]) Pot_Cover_model();
    
    // DRAW THE BC ARM 
    translate([lenAB,0,-8]) rotate([0,0,B_angle]) {
        color("lightblue",1) Input_Arm_model(len=lenBC,width=widthAB,overtravel=true);
        color("lightblue",.5) translate([lenBC,0,-28]) rotate([0,0,180]) Pot_Cover_model();
        translate([0,0,-5]) P090S_pot(negative=false);
        
        // DRAW THE C END
        translate([lenBC,0,-13]) rotate([0,0,C_angle]) {
            color("blue",1) C_End_Knob_model();
            P090S_pot(negative=false);
            }
        }
}
*Input_Arm_Assembly();

module base_turntable_model () {
    // Model of turntable Base, for the input arm

    // Potentiometer support for Joint A
    translate([-5,0,A_joint_z_shift+base_t/2]) rotate([0,-90,180]) pot_joint();

    difference() {
        cylinder(h=base_t,d=base_w,center=true,$fn=fascets); // turntable
        
        // remove turntable T potentiometer interface
        translate([0,0,-7]) P090S_pot(negative=true);
        
        // Wire access holes
        rotate([0,0,90]) Rotation_Pattern(number=2,radius=base_w/4,total_angle=360)
                cylinder(h=base_t*3,d=12,center=true,$fn=fascets);
    }
}
*base_turntable_model();

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
            
            screw_holes(dia=2.5,height=10); // subtract top cap screw holes
            // subtract a big hole for the wires
            translate([0,-30,-8]) rotate([90,0,0]) cylinder(h=46,r=14,center=true,$fn=60);
        // bottom attach screw holes
        translate([0,0,-20]) Rotation_Pattern(number=4,radius=20,total_angle=360)
                cylinder(h=base_t*3,d=3,center=true,$fn=12);
            }
            
    } else { // PART TWO = top layer
        difference() {  // top cover
            translate([0,0,top_h/2+base_t/2+0.2])
                washer(d=base_w+add_d,t=top_h,d_pin=base_w-10,$fn=fascets); 
            screw_holes(dia=3,height=top_h*4);
            translate([0,0,top_h*2]) screw_holes(dia=5.8,height=top_h);
        }
    }

    module screw_holes (dia=2.5,height=4) {
        // top cover screw holes
        rotate([0,0,90]) Rotation_Pattern(number=5,radius=base_w/1.8,total_angle=360)
                cylinder(h=height,d=dia,center=true,$fn=12);
    }
        
}
*base_model(part_one=true);
*base_model(part_one=false);

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
            rotate([-90,0,-90]) P090S_pot(negative=false);
    }
}

module draw_assy (A_angle=0,B_angle=0,T_angle = 0) {
    // Display the input arm assembly.
    //  This module is not for printing.
    // Input parameters are the angle from the input arm
    //  A and B are the angles of the potentiometers
    // The C angle is calculated so that the claw is vertical pointing down
    //  D is the claw angle
    //  T is the turntable angle
    // A larger wireframe robot arm is drawn with the computed angles
    Arob = A_angle;  // A robot arm = A input arm
    Brob = (Arob+B_angle < -45) ? -45 : Arob+B_angle; // B robot arm = A + B input arm
    Crob = (Brob > 45) ? Brob - 135 : -90; // limit the claw from contacting the arm
    echo(Brob=Brob,Crob=Crob);
    
    a=[20,0,A_Z_shift]; // location of A

    // calculate b and c positions from angles
    br=[0,lenAB*cos(Arob),lenAB*sin(Arob)];  // B relative location
    b=a+br; // B absolute
    cr = [0,cos(Brob)*lenBC,sin(Brob)*lenBC];
    c=b+cr;  // C absolute
    
    // IF C IS TOO LOW, MODIFY KINEMATICS TO PREVENT ARM FROM RUNNING INTO THE GROUND
    angles = (c[2] > A_Z_shift) ? [Arob,Brob] : inverse_arm_kinematics([0,c[1],0],lenAB=lenAB,lenBC=lenBC); 

    base_assy(T_angle = T_angle);
    
    translate([0,0,A_Z_shift]) // translate to base joint location
        rotate([A_angle,0,T_angle]) // A rotation
            rotate([90,0,90]) 
                // Draw the AB arm assembly
                Input_Arm_Assembly(B_angle,C_angle=Crob);
                
    // calculate NEW b and c positions from angles
    br2=[0,lenAB*cos(angles[0]),lenAB*sin(angles[0])];  // B relative location
    b2=a+br2; // B absolute
    ba2 = (c[2]>A_Z_shift) ? Brob : -(180-angles[0]-angles[1]);  // Angle of BC arm relative to horizontal
    cr2 = [0,cos(ba2)*lenBC,sin(ba2)*lenBC];
    c2=b2+cr2;  // C absolute
    dr=[0,cos(Crob)*cdLen,sin(Crob)*cdLen];
    d=c2+dr; // D absolute
   
    rotate([0,0,T_angle]) draw_dummy_arm(a,b2,c2,d);
}
*draw_assy(A_angle=AA,B_angle=BB,T_angle=TT);
    
*translate([30,0,0]) rotate([0,90,90]) ruler(100);
