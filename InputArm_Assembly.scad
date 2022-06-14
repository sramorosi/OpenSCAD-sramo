// Input Arm Assembly
//  Design for Human Hand to drive a Robot Arm
//  last modified June 2022 by SrAmo
//
//  To make printable models find "FOR_PRINT" and remove * suffix, 
//     then F6 & Export .stl

use <Robot_Arm_Parts_lib.scad>

// Joint A angle
AA = 5; // [0:170.0]
// Joint B angle
BB = -10; // [-170:1:0.0]
// Turntable angle
TT = 0; // [-90:90]
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
lenBC=70;      // mm
widthAB = 15; // mm

// A joint shift in Z direction
Z_shift = 13; // mm
A_joint_y = 10;
A_joint_lateral = 6.5;

base_t = 6;
base_w = 60;

// CLAW LENGTH on Real arm, for dummy model
cdLen = 20;  

// Potentiometer support dimensions
// Outer Body Diameter
dbody = 24; 
// Inner Lug Diameter
dlug = 13.5;
// Inner Lug Thickness
tlug = 8;

// Angle for add-on pot lug screws
ang_add = 64;

/*################### PART LIB FUNCTIONS AND MODULES COPY, FOR THINGIVERSE MAKE 2

//######################################################## */
module pot_joint(lug_two = true) {
    // Model of Potentiometer holder
    // 
    difference () {
        union() {
            translate([0,0,-1.25]) washer(d=dbody,t=tlug+0.5,d_pin=1,$fn=fascets);
            // lug opposite pot.  Large hole for access
            if (lug_two) translate([0,0,5.3+tlug]) washer(d=dbody,t=4,d_pin=6.3,$fn=fascets); 
        }
        // remove potentiometer interfaces
        P090L_pot(negative=true);
        // screw holes
        Rotation_Pattern(number=2,radius=dbody/2.7,total_angle=360)
                cylinder(h=50,d=2.5,center=true,$fn=fascets);
    }
}
module lug_joint() {
    // lug that goes on the shaft
    difference () {
        translate([0,0,7.2]) washer(d=dlug,t=tlug,d_pin=1,$fn=fascets);
        // remove potentiometer interfaces
        translate([0,0,2]) scale([1.02,1.02,1.02]) P090L_pot(negative=true);
    }
}
module Pot_Cover_model() {
    color("Cyan",.5) translate([0,0,-7]) {
        difference() {
            washer(d=dbody,t=3,d_pin=2,$fn=fascets);
            // screw holes
            Rotation_Pattern(number=2,radius=dbody/2.7,total_angle=360)
                    cylinder(h=6,d=3,center=true,$fn=fascets);
            // two bumps on backside
            translate([0,-8.5/2,1]) cylinder(h=6,d=2.4,center=true,$fn=20);
            translate([0,8.5/2,1]) cylinder(h=6,d=2.4,center=true,$fn=20);
        }
        translate([0,-16,0]) cube([16,16,3],center=true);
        translate([0,-25,5.5]) cube([16,3,14],center=true);
    }
}
*Pot_Cover_model(); // FOR_PRINT

*difference() { // Used for visulization of Pot Joints
    union() {
        pot_joint(); // not for print
        lug_joint(); // not for print
        P090L_pot(negative=false); // not for print
        Pot_Cover_model(); // not for print
   }
    translate([-20,0,0]) cube(40,center=true);  // Section Cut cube
}

module C_knob_base () {
    pot_joint(lug_two = false);
    translate([0,dbody/2,-Z_shift/2+2]) rotate([90,0,0]) {
        difference() {
            cube([50,Z_shift+2,4],center=true);
            // screw holes
            Rotation_Pattern(number=2,radius=20,total_angle=360)
                    cylinder(h=10,d=3,center=true,$fn=fascets);
        }
    }
}
*C_knob_base(); // FOR_PRINT

module C_End_Knob_model(notch_rotation) {
    color("DeepPink") {
        rotate([0,0,notch_rotation]) lug_joint($fn=fascets);
        // finger point rounded_cube(size=[x,y,z],r=rad,center=true)
        // center of end = 19/2 + 13 -3 = 19.5
        translate([13,0,7.2]) rounded_cube([19,6,tlug],r=3,center=true,$fn=fascets);
        // Add a knurl
        translate([19.5,0,7.2]) 
            Rotation_Pattern(number=12,radius=3,total_angle=360)
                    cylinder(h=tlug,d=1,center=true,$fn=fascets);
    }
}
*C_End_Knob_model(notch_rotation=-90);  // FOR_PRINT

module AB_Arm_model(len=100,width=10) {
    difference () {
        union() {
            rotate([0,0,180]) lug_joint(); // lug
            translate([len,0,-4.1]) rotate([0,0,-90]) pot_joint(lug_two = true);
            translate([len-dbody/2-3,-dlug/2,-tlug/2]) cube([8,dlug,tlug*1.6],center=false);
            difference () {
                translate([0,-dlug/2,3.2]) cube([len-8,dlug,tlug],center=false);
                cylinder(h=40,d=8,center=true);
            }
        }
        // remove hole for wire
        translate([len-20,0,10]) cylinder(h=20,r=3.5,center=true,$fn=fascets);
    }
}
// AB_model
*AB_Arm_model(len=lenAB,width=widthAB);  // FOR_PRINT

module BC_Arm_model(len=100,width=10) {
    difference() {
        union() {
            lug_joint(); // lug
            difference() {
                translate([-dlug/2,0,3.2]) cube([dlug,dlug,tlug],center=false);
                cylinder(h=40,d=8,center=true);
            }
            // Cubes that connect the two ends
            translate([-20,width/4+4,3.2]) cube([50,width-4,tlug],center=false);
            translate([30,width/4,3.2]) cube([len-38,width,tlug],center=false);
            translate([len-dbody/2-3,width/4,-tlug/2]) cube([8,width,tlug*1.6],center=false);
            
            translate([len,width*3/4,-tlug/2-.1]) 
                rotate([0,0,-90]) pot_joint(lug_two = true);
        }
        // remove hole for wire
        translate([len-20,width*3/4,10]) cylinder(h=20,r=3.5,center=true,$fn=fascets);
        // screw holes to hold add-on to BC arm
        translate([len,width*3/4,10])
            rotate([0,0,180-ang_add/4]) 
                Rotation_Pattern(number=2,radius=dbody/2.5,total_angle=ang_add)
                    cylinder(h=5,d=2.5,center=true,$fn=fascets);

    }
}
*BC_Arm_model(len=lenBC,width=widthAB); // FOR PRINT 

module add_on_pot_joint() {
    color("Peru") {
        // Model of Potentiometer holder
        // 
        difference () {
            union() {
                translate([0,0,2.9]) washer(d=dbody,t=tlug*2+0.5,d_pin=1,$fn=fascets);
            }
            // remove potentiometer interfaces
            P090L_pot(negative=true);
            // screw holes for cover
            Rotation_Pattern(number=2,radius=dbody/2.7,total_angle=360)
                    cylinder(h=50,d=2.5,center=true,$fn=fascets);
            // screw holes to hold add-on to BC arm
            rotate([0,0,-90-ang_add/4]) 
                Rotation_Pattern(number=2,radius=dbody/2.5,total_angle=ang_add)
                    cylinder(h=50,d=2.5,center=true,$fn=fascets);
            // remove the area for the knob to move
            translate([0,dbody/2.2,tlug-1]) 
                rotate([0,0,45]) 
                    cube([dbody,dbody,tlug+0.5],center=true);
            translate([0,0,tlug-.75]) 
                cylinder(h=tlug+1,d=dlug*1.05,center=true,$fn=fascets);
        }
    }
}
*add_on_pot_joint();  // FOR PRINT
*rotate([0,0,90]) C_End_Knob_model(notch_rotation=-90);

// BC Assembly
module BC_Assy(C_angle=0) {
    // DRAW THE BC ARM 
    color("lightblue",1) BC_Arm_model(len=lenBC,width=widthAB);
    translate([lenBC,widthAB*3/4,-4]) rotate([0,0,-90])
        Pot_Cover_model();
    
    // DRAW THE C END
    translate([lenBC,widthAB*3/4,-4]) {
        rotate([0,0,C_angle]) 
            color("blue",1) C_End_Knob_model(notch_rotation=-90);
    
        rotate([0,0,-90]) P090L_pot(negative=false);
        // SECOND, ADD ON KNOB
        translate([0,0,12]) color("green",1) C_End_Knob_model(notch_rotation=-90);
        
        translate([0,0,26.5]) rotate([180,0,90]) {
            P090L_pot(negative=false);
            add_on_pot_joint();
            Pot_Cover_model();

        }
    }
}
*BC_Assy(90);  // Not for print

module Input_Arm_Assembly(B_angle = 0,C_angle=0){
    // Display the Input Arm Assembly from the AB arm and on
    // A joint is at [0,0], Second joint is at [length,0]
    
    // DRAW THE AB ARM
    color("plum",1) rotate([180,0,0]) AB_Arm_model(len=lenAB,width=widthAB);
    translate([lenAB,0,4]) rotate([180,0,90]) Pot_Cover_model();
    translate([lenAB,0,4]) rotate([180,0,90]) P090L_pot(negative=false);
    
    translate([lenAB,0,-10]) rotate([0,0,B_angle]) BC_Assy(C_angle);
}
//translate([A_joint_lateral,A_joint_y,Z_shift+base_t/2]) // translate to A joint location
    //rotate([A_angle,0,0]) // A rotation
        //rotate([90,0,90]) 
            *Input_Arm_Assembly(-170,90);  // do not print


module base_turntable_model () {
    // Model of turntable Base, for the input arm

    // Potentiometer support for Joint A
    translate([-8,A_joint_y,Z_shift+base_t/2]) rotate([0,-90,180]) rotate([0,0,180]) pot_joint(lug_two = true);
    translate([-1,A_joint_y,base_t/2+2]) cube([24,18,4],center=true);

    difference() {
        cylinder(h=base_t,d=base_w,center=true,$fn=fascets); // turntable
        
        // remove turntable T potentiometer interface
        translate([0,0,-7]) P090L_pot(negative=true);
        
        // Wire access hole
        translate([0,-14,0]) cylinder(h=base_t*3,d=16,center=true,$fn=fascets);
    }
}
*base_turntable_model(); // FOR_PRINT

module base_model (part_one = true) {
    // Model of Base, for the input arm (THE FIXED PART)
    // TWO PART MODEL. 1 = MAIN BASE,  2 = SCREW ON TOP
    
    bottom_h = 16;
    top_h = 4;
    add_d = 12;
    
    if (part_one) {
        difference() { 
            union () { // PART 1
                // Potentiometer support for Joint T
                translate([0,0,-bottom_h+6]) pot_joint(lug_two = false);
            
                translate([0,0,-bottom_h/2-base_t/2])
                    washer(d=base_w+add_d,t=bottom_h,d_pin=base_w-10,$fn=fascets); // bottom layer
                
                translate([0,0,-bottom_h])
                    washer(d=base_w-8,t=6,d_pin=dbody-4,$fn=fascets); // pot support
            
                // mid layer, outside of turn table
                washer(d=base_w+add_d,t=base_t+0.4,d_pin=base_w*1.01,$fn=fascets); 
            } // END UNION
            
            screw_holes(dia=2.5,height=10); // subtract top cap screw holes
            // subtract a big hole for the wires
            translate([-4,-30,-6]) rotate([90,0,0]) cylinder(h=46,r=12,center=true,$fn=60);
        // bottom attach screw holes
        translate([0,0,-20]) rotate([0,0,30]) Rotation_Pattern(number=4,radius=20,total_angle=360)
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
// bottom
*base_model(part_one=true);  // FOR_PRINT
// top
*base_model(part_one=false); // FOR_PRINT

module base_assy(T_angle=0) {
    // Fixed models
    color("blue") base_model (part_one=true);
    color("lightblue") base_model (part_one=false);
    translate([0,0,-9]) P090L_pot(negative=false);

    // Moving turntable models
    rotate([0,0,T_angle]) {
        base_turntable_model ();
        translate([-8,10,Z_shift+base_t/2]) 
            rotate([-90,90,-90]) P090L_pot(negative=false);
    }
}
*base_assy(T_angle = -90);  // not for print

*difference () { // DIFFERENCE FOR VIEWING SECTION CUT
    base_assy(T_angle = 90);
    translate([-50,0,-50]) cube([100,100,100],center=false); // SECTION CUT
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
    
    a=[20,0,Z_shift]; // location of A

    // calculate b and c positions from angles
    br=[0,lenAB*cos(Arob),lenAB*sin(Arob)];  // B relative location
    b=a+br; // B absolute
    cr = [0,cos(Brob)*lenBC,sin(Brob)*lenBC];
    c=b+cr;  // C absolute
    
    // IF C IS TOO LOW, MODIFY KINEMATICS TO PREVENT ARM FROM RUNNING INTO THE GROUND
    angles = (c[2] > Z_shift) ? [Arob,Brob] : inverse_arm_kinematics([0,c[1],0],lenAB=lenAB,lenBC=lenBC); 

    base_assy(T_angle = T_angle);
    
    rotate([0,0,T_angle]) // T rotation
        translate([A_joint_lateral,A_joint_y,Z_shift+base_t/2]) // translate to A joint location
            rotate([A_angle,0,0]) // A rotation
                rotate([90,0,90]) 
                // Draw the AB arm assembly
                    Input_Arm_Assembly(B_angle,C_angle=Crob);
                
    // calculate NEW b and c positions from angles
    br2=[0,lenAB*cos(angles[0]),lenAB*sin(angles[0])];  // B relative location
    b2=a+br2; // B absolute
    ba2 = (c[2]>Z_shift) ? Brob : -(180-angles[0]-angles[1]);  // Angle of BC arm relative to horizontal
    cr2 = [0,cos(ba2)*lenBC,sin(ba2)*lenBC];
    c2=b2+cr2;  // C absolute
    dr=[0,cos(Crob)*cdLen,sin(Crob)*cdLen];
    d=c2+dr; // D absolute
   
    *rotate([0,0,T_angle]) draw_dummy_arm(a,b2,c2,d);
}

*draw_assy(A_angle=160,B_angle=-165,T_angle=TT);

if (display_assy) {
    difference () {
        draw_assy(AA,BB,TT);
        if (clip_yz) // x = xcp cut 
            translate ([-201,-100,-100]) cube (200,center=false);
        if (clip_xy) // z = 0 cut 
            translate ([-100,-100,-200]) cube (200,center=false);
    }
    *translate([0,-100,0]) rotate([-90,0,90]) {
        C_End_Knob_model(notch_rotation=-90);
        C_knob_base();
        P090L_pot(negative=false);
    }
}    
*translate([30,0,0]) rotate([0,90,90]) ruler(100);
