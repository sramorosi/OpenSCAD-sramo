//  SAMC Make 3
//  Servo Actuated Motion Control
//
//  By SrAmo,  December 2022
//
// To Do:
// Angles, lengths diagram (with Thingiverse publish)
//  Make tube drill templates
// add holes to tubes (for print or routing)
// 
use <force_lib.scad> // contains forces, springs, MS modules and functions
include <Part-Constants.scad>
use <Robot_Arm_Parts_lib.scad>
use <gears_involute.scad>  // Modified version of spur gears by Greg Frost
use <arduino.scad>

// use 140 for printing, 40 for display
FACETS = 40; // [40,140]

// Number of position step in internal calculation
steps = 40; // [2:1:200]
// Joint A angle
AA = 135; // [0:1:175]
// Joint B angle
BB = -100; // [-175:1:0]
// Joint C angle
CC = 10; // [-145:1:145]
// Joint D angle
DD = 10; // [-145:1:145]
// Joint CLAW angle
//CLAW = 20; // [-145:1:145]
// Turntable angle
TT = 0; // [-80:80]

// length of A-B arm (mm)
LEN_AB=350; 
// length of B-C arm (mm)
LEN_BC=380; 
// C to grip/load point (mm)
LEN_CD = 160;  // to become s_CM_x and s_CM_y
// Offset of Joint A from the Turntable axis
SHIFT_TA = 0; // (mm)
// Height of A from base
A_HEIGHT = 50;

//  Using 1 inch square tube for the arms
wTube = 1/mm_inch;  // mm
twall = 0.0625/mm_inch;   // mm
// Space between arm to help movement
armSpace = 3; // mm

max_range = LEN_AB+LEN_BC;

/* DEFINE WEIGHTS. Reference weights:
  A 12 oz can of pop/beer is 375 grams (i.e. payload goal)
  A 40x20x40 mm servo is about 70 grams
  gopro = 120 gram
  claw assembly = 150 (grams)
  smartphone = 160 gram   */
AL_DENSITY = 0.002770;  // aluminum gram/mm^3
SERVO_MASS = 90;    // Servo plus mounting structure (grams)

// Payload weight on point D
PAYLOAD_MASS=200; // Maximum payload mass (thing being lifted) (gram)
CD_MASS=200;  // weight of CD arm (servos, structure) (gram)
CM_CD = LEN_CD/2; // center-of-mass of CD, from C (mm)

echo(LEN_AB=LEN_AB,LEN_BC=LEN_BC,LEN_CD=LEN_CD,PAYLOAD_MASS=PAYLOAD_MASS);

// weight of BC arm
BC_tube_weight = AL_DENSITY*LEN_BC*(4*wTube*twall);
BCweight = BC_tube_weight + SERVO_MASS; // (grams)
CM_BC = LEN_BC*0.6; // location of BC arm center-of-mass from B (mm)

// weight of AB arm
AB_tube_weight = AL_DENSITY*LEN_AB*(4*wTube*twall);
ABweight = AB_tube_weight + 2*SERVO_MASS; // (grams)
CM_AB = LEN_AB*0.5; // location of AB arm center-or-mass from A (mm)

echo(BCweight=BCweight,ABweight=ABweight," grams");

// torsion spring constants for A joint
A_K = -2433; // g-mm/deg,   
            // 600 is for spring 9271K619   ~589
             // 1000 is for spring 9271K145
             // 2688 is SPEC for spring 9271K589
             // 2433 MEASURED FOR spring 9271K589 (94%)
             // Measured in a Make 3 robot arm joint.  
A_theta_zero = 90; // degrees, 90 is straight up

// Angle ratio (288/180) = 1.6   1.6*32 = 51
// With 360 deg servo:
// Angle ratio (360/180) = 2.0    2 * 32 = 64

// Large gear at Joint A and B, Printed
big_gear_teeth = 66; 
// Small gear at Joint A and B, Servo Mount Gear from ServoCity
small_gear_teeth = 32; 

// Maximum Motor Torque (gram-mm) 
Motor_Max_Torque = 250000; 
Geared_Max_Torque = Motor_Max_Torque * (big_gear_teeth/small_gear_teeth)*0.8;
echo("SERVO MOTOR CAPABILITY=",Motor_Max_Torque=Motor_Max_Torque," gram-mm");
echo("Big Gear teeth=",big_gear_teeth," Small Gear teeth =",small_gear_teeth);
echo("GEARED SERVO CAPABILITY=",Geared_Max_Torque=Geared_Max_Torque," gram-mm");

// USE LIST COMPREHENSIONS TO FILL ARRAYS
//  A, zero = horizontal, positive rotation up
//  B, zero = inline with AB, positive = same sense as A
//  C, zero = inline with BC, positive = same sense as A and B
//  T, positive = CCW looking down

//angles = [ for (a = [0 : steps-1]) get_angles_from_t(a/steps,0,180,0,-180)];
angles = [ for (a = [0 : steps-1]) sweep1(a/steps,0,160,0,-170,-90,90)];
//echo (angles=angles);
    
c = [ for (a = [0 : steps-1]) [get_CX(angles[a]),get_CY(angles[a]),0]];

d = [ for (a = [0 : steps-1]) [c[a][0]+LEN_CD*cos(angles[a][2]),c[a][1]+LEN_CD*sin(angles[a][2]),0]];
    
// COMPLIANT CLAW Length
claw_length = 150;
// COMPLIANT CLAW width
claw_width = 120;
// COMPLIANT CLAW height
claw_height = 25; 
// COMPLIANT CLAW curve radius
claw_radius = 18;
// COMPLIANT CLAW thickness
claw_t = 1.8;  
shim_t = 7;

module claw_servo_bracket() {
    servo_plate_t = 8;
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
            rotate([0,90,-90]) servo_body(vis=false,,$fn=FACETS);
        
        // remove the screw holes
        translate ([-servo_plate_l/2-4,0,0]) rotate([90,0,0]) 
            hole_pair (x = 0,y=claw_radius*1.7,d=hole_M3,h=100,$fn=FACETS);
        
        translate([-38,0,0]) rotate([90,0,0]) cylinder(h=100,d=hole_M3,center=true,$fn=FACETS);
    }
}
*claw_servo_bracket();// FOR_PRINT

module Claw(assy=true){
    // DRAW THE COMPLIANT CLAW
    back_plate_w = claw_width - 4*claw_radius;
    difference () {
        translate([claw_radius,-claw_height/2,0]) 
            rotate([0,-90,-90])     
                compliant_claw2 (len=claw_length,width=claw_width,t1=claw_t,t2=claw_height,r=claw_radius,pre_angle=15,$fn=FACETS);
        // remove attach pins (screw holes)
        translate ([claw_radius/2,0,0])
            hole_pair (x = 0,y=claw_height*0.7,d=hole_M3,h=100,$fn=FACETS);
        // remove the screw holes
        translate ([2*claw_radius,0,0]) rotate([90,0,0]) 
            hole_pair (x = 0,y=claw_radius*1.7,d=hole_M3,h=100,$fn=FACETS);
        translate([2*claw_radius-6,0,0]) rotate([90,0,0]) cylinder(h=100,d=hole_M3,center=true,$fn=FACETS);
    }
    if (assy) {
        claw_servo_x = 2*claw_radius+32;
        // Remove Servo Bracket for claw print
        color("green") translate([claw_servo_x,claw_height/2+shim_t,0]) claw_servo_bracket();
        
        // Remove Servo for claw print
        color ("red",.7) translate([claw_servo_x,svo_flange_d+shim_t,0]) 
            rotate([0,90,-90]) servo_body();
    }
}
*Claw(assy=false); //FOR_PRINT

module tube_model(t=1,wall=0.1,l=10) {
    color("grey") linear_extrude(height=l, convexity=10) difference() {
        square(t,center=false);
        translate([wall,wall,0]) square(t-2*wall,center=false);
    }
}
*tube_model(t=wTube,wall=twall,l=20);

module torsion_spring_spacer() {
    translate([0,0,18.5/2]) 
        washer(d=9271K589_ID-1,t=21,d_pin=hole_qtr_inch*1.03,$fn=FACETS);
}
*torsion_spring_spacer(); // EXPORT AS STL

module torsion_spring_make3() {
    torsion_spring (deflection_angle=9271K589_angle,OD=9271K589_OD,wire_d=9271K589_wd,leg_len=9271K589_len,coils=9271K589_coils,LH=9271K589_LH,inverse=false);
}
*torsion_spring_make3(); // not for print

module spring_combo() {
    translate([0,0,-9271K589_t-4]) {
        torsion_spring_spacer();
        torsion_spring_make3();
    }
}

module servo_mount(FT6335=true) {
    // servo mount is designed around servo with zero transformations.
    // servo center  is at x=0,y=0, Axis = z, Top of servo z = 0
    // x axis is long axis of servo
    
    xtotal = svo_l + 36;
    xshift = svo_l/2 - svo_shaft;
    y1 = 4;
    z1 = 25; // 1 inch = 25.4 mm.  
    svo_z_adjust = FT6335 ? 1 : 0; // Adjustment for the FT6335 servo
    
    difference () {
        translate([-xshift,y1/2+0.5,-z1/2])  // 0.5 mm in y if for preload
            cube([xtotal,svo_w+y1,z1],center=true);

        translate([0,0,svo_z_adjust]) servo_body (vis=false);
        translate([-xshift-1,0,-5])
            cube([svo_flange_l,svo_w,11],center=true);
        
        }
        translate([0,0,-12]) rotate([90,0,0]) {
            hole_pair (x = 22,y=15,d=3.3,h=40,$fn=FACETS);
            hole_pair (x = -43,y=15,d=3.3,h=40,$fn=FACETS);
        }
    }

*servo_mount(); // EXPORT AS STL
*color("red",0.5) servo_body();  // shoulder servo

module guss_profile(tube=wTube,gap=0.1,daxel=Qtr_bearing_od,dholes=3.5) {
    // 2D SHAPE.  Can write .svg
    // Profile of a gusset for tube of width tube
    r=tube/4;
    poly1 = [[0,0],[0,-1.5*tube-gap+r],[-r,-1.5*tube-gap],[-2*tube+r,-1.5*tube-gap],[-2*tube,-1.5*tube-gap+r],[-2*tube,-0.5*tube-gap-r],[0,tube/2]];
    difference () {
        hull() {
            polygon(poly1);
            circle(tube/1.7,$fn=FACETS);
        }
        circle(d=daxel,$fn=FACETS);
        translate([-tube,-tube-armSpace,0]) {
            rotate([0,0,90]) 
                hole_pair_2D (x = 0,y=1.6*tube,d=dholes); // attach holes
            circle(d=12,$fn=FACETS); // wire access hole
        }
    }
}
*guss_profile(tube=wTube,gap=armSpace,daxel=Qtr_bearing_od,dholes=3.5); // EXPORT AS SVG

*translate([-wTube,0,0]) { // BC ARM MACHINING. EXPORT AS SVG, rotate 90
    translate([-LEN_BC+wTube,0,0]) rotate([0,0,180]) svo2D(); 
    rotate([0,0,90]) hole_pair_2D (x = 0,y=1.6*wTube,d=3.5); // attach holes
    circle(d=12); // wire access hole
    // ADD MANUAL HOLE 0.67 INCH FROM y=0 IN EASLE FOR servo hub bore
}
*hole_pair_2D (x = 0,y=1.6*wTube,d=3.5); // TA MACHINING. EXPORT AS SVG, rotate 90
*circle(d=12); // wire access hole, TA MACHINING. EXPORT AS SVG, rotate 90

module plain_guss() {
    color("DeepSkyBlue") difference () {
        linear_extrude(6, convexity=10) 
            guss_profile(tube=wTube,gap=armSpace,daxel=Qtr_bearing_od,dholes=3.5);
        
        // use one or the other, depending on which side wanted
        //cube([200,wTube+2*armSpace,1],center=true);
        //cube([wTube,100,1],center=true);
        translate([0,0,6]) cube([200,wTube+2*armSpace,1],center=true);
        translate([wTube/2,0,6]) cube([wTube,100,1],center=true);
    }
}
*plain_guss(); // EXPORT AS STL

module big_gear_guss(teeth=10) {
    color("Aqua") union() {
        32P_Actobotics(teeth=big_gear_teeth,thickness=8,bore=Qtr_bearing_od);    
        linear_extrude(8, convexity=10) guss_profile(tube=wTube,gap=armSpace,daxel=Qtr_bearing_od,dholes=3.5);
    }
}
*big_gear_guss(teeth=big_gear_teeth); // EXPORT AS STL

module big_gear_lollypop(teeth=10) {
    // wTube
    lugT = 8; // mm
    gear_dia = (teeth)*(2.54/3.14159);
    color("Aqua") union() {
        translate([0,0,wTube/2 + 1])
            32P_Actobotics(teeth=teeth,thickness=8,bore=Qtr_bearing_od); 
       translate([0,0,-wTube/2 - lugT - 1])
            washer(d=gear_dia,t=lugT,d_pin=Qtr_bearing_od,center=false,$fn=FACETS);

        
        rotate([90,0,0]) translate([0,0,wTube/2]) 
        difference() {
            rotate([-90,0,0]) linear_extrude(wTube + 2*lugT + 2,convexity=20,center=true) 
                polygon([[wTube/4,0],[wTube/4,-wTube/4],[-wTube/4,-wTube/4],[-wTube*.8,0],[-wTube,0],[-wTube/2,-A_HEIGHT],[wTube/4,-A_HEIGHT],[wTube/4,0]]);
            //hex (size=wTube + 2*lugT + 2,l=2/mm_inch);
            hex (size=0.5/mm_inch,l=2.1/mm_inch); // hex shaft
        }
    }
}
*big_gear_lollypop(teeth=big_gear_teeth); // EXPORT AS STL

module geared_svo_block_assy(big_gear_teeth=60,small_gear_teeth=32,wbeam=10) {
    // Locate servo block assembly so that it interfaces with
    // big_gear_teeth located at 0,0,  on a beam of width wbeam,
    // using a small_gear_teeth on the servo
    
    // Distance between gears is the (total_teeth/2)*pitch/pi
    
    big_gear_rad = (big_gear_teeth)/2*(2.54/3.14159);
    small_gear_rad = (small_gear_teeth)/2*(2.54/3.14159);

    svoCtrLateral = wbeam/2 + svo_w/2; // offset in the y direction
    if (big_gear_rad+small_gear_rad < svoCtrLateral) {
        echo("********GEARS ARE TOO SMALL********");
    } else {
        svoAng = asin(svoCtrLateral/(big_gear_rad+small_gear_rad));
        //echo(svoCtrLateral=svoCtrLateral,svoAng=svoAng);
        svoCtrAxial=(big_gear_rad+small_gear_rad)*cos(svoAng);
    
        translate([-svoCtrAxial,svoCtrLateral,0]) {
            color("blue") servo_mount();
            color ("red",.5) servo_body();  // servo
            // turn off for thingiverse, purchased part
            32P_Actobotics(teeth=small_gear_teeth);   // servo gear 32 tooth
        }   
    }
}
*geared_svo_block_assy(big_gear_teeth=200,small_gear_teeth=small_gear_teeth,wbeam=wTube);

module claw_bracket(width=30,thk=25,len=60) {
    // COMPLIANT CLAW height
    claw_height = 25; 
    hole_space = claw_height*0.7;
    difference () {
    union() {
        svo_block_ring(thk=thk,$fn=FACETS);
        translate([20,width,(thk+5)/2]) ear();
        translate([20,-width,(thk+5)/2]) rotate([180,0,0]) ear();
    }
    // remove End attach pin
    translate ([len-claw_height,0,15]) rotate([90,90,0])
        hole_pair (x = 0,y=hole_space,d=3.5,h=100);
    }
    module ear() {
        difference() {
            cube([len,width,thk-5],center=true);
            translate([-20,-10,0]) cylinder(h=thk*2,d=24,center=true);
        }
    }
}
*claw_bracket(width=10);  // FOR PRINT

module DClaw_assy(t_D=0,assy=true){
    claw_end_w = 10; // claw interface width, mm
    if (assy) servo_block(angle=t_D);
    translate([0,0,28]) 
        rotate([0,0,t_D]) {
            color("blue",0.5) claw_bracket(width=claw_end_w);
            if (assy) translate([35,0,6]) rotate([90,-90,0]) Claw(assy=true);
        }
}
*DClaw_assy(t_D=0);

module CD_assy(t_C=0,t_D=0) {
    rotate([0,0,0]) {
        color ("red",.5) servo_body();  // servo
        servo_shim();
        servo_hub();
        // 1.31/2/mm_inch
        rotate([0,0,t_C+90]) {
            translate([0,5,43]) 
                rotate([90,0,0]) DClaw_assy(t_D=t_D);
        }
    }
}
*CD_assy(t_C=20,t_D=0);

module BC_arm_assy(armLen = 100,t_C=0,t_D=0){
    // fixed tube assy
    big_gear_guss(teeth=big_gear_teeth);
    translate([0,0,-wTube-2*armSpace]) plain_guss();

    // BC tube
    translate([0,-wTube-armSpace,0])  
        difference () {
            translate([0,-wTube/2,-wTube]) 
                rotate([0,-90,0]) 
                    tube_model(t=wTube,wall=twall,l=armLen+wTube);
            translate([-armLen,0,0]) {
                cylinder(h=3*wTube,d=12.64,center=true,$fn=FACETS); // servo block shaft bore
                translate([0,0,-30]) rotate([0,0,180]) linear_extrude(10,convexity=10) svo2D();
            }
        }
    translate([-armLen,-wTube-armSpace,-18]) rotate([0,0,180])
        CD_assy(t_C=t_C,t_D=t_D);
}
*BC_arm_assy(armLen=LEN_BC,t_C=0,t_D=45); // not for print

module AB_arm_assy(armLen = 100){
    // AB tube
    difference () {
        translate([wTube/2,-wTube/2,-wTube]) 
            rotate([0,-90,0]) 
                tube_model(t=wTube,wall=twall,l=armLen+wTube);
        cylinder(h=2*wTube,d=0.25/mm_inch,center=true,$fn=FACETS);
        translate([-armLen,0,0]) cylinder(h=2*wTube,d=0.25/mm_inch,center=true,$fn=FACETS);
    }
    
    geared_svo_block_assy(big_gear_teeth=big_gear_teeth,small_gear_teeth=small_gear_teeth,wbeam=wTube);
    
    translate([-armLen,0,0]) rotate([0,0,180]) 
        geared_svo_block_assy(big_gear_teeth=big_gear_teeth,small_gear_teeth=small_gear_teeth,wbeam=wTube);
    
    spring_combo();
}
*AB_arm_assy(armLen=LEN_AB); // not for print

module TA_assy() { // Assy between Turntable and joint A
    translate([0,0,0]) big_gear_lollypop(teeth=big_gear_teeth);
    
    length=4/mm_inch; // length of hex shaft
    translate([0,-wTube,0]) rotate([90,0,0])  
        color("RoyalBlue") hex (size=0.5/mm_inch,l=length);
}
*TA_assy();

function get_CX (a) = (cos(a[0])*LEN_AB+cos(a[0]+a[1])*LEN_BC);
function get_CY (a) = (sin(a[0])*LEN_AB+sin(a[0]+a[1])*LEN_BC);

function sweep1(t=0.0,lowA=0,highA=180,lowB=0,highB=180,lowC=-20,highC=20) =
(t<0.2) ? ([lowA,lowB,linear_interp(lowC,highC,t,0,0.2)]) : 
(t<0.5) ? ([linear_interp(lowA,highA,t,0.2,0.5),lowB,(lowC+highC)/2]) : 
(t<0.8) ? ([highA,linear_interp(lowB,highB,t,0.5,0.8),(lowC+highC)/2]) : 
          ([highA,highB,linear_interp(lowC,highC,t,0.8,1.0)]) ;

module CalculateMoments(display=false) {
    // Calculate joint Moments through range of motion in angles[]
    // Display (boolean) makes 3D column of moments magnitude (3D chart)
    // USE LIST COMPREHENSIONS TO FILL ARRAYS
    // MOMENT ON CD
    C_mom = [ for (a = [0 : steps-1]) (PAYLOAD_MASS*LEN_CD+CD_MASS*CM_CD)*cos(angles[a][0]+angles[a][1]+angles[a][2]) ];
    Margin_Safety2(C_mom,Motor_Max_Torque,"C moment");
        
    // MOMENT ON B, DUE TO CD AND BC
    B_trq = [ for (a = [0 : steps-1]) (BCweight*CM_BC+(PAYLOAD_MASS+CD_MASS)*LEN_BC)*cos(angles[a][0]+angles[a][1])+C_mom[a]];
    Margin_Safety2(B_trq,Geared_Max_Torque,"B SERVO - GREEN");
    *if (display) draw_3d_list(c,max_range/100,"green",B_trq/400); 
    *echo(B_trq=B_trq);
    
    B_trq_noload = [ for (a = [0 : steps-1]) (BCweight*CM_BC+(CD_MASS)*LEN_BC)*cos(angles[a][0]+angles[a][1])+CD_MASS*CM_CD*cos(angles[a][0]+angles[a][1]+angles[a][2])];

    A_trq_load_nospr = [ for (a = [0 : steps-1]) (ABweight*CM_AB+(BCweight+PAYLOAD_MASS+CD_MASS)*LEN_AB)*cos(angles[a][0])+ B_trq[a] ]; 
    Margin_Safety2(A_trq_load_nospr,Geared_Max_Torque,"A SERVO - NO SPRING - BLUE");
    *if (display) draw_3d_list(c,max_range/80,"blue",A_trq_load_nospr/400); 
    *echo(A_trq_load_nospr=A_trq_load_nospr);

    // The A spring helps the Joint A MOTOR
    // torsion spring
    A_spr_torque = [ for (a = [0 : steps-1]) torsion_spr_torque(A_K,angles[a][0],A_theta_zero) ];  
    A_spr_torque_min=min(A_spr_torque);
    A_spr_torque_max=max(A_spr_torque);
    echo (A_spr_torque_min=A_spr_torque_min,A_spr_torque_max=A_spr_torque_max);
    *echo(A_spr_torque=A_spr_torque);
    
    // calculate max A moment with full payload and spring
    A_trq_load_spr = [ for (a = [0 : steps-1]) A_trq_load_nospr[a] - A_spr_torque[a] ]; 
    Margin_Safety2(A_trq_load_spr,Geared_Max_Torque,"A SERVO - SPRING - BLUE");
    //echo(A_trq_load_spr=A_trq_load_spr);
    if (display) draw_3d_list(c,max_range/80,"blue",A_trq_load_spr/400); 
    
    // calculate max A moment with NO payload and spring (can be critical!)
    A_trq_noload_spr = [ for (a = [0 : steps-1]) (ABweight*CM_AB+(BCweight+CD_MASS)*LEN_AB)*cos(angles[a][0])+ B_trq_noload[a]  - A_spr_torque[a] ]; 
    Margin_Safety2(A_trq_noload_spr,Geared_Max_Torque,"A SERVO - SPRING - NO PAYLOAD - Yellow");
    if (display) draw_3d_list(c,max_range/70,"yellow",A_trq_noload_spr/400); 
    *echo(A_trq_noload_spr=A_trq_noload_spr);
        
    *rotate([-90,0,0]) for (a = [0 : steps-1]) 
        draw_assy(angles[a][0],angles[a][1],angles[a][2],0,LEN_AB,LEN_BC,LEN_CD);

}
CalculateMoments();

// Draw the steps, outside of force calculation module 
*for (a = [0 : steps-1]) draw_assy(angles[a][0],angles[a][1],angles[a][2],0,LEN_AB,LEN_BC,LEN_CD);
    
module zip_loop() {
    translate([0,0,-12]) rotate([90,0,0]) linear_extrude(4, convexity=10) 
        U_section(Lbase=14,Lleg=8,Tbase=4,Tleg=4);
}
*zip_loop();// not for print

module Electronics_Board (Assy=true) {
    board_l = 100;
    board_w = 90;
    board_t = 4;
    board_shift = 60;
    y_w = (board_w-20)/2;
    x_w = (board_l-20)/2;
    difference() {
        translate([board_shift,0,-2.1]) rounded_cube([board_l,board_w,board_t],r=10,center=true);
        translate([board_shift,33,0]) scale([1.01,1.01,1]) rotate([0,0,90]) Rocker_Switch();
        
        // subtract the 4 2X4 screw mounting holes
        translate([x_w+board_shift,y_w-20,0]) cylinder(h=board_t*3,d=3,center=true);
        translate([x_w+board_shift,-y_w,0]) cylinder(h=board_t*3,d=3,center=true);
        translate([-x_w+board_shift,-y_w,0])cylinder(h=board_t*3,d=3,center=true);
        translate([-x_w+board_shift,y_w-20,0]) cylinder(h=board_t*3,d=3,center=true);
        
        // arduino board holes (see arduino.scad)
        // off for thingiverse, purchased part
        translate([board_w,20,0]) rotate([180,0,-90]) holePlacement()
            union() {
                cylinder(d=4, h = board_t*3,$fn=FACETS);
              };
       *translate([125,40,-1]) rotate([0,0,-180]) linear_extrude(height = 1, convexity=10) 
           text("ARM MK 3", font = "Arial", size = 11);
    }
    translate([board_shift+20,40,0]) zip_loop(); // zip tie loop
    translate([board_shift-20,-40,0]) zip_loop();// zip tie loop

    if (Assy) {
        translate([board_shift,33,0]) rotate([0,0,90]) Rocker_Switch();
        // off for thingiverse, purchased part
        translate([board_w,20,-8]) rotate([180,0,-90]) arduino(); 
    }
}
*Electronics_Board(Assy=false);//FOR_PRINT

base_t = 0;   //  was 10 for old design

module new_base_assy() {

    // Representation of 2x4 wood
    LENGTH = 430;  // about 17 inches
    WIDTH = 300;   // about 12 inches
    WOOD_T = 3.5/mm_inch;  
    WOOD_W = 1.5/mm_inch;

    translate([0,0,-WOOD_W]) rotate([180,0,90]) 
        geared_svo_block_assy(big_gear_teeth=96,small_gear_teeth=48,wbeam=WOOD_T);
    
    color("Khaki") {
        translate([-WOOD_T/2,WIDTH/2-10,-base_t/2]) rotate([0,90,0]) cube([WOOD_T,WOOD_W,LENGTH]);
        translate([-WOOD_T/2,-WIDTH/2+10-WOOD_W,-base_t/2]) rotate([0,90,0]) cube([WOOD_T,WOOD_W,LENGTH]);
        // ROBOT ARM SUPPORT BEAM
        translate([-WOOD_T/2,-WIDTH/2-10,-base_t/2]) rotate([-90,0,0]) cube([WOOD_T,WOOD_W,WIDTH]);
        translate([LENGTH-WOOD_T/2,-WIDTH/2-10,-base_t/2]) rotate([0,90,90]) cube([WOOD_T,WOOD_W,WIDTH]);

    }
    Bearing_Flanged (t=Half_bearing_t, flange_t=Half_bearing_flange_t,od=Half_bearing_od,id=Half_bearing_id,flange_od=Half_bearing_flange_od);

    translate([0,0,-WOOD_W]) rotate([180,0,0]) Bearing_Flanged (t=Half_bearing_t, flange_t=Half_bearing_flange_t,od=Half_bearing_od,id=Half_bearing_id,flange_od=Half_bearing_flange_od);
    
    // Representation of electronics board
    translate([130,50,0]) rotate([0,0,-90]) Electronics_Board();
}
*new_base_assy(); // not for print

base_z_top = -21.5; // base offset down from zero, mm

module draw_assy (t_A=0,t_B=0,t_C=0,t_D=0,t_T=0) {
    // XZ = HORIZON
    // calculate b and c positions from angles
    //b=[LEN_AB*cos(t_A),LEN_AB*sin(t_A),0];  // B location
    //c = [(cos(t_A)*LEN_AB+cos(t_B)*LEN_BC),(sin(t_A)*LEN_AB+sin(t_B)*LEN_BC),0];

    // Draw Base and Shoulder assembly adjust z translation as required
    translate([0,0,base_z_top]) rotate([0,0,180]) new_base_assy();
    
    translate([SHIFT_TA,-wTube/2,A_HEIGHT-wTube/2+6])
        rotate([90,0,t_T]) {
            translate([0,0,-wTube/2]) TA_assy();
            rotate([0,0,t_A-180]) {
                AB_arm_assy(armLen=LEN_AB);
                // A joint .25 inch shaft,  LENGTH = 2.5 inch
                translate([0,0,-10]) 
                    cylinder(h=2.5/mm_inch,d=hole_qtr_inch,center=true);
                // Draw the BC link and End
                translate([-LEN_AB,0,0]) {
                    rotate([0,0,t_B]) 
                        rotate([0,0,0]) 
                            BC_arm_assy(armLen=LEN_BC,t_C=t_C,t_D=t_D);
                    // B joint .25 inch shaft,  LENGTH = 2.5 inch
                    translate([0,0,-10]) cylinder(h=2.5/mm_inch,d=hole_qtr_inch,center=true);
                }

            }
        }
} 
draw_assy (t_A=AA,t_B=BB,t_C=CC,t_D=DD,t_T=TT); // not for print