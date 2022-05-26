//  SAMC Make 3
//  Servo Actuated Motion Control
//
//  By SrAmo,  May 2022
//
// To Do:
// AB and BC arm model
// Torsion spring model
// Joint C model/profile for tube
// Servo block model
// Claw/camera holder model
// Base model
use <force_lib.scad> // contains forces, springs, MS modules and functions
include <Part-Constants.scad>
use <Robot_Arm_Parts_lib.scad>
use <BasicArm_Assembly.scad>
//use <Pulley-GT2_2.scad>
use <gears_involute.scad>  // off for thingiverse, purchased part

// Number of position step in internal calculation
steps = 20; // [2:1:200]
// length of A-B arm, mm
lenAB=300; 
// length of B-C arm, mm
lenBC=320; 
// End effector offsets from C to grip/load point. Moment arm.
lenCD = 160;

max_range = lenAB+lenBC;

/* DEFINE WEIGHTS
    Reference weights:
  A 12 oz can of pop/beer is 375 grams (i.e. payload goal)
  A 40x20x40 mm servo is about 70 grams
  gopro = 120 gram
  smartphone = 160 gram
*/
// Maximum payload weight (thing being lifted) (g)
// payload weight acts at point D
payload=200;  // 400 g goal (try to maximize this)
// weight of end effector with no payload (g)
// CD weight acts at point D
CDweight=200;  // weight of CD arm (servos, structure)
cgCD = 80; // mm

// weight of BC arm
BCweight = 232; // grams
cgBC = 215; // location of BC arm cg (mm)
// weight of AB arm
ABweight = 323; // grams
cgAB = 150; // location of BC arm cg (mm)

// Maximum Motor Torque (gram-mm) 
Motor_Max_Torque = 250000; 
echo("MOTOR CAPABILITY=",Motor_Max_Torque=Motor_Max_Torque," gram-mm");

// torsion spring constants for A joint
A_K = -2688; // g-mm/deg,   
            // 600 is for spring 9271K619   ~589
             // 1000 is for spring 9271K145
             // 2688 is for spring 9271K589
A_theta_zero = 90; // degrees, 90 is straight up

// Angle ratio (288/180) = 1.6   1.6*32 = 51
big_gear_teeth = 53;
small_gear_teeth = 32;

// the distance between gears is the teeth*pitch/pi
//gear_center_dist = (big_gear_teeth-small_gear_teeth)/2*(2.54/3.14159) - 0;
big_gear_rad = (big_gear_teeth)/2*(2.54/3.14159);
small_gear_rad = (small_gear_teeth)/2*(2.54/3.14159);
*echo(big_gear_rad=big_gear_rad);

//  Using 1 inch square tube for the arms
t1 = 25.4; // mm
twall = 0.0625*25.4;  // mm
// Space between arm to help movement
t2 = 2; // mm

// USE LIST COMPREHENSIONS TO FILL ARRAYS
//  A, zero = horizontal, positive rotation up
//  B, zero = inline with AB, positive = same sense as A
//  C, zero = inline with BC, positive = same sense as A and B
//  T, positive = CCW looking down

//angles = [ for (a = [0 : steps-1]) get_angles_from_t(a/steps,0,180,0,-180)];
angles = [ for (a = [0 : steps-1]) sweep1(a/steps,0,160,0,-180,0)];
//echo (angles=angles);
    
c = [ for (a = [0 : steps-1]) [get_CX(angles[a]),get_CY(angles[a]),0]];

d = [ for (a = [0 : steps-1]) [c[a][0]+lenCD*cos(angles[a][2]),c[a][1]+lenCD*sin(angles[a][2]),0]];

module tube_model(t=1,wall=0.1,l=10) {
    color("grey") linear_extrude(height=l) difference() {
        square(t,center=false);
        translate([wall,wall,0]) square(t-2*wall,center=false);
    }
}
*tube_model(t=t1,wall=twall,l=20);

module servo_mount() {
    // servo mount is designed around servo with zero transformations.
    // servo center  is at x=0,y=0, Axis = z, Top of servo z = 0
    // x axis is long axis of servo
    $fn=$preview ? 64 : 128; // minimum number of fragements
    
    xtotal = svo_l + 34;
    xshift = svo_l/2 - svo_shaft;
    y1 = 4;
    z1 = 25; // 1 inch = 25.4 mm.  
    
    difference () {
        translate([-xshift,y1/2+0.1,-z1/2])
            cube([xtotal,svo_w+y1,z1],center=true);

        servo_body (vis=false);
        translate([-xshift-1,0,-5])
            cube([svo_flange_l,svo_w,11],center=true);
        
        translate([0,0,-12]) rotate([90,0,0]) {
            hole_pair (x = 22,y=15,d=3,h=40);
            translate([0,0,-20]) hole_pair (x = 22,y=15,d=6,h=40);
            hole_pair (x = -43,y=15,d=3,h=40);
            translate([0,0,-20]) hole_pair (x = -43,y=15,d=6,h=40);
        }
    }
}
*servo_mount();
*color("red") servo_body();  // shoulder servo

module guss_profile(tube=1,gap=0.1,daxel=2,dholes=3.5) {
    // 2D SHAPE.  Can write .svg
    // Profile of a gusset for tube of width tube
    $fn=$preview ? 64 : 120; // minimum fragment
    r=tube/4;
    poly1 = [[0,0],[0,-1.5*tube-gap+r],[-r,-1.5*tube-gap],[-2*tube+r,-1.5*tube-gap],[-2*tube,-1.5*tube-gap+r],[-2*tube,-0.5*tube-gap-r],[0,tube/2]];
    difference () {
        hull() {
            polygon(poly1);
            circle(tube/2);
        }
        circle(d=daxel);
        translate([-1.5*tube/2-7,0,0]) 
            rotate([0,0,90]) 
                hole_pair_2D (x = -tube-gap,y=1.5*tube,d=dholes); 
    }
}
module plain_guss() {
    color("green") difference () {
        linear_extrude(6) guss_profile(tube=t1,gap=t2,daxel=hole_qtr_inch,dholes=3.5);
        
        // use one or the other, depending on which side wanted
        //cube([200,t1+2*t2,1],center=true);
        //cube([t1,100,1],center=true);
        translate([0,0,6]) cube([200,t1+2*t2,1],center=true);
        translate([t1/2-2,0,6]) cube([t1,100,1],center=true);
    }
}
*plain_guss();

module big_gear_guss() {
    $fn=$preview ? 64 : 120; // minimum fragment
    union() {
        32P_Actobotics(teeth=big_gear_teeth,thickness=8,bore=hole_qtr_inch*1.1);    
        linear_extrude(8) guss_profile(tube=t1,gap=t2,daxel=hole_qtr_inch,dholes=3.5);
    }
}
*big_gear_guss();

module base_assy(A1=0){
    // Draw B joint
    // 
    // Fixed tube
    translate([t1/2,-t1/2,-t1]) rotate([0,-90,0]) tube_model(t=t1,wall=twall,l=lenAB);

    y1 = 25.4/2 + svo_w/2; // offset in the y direction
    ang = asin(y1/(big_gear_rad+small_gear_rad));
    //echo(ang=ang);
    x1=(big_gear_rad+small_gear_rad)*cos(ang);
    rotate([0,0,A1]) {
        big_gear_guss();
        translate([0,0,-t1-6]) plain_guss();
        // Moving tube
        translate([-2,-1.5*t1-t2,-t1]) 
            rotate([0,-90,0]) tube_model(t=t1,wall=twall,l=100);
    }
    // the distance between gears is the teeth*pitch/pi
    translate([-x1,y1,0]) {
        color("blue") servo_mount();
        color ("red",.5) servo_body();  // shoulder servo
        // off for thingiverse, purchased part
        32P_Actobotics(teeth=small_gear_teeth);   // servo gear 32 tooth
    }
}
difference() {
    base_assy(A1=90); // not for print
    *translate([-150,-150,-306]) cube(300);
}

*draw_assy(angles[0][0],-0,angles[0][2],0,lenAB,lenBC,lenCD);


function get_CX (a) = (cos(a[0])*lenAB+cos(a[0]+a[1])*lenBC);
function get_CY (a) = (sin(a[0])*lenAB+sin(a[0]+a[1])*lenBC);

function sweep1(t=0.0,lowA=0,highA=180,lowB=0,highB=180,c=10) =
(t<0.5) ? ([linear_interp(lowA,highA,t,0,0.5),lowB,c]) : 
          ([highA,linear_interp(lowB,highB,t,0.5,1.0),c]) ;

module calc_forces() {
        // This should be a module, and put inside of the arm.scad
    // USE LIST COMPREHENSIONS TO FILL ARRAYS
    // MOMENT ON CD
    C_mom = [ for (a = [0 : steps-1]) (payload*lenCD+CDweight*cgCD)*cos(angles[a][0]+angles[a][1]+angles[a][2]) ];
    Cmom_min=min(C_mom);
    Cmom_max=max(C_mom);
    Margin_Safety(Cmom_min,Cmom_max,Motor_Max_Torque,"C moment");
        
    // MOMENT ON B, DUE TO CD AND BC
    B_trq = [ for (a = [0 : steps-1]) (BCweight*cgBC+(payload+CDweight)*lenBC)*cos(angles[a][0]+angles[a][1])+C_mom[a]];
    B_trq_min=min(B_trq);
    B_trq_max=max(B_trq);
    Margin_Safety(B_trq_min,B_trq_max,Motor_Max_Torque,"B SERVO - GREEN");
    *draw_3d_list(c,max_range/100,"green",B_trq/400); 
    *echo(B_trq=B_trq);
    
    B_trq_noload = [ for (a = [0 : steps-1]) (BCweight*cgBC+(CDweight)*lenBC)*cos(angles[a][0]+angles[a][1])+CDweight*cgCD*cos(angles[a][0]+angles[a][1]+angles[a][2])];

    A_trq_load_nospr = [ for (a = [0 : steps-1]) (ABweight*cgAB+(BCweight+payload+CDweight)*lenAB)*cos(angles[a][0])+ B_trq[a] ]; 
    A_trq_load_nospr_min=min(A_trq_load_nospr);
    A_trq_load_nospr_max=max(A_trq_load_nospr);
    Margin_Safety(A_trq_load_nospr_min,A_trq_load_nospr_max,Motor_Max_Torque,"A SERVO - NO SPRING - BLUE");
    *draw_3d_list(c,max_range/80,"blue",A_trq_load_nospr/400); 
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
    A_trq_load_spr_min=min(A_trq_load_spr);
    A_trq_load_spr_max=max(A_trq_load_spr);
    Margin_Safety(A_trq_load_spr_min,A_trq_load_spr_max,Motor_Max_Torque,"A SERVO - SPRING - BLUE");
    echo(A_trq_load_spr=A_trq_load_spr);
    draw_3d_list(c,max_range/80,"blue",A_trq_load_spr/400); 
    
    // calculate max A moment with NO payload and spring (can be critical!)
    A_trq_noload_spr = [ for (a = [0 : steps-1]) (ABweight*cgAB+(BCweight+CDweight)*lenAB)*cos(angles[a][0])+ B_trq_noload[a]  - A_spr_torque[a] ]; 
    A_trq_noload_spr_min=min(A_trq_noload_spr);
    A_trq_noload_spr_max=max(A_trq_noload_spr);
    Margin_Safety(A_trq_noload_spr_min,A_trq_noload_spr_max,Motor_Max_Torque,"A SERVO - SPRING - NO PAYLOAD - Yellow");
    draw_3d_list(c,max_range/70,"yellow",A_trq_noload_spr/400); 
    *echo(A_trq_noload_spr=A_trq_noload_spr);
        
    *rotate([-90,0,0]) for (a = [0 : steps-1]) 
        draw_assy(angles[a][0],angles[a][1],angles[a][2],0,lenAB,lenBC,lenCD);

}
*calc_forces();

*for (a = [0 : steps-1]) draw_assy(angles[a][0],angles[a][1],angles[a][2],0,lenAB,lenBC,lenCD);

