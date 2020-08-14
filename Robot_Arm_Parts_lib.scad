// Robot Arm Parts Object Library
//  Started on 4/6/2020 by SrAmo
//  last modified 7/26/2020 by SrAmo
use <force_lib.scad>
use <Pulley-GT2_2.scad>
include <SACC-26-Configuration.scad>

// Rounded Cube
display_round_cube = false;
// Point to Point Cylinder
display_pt_pt_cylinder = false;
// Point to Point belt over 2 pulleys
display_pt_pt_belt = false;
// Lug or Padeye
display_lug = false;
// Simple Link (2 force member)
display_simple_link = false;
// Simple Dog Leg
display_dog_leg = false;
// Fancy Dog Leg (hollow with clevis and pin hole)
display_fancy_dog_leg = false;
// Hollow offset link (two fancy dog legs, mirrored about x=0)
display_hollow_link = false;
// Fork attachment for End
display_fork = false;
// Compliant U Claw for End (90 deg fixed)
display_U_Claw = false;
// Compliant adjustable Claw (Variable angle)
display_V_Claw = false;
// Pulley for round belt
display_pulley = false;
// Spacer or Washer
display_spacer = false;
// Calculate and display round cross belt for pulleys
//display_cross_belt = false;
// Tension Spring (low quality)
display_spring = false;
// Balance Weight Arm for Servo
//display_balance_arm = false;
// Servo Horn (for subtracting from solids)
display_servo_horn = false;
// Servo Body with screw holes (for subtracting from solids)
display_servo_body = false;
// Servo Body Shim
display_servo_shim = false;
// 3 point cubes
display_3_pt_cube = false;
// display GT2-2 Idler Pulley
display_GT2_idle_pulley = false;
// display 1/4 inch ID flanged bearing
display_qtr_flng_bearing = false;
// display M5 Rounded Head Screw
display_M5_RHS = false;
// display hex cylinder (for nuts)
display_hex = false;
// display tube barb (for hooking ends of tube together)
display_barb = false;


if (display_round_cube) rounded_cube();
if (display_lug) lug();
if (display_simple_link) simple_link(l=1.8,w=0.5,t=0.17);
if (display_dog_leg) dog_leg ();
if (display_fancy_dog_leg) fancy_dog_leg ();
if (display_hollow_link) hollow_offset_link ();
if (display_fork) color ("green") fork ();
if (display_U_Claw) color ("yellow") compliant_claw (l=6,w=4.6,t1=0.075,t2=1.5);
if (display_V_Claw) color ("blue") compliant_claw2 (l=6,w=4.6,t1=0.075,t2=1.5,pre_angle = 15);
if (display_pulley) pulley ();
//if (display_pulley) translate ([0,0,1]) pulley (round=false);
if (display_spacer) spacer ();
if (display_spring) tension_spring ();
if (display_pt_pt_cylinder) pt_pt_cylinder (d=.5);
if (display_pt_pt_belt) {
    color ("black") pt_pt_belt (round=false);
    color ("grey") pt_pt_belt ([2,0,0],[-2,-1,0],round=true);
}
//if (display_cross_belt) cross_belt ();
//if (display_balance_arm) balance_weight ();
if (display_servo_horn) servo_horn ();
if (display_servo_body) servo_body ();
if (display_servo_shim) servo_shim (t=.2);
if (display_GT2_idle_pulley) GT2_2_idle_pulley();
if (display_qtr_flng_bearing) bearing_flng_qtr ();
if (display_M5_RHS) M5_RHS (length=20);
if (display_hex) hex ();
if (display_barb) tube_barb ();
    
if (display_3_pt_cube) {
    p = [[0,0,0] , [10,0,-20], [-30,20,10] ];
    cubeOnThreePoints(p); 
}

module rounded_cube(size=[10,20,10],r=1,center=true) {
    // Create a rounded cube in the xy plane, flat on the Z ends
    // Creates 4 cylinders and then uses hull
    $fa=$preview ? 2 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    
    xp=center ? size[0]/2-r : size[0]-r;
    yp=center ? size[1]/2-r : size[1]-r;
    xn=center ? -xp : r;
    yn=center ? -yp : r;
    z=center ? 0 : size[2]/2;

    hull () {
        translate([xp,yp,z])
            cylinder(h=size[2],r=r,center=true);
        translate([xp,yn,z])
            cylinder(h=size[2],r=r,center=true);
        translate([xn,yp,z])
            cylinder(h=size[2],r=r,center=true);
        translate([xn,yn,z])
            cylinder(h=size[2],r=r,center=true);       
    }  
}
module lug (r=1,w=3,h=2,t=.2,d=0.02) {
    // Create a lug part on the xy plane, thickness t from z=0
    //   base is on y=0 and has width w
    //   center of lug is at [0,h] with radius r
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    opp=(w-2*r)/2;
    ang=atan2(opp,h);
    y=h+r*sin(ang);
    x=r*cos(ang);
    difference() {
        union () {
            translate([0,h,t/2])   
                cylinder(h=t,r=r,center=true);
    
            linear_extrude(height = t)
                polygon(points=[[-w/2,0],[w/2,0],[x,y],[-x,y],[-w/2,0]]);
        }
        translate([0,h,t/2])  
            cylinder (h=3*t,d=d,center=true);
    }
}
module simple_link (l=5,w=0.5,t=0.2,d=0.1) {
    // Simple two force link, normale to xy plane, pointing x
    // l=Lenth, w=Width, t=Thickness, d=bord Diameter
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.03; // minimum size of fragment (default is 2)
    difference () {
        union () {
            cylinder (h=t,d=w,center=false);
            translate([l,0,0])
                cylinder (h=t,d=w,center=false);
            translate([0,-w/2,0])
                cube ([l,w,t],center=false);
        }
        cylinder (h=3*t,d=d,center=true);
        translate([l,0,0])
            cylinder (h=3*t,d=d,center=true);        
    }
}
module dog_leg (d1=10,ang=45,d2=5,w=2,t=1) {
    // Create a Dog Leg part on the xy plane, along x axis
    // First length is d1, turn is ang, second length is d2
    // Thickness is t, positive z from zero
    // Width is w
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.02; // minimum size of fragment (default is 2)
    dx = d1 + d2*cos(ang); //x of end of leg
    dy = d2*sin(ang);      // y of end of leg
    Pdist = sqrt(dx*dx+dy*dy);
    fx=d1-tan(ang/2)*(w/2);
    fs = 0.02*Pdist;  // inside chamfer size
    union () {
        translate([0,-w/2,0]) // first leg
            cube([d1,w,t],center=false);
        translate([d1,0,0])   // second leg
            rotate([0,0,ang])
                translate([0,-w/2,0])
                    cube([d2,w,t],center=false);
        translate([dx,dy,0])   // rounded end
            cylinder(h=t,d=w,center=false);
        translate([d1,0,0])    // rounded knee
            cylinder(h=t,d=w,center=false);
        linear_extrude(height=t,convexity=10) // add inside chamfer
            polygon([[fx,w/2],[fx-fs,w/2],[fx+fs,w/2+fs*tan(ang)],[fx+fs,w/2],[fx,w/2]]);
    }
}
module fancy_dog_leg (d1=10,ang=45,d2=4,w=2,t=1,d_pin=0.25,wall=0.2) {
    // Create dog leg with center and pin-holes removed
    // padeye is the same thing as a lug 
    // The padeyes will fit a pin of diameter d_pin
    // wall is the wall thickness of the link
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    dx = d1 + d2*cos(ang);
    dy = d2*sin(ang);
    t_inside = t-2*wall; // z direction
    w_inside = w-2*wall; // y direction
    clevis_r = (d2<w) ? w : d2/1 ;
    fillet_r = t/3;
    difference() {
        dog_leg(d1,ang,d2,w,t);
        
        translate([dx,dy,-t/2])   // hole for pin
            cylinder(h=t*2,d=d_pin,center=false);
        
        // cylinder to make clevis
        translate([dx-w/.74,2*w,t_inside+wall])   
            rotate([-15,90,-90])
            rounded_cube([t_inside,4*w_inside,d1+d2],fillet_r,center=false);
        
        // hollow the first leg
        translate([-d2/2,-w_inside/2,t_inside+wall]) 
            rotate([0,90,0])
            rounded_cube([t_inside,w_inside,d1+d2],fillet_r,center=false);
    }
}
module hollow_offset_link (length=20,d_pin=0.5,w=2,t=2,offset=1,
        ang=45,wall=0.1,pulley_r=1,d_grv=0.1,right=true) {
    // Create a Hollow Offset Link on xy plane along the X axis 
    // First joint is at [0,0], Second joint is at [length,0]
    // padeye is the same thing as a lug 
    // The padeyes will fit a pin of diameter d_pin
    // link width is w (y dimension), constant along length of part
    // thickness is t, positive z from zero
    // offset is the distance down from the x axis
    // ang is the dog leg angle
    // wall is the wall thickness of the link
    
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    d2 = offset/sin(ang);   // length of second dog leg segment
    d1 = length/2 - d2*cos(ang); // length of first dog leg segment
    fillet_r = t/4;        
    difference () {
        translate ([length/2,offset,0])
            rotate ([0,0,180]) // flip the offset up
            union () {  // not sure what union?
                fancy_dog_leg (d1,ang,d2,w,t,d_pin,wall);
                mirror ([1,0,0]) // make a mirror copy
                    fancy_dog_leg (d1,ang,d2,w,t,d_pin,wall);
            }
        translate([length/2,0,w/2])
            rotate([90,0,0])
            rounded_cube([length,0.6*w,t],fillet_r,center=true);
    }
}

module fork (l_fork=2,d_fork=0.2) {
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.03; // minimum size of fragment (default is 2)
    difference() {
        cylinder(h=l_fork,d1=6*d_fork/4,d2=3*d_fork/4,center=true);
        translate([-d_fork/2,-d_fork*1.5,0])
            rotate([-5,0,0])
                cube([d_fork,d_fork,l_fork],center=false);
    }
}
module compliant_claw2(l=5,w=4.6,t1=0.075,t2=1,r=claw_corner,pre_angle=0) {
    // U shaped claw with a pre angle
    // implement corner chamfers
    // Draws half and uses mirror
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)
    lug_t=0.17;
    poly_z = t2/2-lug_t/2;
    
    module subtract_1 () {
        linear_extrude(height = 2)
        polygon(points=[[0,0],[-.4,-poly_z/2],[-1.4,-poly_z],[-2,-poly_z],[-2,1],[0,0]]);
    }    
    module subtract_2 () {
        linear_extrude(height = 1)
            polygon(points=[[-t2/3,0],[0,t2/3],[t2/3,0],[-t2/3,0]]);
    }
    
    half_claw ();
    mirror([1,0,0]) half_claw ();
    
module half_claw () {    
    // The cylinder part of the claw
    rotate([0,0,-90])
        translate([-r-t1,w/2-r,0])
        rotate([0,0,-90]) 
        rotate_extrude(angle=180+pre_angle,convexity = 20)
            translate([r, 0, 0])
                square([t1,t2],center=false); // on X,Z plane
    // Everything else
    union () {
       difference () {
           // top of the U, flat cube
           x9 = w/2-2*r;  // local x
           //echo(x9=x9);
           back_height = 2.2; // match claw interface
           translate([0,r,0]) cube([x9,3*t1,back_height],center=false);
           // remove top chamfers
           translate([0,2*r,0]) rotate([90,0,0])
            linear_extrude(height = 2*r)
                polygon(points=[[x9,back_height-.6],[x9+.2,back_height+.1],[x9-.6,back_height],[x9,back_height-.6]]);
           // remove bottom chamfers
           translate([0,2*r,0]) rotate([90,0,0])
            linear_extrude(height = 2*r)
                polygon(points=[[0,1],[.1,1],[.8,0],[-.1,-.1],[0,1]]);
           // subtract claw attach hole
            //rotate ([90,0,0])
            //    translate ([.3,t2/2,0])
            //        cylinder (h=1,d=hole_M3_inch,center=true,$fn=32);
        }
        // The long Finger and the link to the servo
        // Multiple transformations to the preload
        translate([w/2-r,r+t1,0])
        rotate([0,0,pre_angle]) 
        translate ([r,0,0]) { // x=r
            difference () {
                union() {
                translate ([-1.2,claw_servo_y-r,0])rotate ([0,0,90])
                    lug (r=0.2,w=t1,h=.6,t=t2,d=hole_servo_bushing,$fn=32);
                translate ([-1.3,claw_servo_y-r-t1/2,0]) cube([1.3,t1,t2]);
                }
                // subtract lug features
                translate ([0,0,0]) rotate ([-90,0,0]) subtract_1();
                translate ([0,2,t2]) rotate ([90,0,0]) subtract_1();
}
            difference () {
                cube([t1,l-r,t2],center=false);
                // subtract end chamfers
                translate ([-.1,l-r+t1,-t1]) rotate ([90,0,90]) subtract_2();
                translate ([.5,l-r+t1,t2+.05]) rotate ([-90,0,90]) subtract_2();
            }
        }
    }
}

}
module compliant_claw(l=5,w=4.6,t1=0.075,t2=1) {
    // original version. U shaped.

    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    union () {
        difference () {
            difference () {
                translate ([-w/2,0,0])
                    rounded_cube([w,l+2,t2],r=0.7,center=false);
                
                translate ([-w/2+t1,t1,-t2/2])
                    rounded_cube([w-2*t1,l+2,2*t2],r=(0.7-t1),center=false);
        }
        translate ([-w,l,-t2])
            cube([2*w,4,3*t2],center=false);
        rotate ([90,0,0])
            translate ([.3,t2/2,0])
                cylinder (h=1,d=hole_M3_inch,center=true);
        rotate ([90,0,0])
            translate ([-.3,t2/2,0])
                cylinder (h=1,d=hole_M3_inch,center=true);
    }
    rotate ([0,0,90])
        translate ([1.8,-w/2,t2/2-0.17/2])
            lug (r=0.25,w=1,h=0.5,t=0.17,d=hole_M3_inch);
    rotate ([0,0,-90])
        translate ([-1.8,-w/2,t2/2-0.17/2])
            lug (r=0.25,w=1,h=0.5,t=0.17,d=hole_M3_inch);
    }
}
module pulley(r=2,t=.5,d_pin=0.25,d_grv=0.25,round=true){
    // Create pulley on xy plane at 0,0,0 of radius r
    // t is thickness (centered about z=0)
    // Add option for V-groove 4/22/2020
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.03; // minimum size of fragment (default is 2)

    difference(){
        cylinder(t,r=r,center=true);  // outside of pulley
        
        // subtract bore
        cylinder(2*t,d=d_pin,center=true);
        
        // subtract outside groove 
        pulley_groove(r=r,d_grv=d_grv,round=round);
    };

}
module pulley_groove(r=2,d_grv=0.25,round=true){
    // Create pulley groove on xy plane at 0,0,0 of radius r
    // Add option for V-groove 4/22/2020
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.03; // minimum size of fragment (default is 2)

    // subtract outside groove 
    rotate_extrude(convexity = 10)
        if (round) {  // draw a round groove
            translate([r, 0, 0])
            circle(d = d_grv);
        } else {   // draw a V-groove
            translate([r,0,0]) 
            hull () {
                translate([-d_grv/4,0,0])
                    circle(d=d_grv/2);
                translate([d_grv/2,0,0])                    square([d_grv,d_grv],center=true);
            }
        }
}
module spacer(d=.35,t=.13,d_pin=0.255){
    // Create spacer (washer) on xy plane at 0,0,0 of radius r
    // t is thickness (centered about z=0)
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.03; // minimum size of fragment (default is 2)

    difference(){
        cylinder(t,d=d,center=true);  // outside
        
        // subtract bore
        cylinder(2*t,d=d_pin,center=true);
        };
}
module tension_spring(from=[0,0,0],to=[6,5,5]){
    // Create a tenstion spring
    // UPGRADE TO DO A PROPOER LINEAR EXTRUDE
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    vector=vector_subtract(from,to);
    slength=norm(vector);
    if ((slength>0.8) && $preview) {
        ang=atan2(vector[1],vector[0]);
        translate ([from[0],from[1],from[2]]) 
        rotate ([0,0,ang]) rotate([0,90,0])
        linear_extrude(height=slength,center=false,convexity=10,
        twist = -3000)
        translate([.3, 0, 0]) circle(r = .1);  
        
        translate ([from[0],from[1],from[2]]) sphere(.25);
        
        translate ([to[0],to[1],to[2]]) sphere(.25);
    } else {
        echo("MODULE TENSION_SPRING; small spring = ",slength," or render");
    }
    
}
module pt_pt_cylinder (from=[1,1,0],to=[-1,0,-1], d = 0.1){
    // Create a cylinder from point to point
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    
    vec=vector_subtract(to,from);
    length=norm(vec);
    dx = -vec[0];
    dy = -vec[1];
    dz = -vec[2];

    if (length>0.01) {  // check for non zero vector
        
        // "cylinder" is centered around z axis
        // These are the angles needed to rotate to correct direction
        ay = 90 - atan2(dz, sqrt(dx*dx + dy*dy));
        az = atan2(dy, dx);
        angles = [0, ay, az];
        
        translate (from) 
        rotate (angles) 
        cylinder(length,d = d,false);  
    }else {
        echo("MODULE PT_PT_CYLINDER; small length =",length);
    }
}
module pt_pt_belt (from=[1,1,1],to=[-1,0,1], d = 0.1,r_pulley=.3,round=true){
    // Create belts from point to point
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.03; // minimum size of fragment (default is 2)
    
    // default is round belt
    // if round = false, then draw a GT2-6 belt
    gt2t=0.04; // thickness (1 mm)
    gt2w=0.236; // width (6 mm)
    
    vec=vector_subtract(to,from);
    length=norm(vec);
    dx = -vec[0];
    dy = -vec[1];
    dz = -vec[2];
    from_up = [from[0]+(dy/length)*r_pulley,from[1]-(dx/length)*r_pulley,from[2]];
    from_down = [from[0]-(dy/length)*r_pulley,from[1]+(dx/length)*r_pulley,from[2]];

    if (length>0.01) {  // check for non zero vector
        
        // "cylinder" is centered around z axis
        // Angles to rotate to correct direction
        ay = 90 - atan2(dz, sqrt(dx*dx + dy*dy));
        az = atan2(dy, dx);
        angles = [0, ay, az];
        
        translate (from_up) 
            rotate (angles) 
                if (round) 
                    cylinder(length,d = d,center=false);  
                else {
                    translate ([-gt2w/2,-gt2t/2,0])
                    cube([gt2w,gt2t,length],center=false);
                }
        translate (from_down) 
            rotate (angles) 
                if (round) 
                    cylinder(length,d = d,false);  
                else {
                    translate ([-gt2w/2,-gt2t/2,0])
                    cube([gt2w,gt2t,length],center=false);
                }
        translate(from)
            rotate([0,0,az+90])
                rotate_extrude(angle=180,convexity = 10)
                    translate([r_pulley, 0, 0])
                        if (round) circle(r = d/2);
                        else square([gt2t,gt2w],center=true);
        translate(to)
            rotate([0,0,az-90])
                rotate_extrude(angle=180,convexity = 10)
                    translate([r_pulley, 0, 0])
                        if (round) circle(r = d/2);
                        else square([gt2t,gt2w],center=true);
    }else {
        echo("MODULE PT_PT_BELT; small length =",length);
    }
}
module cross_belt(a=[0,0,0],b=[5,0,0],r=1, d = 0.1,right=true){
    // Create a Cross Belt cylinder from pulley a to pulley b
    // of radius r and wire diameter d
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    
    vec=vector_subtract(b,a);
    length=norm(vec);
    if (length>2*r) {  // check for feasible
        phi=right ? acos(r/(length/2)) : -acos(r/(length/2));
        az=atan2(-vec[1],-vec[0]); // angle of ab vector on xy plane
        a_tangent = vector_add(a,[r*cos(az-phi),r*sin(az-phi),0]);
        b_tangent = vector_add(b,[r*cos(az+(180-phi)),r*sin(az+(180-phi)),0]);
        
        pt_pt_cylinder(a_tangent,b_tangent,d);
    }else {
        echo("MODULE CROSS_BELT; points too close = ",length);
    }
}
module balance_weight(l=3,r=1,t=.5,d_pin=0.25,d_grv=0.25){
    // Create balance_weight on xy plane at 0,0,0 of radius r
    // t is thickness (centered about z=0)
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.1; // minimum size of fragment (default is 2)
    ang=60;

    d1 = (l/2)/cos(ang/2);
    difference () {
        union () {
           translate ([0,0,t/2])
            scale ([mm_inch,mm_inch,mm_inch])
            pulley_gt2_2 ( teeth = 80 , pulley_t_ht = 7);
            cylinder(t,r=r,center=true);
        };
        // subtract bores
        cylinder(4*t,d=d_pin,center=true);
        translate ([l,0,0]) 
            cylinder(4*t,d=d_pin,center=true);
    }
}
module servo_horn (l=0.96, d1=.6, d2=.27, t=.31){
    // Create servo horn on xy plane, spline center at 0,0,0
    // horn length l pointing along x axis
    // spline end dia d1, other end d2, thickness t, from z=0 up
    // used for BOOLEAN SUBTRACTION
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.01; // minimum size of fragment (default is 2)
    difference () {
        union () {
        translate([0,0,t/2])
            cylinder(t,d=d1,center=true);
        translate([l,0,t/2])
            cylinder(t,d=d2,center=true);
        translate([0,-d2/2,0])
            cube([l,d2,t],center=false);
        rotate([0,0,-90])
            lug (r=d2/2,w=d1,h=l/2,t=t);
        }
        // subtract main axis cyl for visulization
        cylinder (h=2,d=0.18,center=true);
    }
}
module servo_body (vis=true){
    // Create servo body on xy plane, spline shaft center at 0,0,0
    // long body direction along -x axis
    // body from z=0 down
    // used for BOOLEAN SUBTRACTION
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.01; // minimum size of fragment (default is 2)

    difference () {
        union () {
        translate([-svo_shaft,0,-svo_d/2]) // main body
            rounded_cube(size=[svo_l,svo_w,svo_d],r=.03,center=true,$fn=24); 
        
        translate([-svo_shaft,0,-svo_flange_d-svo_flange_t/2])  // flange
            rounded_cube(size=[svo_flange_l,svo_w,svo_flange_t],r=.03,center=true);
        
        translate([-svo_shaft,0,-.35])
            cube(size=[svo_flange_l,.1,.12],center=true); // gussetts
        
        // cylinders for screw starts
        translate([svo_screw_l/2-svo_shaft,svo_screw_w/2,-svo_flange_d])
            cylinder(h=2*svo_shaft,d=hole_no6_screw,center=true);
        translate([svo_screw_l/2-svo_shaft,-svo_screw_w/2,-svo_flange_d])
            cylinder(h=2*svo_shaft,d=hole_no6_screw,center=true);
        translate([-svo_screw_l/2-svo_shaft,-svo_screw_w/2,-svo_flange_d])
            cylinder(h=2*svo_shaft,d=hole_no6_screw,center=true);
        translate([-svo_screw_l/2-svo_shaft,svo_screw_w/2,-svo_flange_d])
            cylinder(h=2*svo_shaft,d=hole_no6_screw,center=true);
        }
        // subtract main axis cyl for visulization
        if (vis) cylinder (h=4*svo_d,d=0.16,center=true);
    }
}
module servo_shim (l=2.4,w=1,t=0.1) {
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.03; // minimum size of fragment (default is 2)
    flange_z = -.38;
    difference () {
        translate([-svo_shaft,0,flange_z+t/2]) cube([l,w,t],center=true);
        servo_body (vis=false,$fn=16);
    }
}
module GT2_2_idle_pulley () {
    $fs=0.1; // minimum size of fragment (default is 2)
    // draw in inches (always scaled)
    color ("Silver") 
    scale ([mm_inch,mm_inch,mm_inch])
        translate ([0,0,4.25])
        difference () {
            cylinder(h=8.5,d=18,center=true);
            cylinder(h=9,d=5,center=true);
            difference () {
                cylinder(h=6.5,d=20,center=true);
                cylinder(h=8,d=12,center=true);
            }
    }
}
module bearing_flng_qtr () {
    $fs=0.05; // minimum size of fragment (default is 2)
    color ("SlateGrey") 
    
    difference () {
        union() {
            translate ([0,0,-bearing_t])
                // main body, slightly undersize for assembly visibility
                cylinder(h=bearing_t,d=bearing_od-.01,center=false); 
            
            // flange, slightly less thick for assembly visibility
            cylinder(h=bearing_flange_t-.01,d=bearing_flange_od,center=false);
        }
    cylinder(h=bearing_t*3,d=hole_p25_inch,center=true); // center hole
    }
}
module M5_RHS (length=10) {
    // Make a M5 Round Head Screw of length length
    // draw in inches (always scaled)
    $fs=0.1; // minimum size of fragment (default is 2)
    color ("DarkSlateGrey") 
    scale ([mm_inch,mm_inch,mm_inch])
    union () {
        cylinder(h=length,d=4.8,center=false);
        //cylinder(h=9,d=5,center=true);
        rotate_extrude(convexity=10) 
            polygon( points=[[0,0],[4.5,0],[4.5,-2],[3,-2.9],[0,-2.9],[0,0]] );
    }
}
module hex (size=0.5,l=1) {
    // Make a hex extrution with distance across flats = size
    // centered on xy=0 and up l from z=0
    x = size/2;
    y1 = x * tan(30);
    y2 = x / cos(30);
    linear_extrude(height=l,convexity=10) 
        polygon( points=[[x,y1],[0,y2],[-x,y1],[-x,-y1],[0,-y2],[x,-y1],[x,y1]] );
}


module tube_barb (ID=4.763, OD=6.35) {
    // barb to connect two ends of tube into a loop
    // ASSUMING METRIC
    $fn=24;
    barb ();
    mirror ([0,0,1]) barb(); 
    
    module barb () {
        rotate_extrude ()
            polygon([[0,0],[0,1.5*ID],[ID/2,1.5*ID],[OD/2,1.25*ID],[OD/2,ID],[ID/2,ID],[ID/2,.2*ID],[OD/2,.2*ID],[OD/2,0],[0,0]]);
    }
}
module above_plane(p1,p2,p3,s=100) {
        v1 = p2 - p1;
        v2 = p3 - p1;
        n = cross(v1,v2);
        n2 = n / norm(n) * sign(n.z);
        cp = (p1 + p2 + p3) / 3;
        axis = cross([0,0,1],n2);
        dot = [0,0,1] * n2;
        ang = acos(dot>1? 1 : dot<-1? -1 : dot); // Fixes FP rounding errors.
        difference() {
                children();
                translate(cp) rotate(a=ang,v=axis) translate([0,0,-2*s]) cube(4*s,center=true);
        }
}

module cubeOnThreePoints(p) {
  norml = (cross(p[0]-p[1],p[0]-p[2]));
  normal = sign(norml.z)*norml;

  size = max(norm(p[0]-p[1]),norm(p[1]-p[2]),norm(p[0]-p[2]))*sqrt(2);
    echo (size=size);
  translate((p[0]+p[1]+p[2])/3 ) // center at the baricenter
    rotFromTo([0,0,1],normal)
      translate(-size*[1,1,0]/2)
        cube(size);
// just to illustration
  color("blue")
  for(pi=p) translate(pi) cube(3,center=true);

}

// minimum rotation that brings direction di to direction do
module rotFromTo(di,do)
  if( norm(di-do)==0 || norm(di)==0 || norm(do)==0 )
    children();
  else
    mirror(do/norm(do)+di/norm(di)) mirror(di) children();
