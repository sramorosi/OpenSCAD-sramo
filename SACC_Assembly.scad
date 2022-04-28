// SACC Robot Arm Assembly,  MAKE 2
// SACC = Servo Actuated (robot arm with) Compliant Claw
//  Started on March 2020 by SrAmo
//  last modified March 2022 by SrAmo
//
//  To make printable models find "FOR_PRINT" and remove * suffix, 
//     then F6 & Export .stl

// Note: Thingiverse only wants one .scad file per publication
use <Robot_Arm_Parts_lib.scad>
//use <Pulley-GT2_2.scad>
use <gears_involute.scad>  // off for thingiverse, purchased part
use <arduino.scad>         // off for thingiverse, purchased part
include <Part-Constants.scad>

// Draw the Robot Arm Assembly
display_assy = false;
// Section cut at X = 0?
clip_yz = false;
// Section cut at Z = 0?
clip_xy = false;
// Joint A angle
AA = 110; // [0:1:180]
// Joint B angle
BB = -10; // [-45:1:145]
// Joint C angle
CC = -10; // [-145:1:145]
// Turntable angle
TT = 0; // [-60:60]
// length of A-B arm, mm
lenAB=195; 
// length of B-C arm, mm
lenBC=240; 
// A-B arm width. The section is square.
widthAB = 40;
// B-C arm width. The section is square.
widthBC=widthAB; 
// Arm wall thickness throughout
wall_t=4;  
wAB_inside = widthAB - 2*wall_t; // AB Slot inside (calculated)
wBC_inside = widthBC - 2*wall_t; // BC Slot inside (calculated)
// Number of teeth in AB ARM pulleys
AB_pulley_teeth = 80; 
// BC ARM pulley diameter, 2 inch = 50.8 mm
AB_pulley_d = 50.8; 
// BC ARM pulley thichness mm, subtract 2 mm for flange
AB_pulley_t = 10; 
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
// distance of shoulder plate from zero
shoulder_y_shift = 45; 
// Shoulder part offset down from zero, mm
shoulder_z_top = 40;  
// Shoulder thickness
shoulder_t = 7;
// Shoulder width
shoulder_w = 75;
// Shoulder length
shoulder_l = 88;
// Shoulder gusset thickness
t_guss = 7.5;
// Shoulder gusset offset from center
x_guss = 15;

// rotation of joint A torsion spring in installation (deg)
A_Tspr_rot = 6;  
// Turntable Gear Space Adjustment, mm
gear_space_adjustment = 1; 
// B Servo Horn angle adjustment from Vertical in installation (deg)
B_Horn_Zero_Angle = 30; 

//################### variables, modules and functions from use <Pulley-GT2_2.scad>
function tooth_spacing(teeth,tooth_pitch,pitch_line_offset)
	= (2*((teeth*tooth_pitch)/(3.14159265*2)-pitch_line_offset)) ;
// height of retainer flange, part of pulley_t_ht
retainer_ht = 1.;	
// height of idler flange, part of pulley_t_ht
idler_ht = 1.;		
module pulley_gt2_2( teeth = 40 , pulley_t_ht = 10, motor_shaft = 5.2, tooth_depth=0.764 , tooth_width=1.494 ,retainer=1,idler=1) 	{

    // for pulley 0.2, for belt -0.2 mm
    additional_tooth_width = 0.1; //mm
    
    //	If you need more tooth depth than this provides, adjust the following constant. However, this will cause the shape of the tooth to change.
    // for pulley 0, for belt 0 mm
    additional_tooth_depth = 0; //mm
        
    // calculated constants
    
    // The following set the pulley diameter for a given number of teeth
    pulley_OD = tooth_spacing (teeth,2,0.254);

	tooth_distance_from_centre = sqrt( pow(pulley_OD/2,2) - pow((tooth_width+additional_tooth_width)/2,2));
	tooth_width_scale = (tooth_width + additional_tooth_width ) / tooth_width;
	tooth_depth_scale = ((tooth_depth + additional_tooth_depth ) / tooth_depth) ;

	difference() {	 
		union()	{
			linear_extrude(pulley_t_ht)
            difference() 	{
			//shaft - diameter is outside diameter of pulley
			
			translate([0,0,0]) 
			rotate ([0,0,360/(teeth*4)]) 
            circle(r=pulley_OD/2);
			//cylinder(r=pulley_OD/2,h=pulley_t_ht, $fa=2);
	
			//teeth - cut out of shaft
		
			for(i=[1:teeth]) 
			rotate([0,0,i*(360/teeth)])
			translate([0,-tooth_distance_from_centre,-1]) 
			scale ([ tooth_width_scale , tooth_depth_scale , 1 ])  
                GT2_2mm(pulley_t_ht=pulley_t_ht);
			}
			
		//belt retainer / idler
		if ( retainer > 0 ) {translate ([0,0, pulley_t_ht-retainer_ht ]) 
		rotate_extrude($fa=2)  
		polygon([[0,-retainer_ht],[pulley_OD/2-retainer_ht,-retainer_ht],[pulley_OD/2 + retainer_ht , retainer_ht],[0 , retainer_ht],[0,-retainer_ht]]);}		
        
		if ( idler > 0 ) {translate ([0,0, 0 ]) 
		rotate_extrude($fa=2)  
        polygon([[0,0],[pulley_OD/2 + idler_ht,0],[pulley_OD/2-idler_ht , 2*idler_ht],[0 , 2*idler_ht],[0,0]]);}
		}
	   
		//hole for motor shaft
		translate([0,0,-1])
            cylinder(r=motor_shaft/2,h= pulley_t_ht + retainer_ht + 2,$fa=1,$fs=1);
	 }
}
// Tooth profile modules
module GT2_2mm(pulley_t_ht=6){
	/*linear_extrude(height=pulley_t_ht+2) */ polygon([[0.747183,-0.5],[0.747183,0],[0.647876,0.037218],[0.598311,0.130528],[0.578556,0.238423],[0.547158,0.343077],[0.504649,0.443762],[0.451556,0.53975],[0.358229,0.636924],[0.2484,0.707276],[0.127259,0.750044],[0,0.76447],[-0.127259,0.750044],[-0.2484,0.707276],[-0.358229,0.636924],[-0.451556,0.53975],[-0.504797,0.443762],[-0.547291,0.343077],[-0.578605,0.238423],[-0.598311,0.130528],[-0.648009,0.037218],[-0.747183,0],[-0.747183,-0.5]]);
}

//################### END variables, modules and functions from use <Pulley-GT2_2.scad>

/*################### variables, modules and functions from use <Robot_Arm_Parts_lib.scad>
function law_sines_angle (C=30,a=10,c_ang=120) = 
    // law of sines, given length C, a, and C Angle, return A angle
   asin((a/C)*sin(c_ang));

function law_sines_length (C=30,c_ang=120,b_ang=30) =
    // law of sines, given length C, c angle and b angle, return B length
   C * (sin(b_ang)/sin(c_ang));
module lug (r=1,w=3,h=2,t=.2,d=0) {
    // Create a lug part on the xy plane, thickness t from z=0
    //   base is on y=0 and has width w
    //   center of lug is at [0,h] with radius r
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
        if (d != 0) translate([0,h,t/2])  
            cylinder (h=3*t,d=d,center=true);
    }
}
module Cbore_Screw_Hole(d=3,h=21,cb_d=7,cb_h=2) {
    // Hole for a screw with counter bore, to hid the head
    $fn=$preview ? 16 : 32; // minimum angle fragment
    translate([0,0,h/2-0.1])cylinder(h=h,d=d,center=true);
    translate([0,0,-cb_h/2]) cylinder(h=cb_h,d=cb_d,center=true);
}

module simple_link (l=50,w=5,t=4,d=1,cored=2) {
    // Simple two force link, normale to xy plane, pointing x
    // l=Lenth, w=Width, t=Thickness, d=Pin bore Diameter, cored = Core diameter
    $fa=$preview ? 6 : 1; // minimum angle fragment
    difference () {
        hull () {
            cylinder (h=t,d=w,center=false);
            translate([l,0,0])
                cylinder (h=t,d=w,center=false);
        }
        if (d != 0) {
            cylinder (h=3*t,d=d,center=true);
            translate([l,0,0])
                cylinder (h=3*t,d=d,center=true);
        }     
        if (cored != 0) {
            translate([0,0,t/2]) rotate([0,90,0]) 
                cube([cored,cored,3*l],center=true);
                //cylinder (h=3*l+2*w,d=cored,center=true);
        }     
    }
}
module rounded_cube(size=[10,20,10],r=1,center=true) {
    // Create a rounded cube in the xy plane, flat on the Z ends
    // Creates 4 cylinders and then uses hull
    
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
module servo_horn (l=servo_horn_l, d1=servo_horn_d1, d2=servo_horn_d2, t=servo_horn_t,vis=true){
    // Create servo horn on xy plane, spline center at 0,0,0
    // horn length l pointing along x axis
    // spline end dia d1, other end d2, thickness t, from z=0 up
    // used for BOOLEAN SUBTRACTION set vis = false
    $fn=$preview ? 24 : 48; // minimum angle fragment
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
            if (!vis) {
                // SCREW HOLE FOR SECURING HORN
                translate([l*.85,0,t/3]) rotate([90,0,0]) cylinder(h=2*l,d=2.8,center=true);
            }
        }
        // subtract main axis cyl for visulization
        if (vis) cylinder (h=4*t,d=2,center=true);
    }
}module hole_pair (x = 50,y=10,d=hole_M3,h=100) {
    // make a pair of holes that are y appart, 
    // at x location. holes are parallel to the Z axis
    // Created for zip tie holes
    $fn=$preview ? 8 : 16; // minimum angle fragment
    translate ([x,-y/2,0]) cylinder(h=h,d=d,center=true);
    translate ([x,y/2,0]) cylinder(h=h,d=d,center=true);
}
module filled_donut(t=10,d=50, r = 2) {
    // t = donut thickness,   d = donut diameter, r = fillet radius
    // Fillet radius must be less than d/4.
    $fn=$preview ? 20 : 72; 
    if (r < d/4) {
        hull() {
        translate([0,0,t/2-r]) 
            rotate_extrude(convexity = 10) translate([(d/2)-r/2, 0, 0]) 
                circle(r = r);
        translate([0,0,-t/2+r]) 
            rotate_extrude(convexity = 10) translate([(d/2)-r/2, 0, 0]) 
                circle(r = r);
        }
    } else {
        echo("ERROR in Filled Donut. Fillet radius must be less than d/4");
    }
}
module washer(d=20,t=2,d_pin=10){
    // model washer on xy plane at 0,0,0 of radius r
    // t is thickness (centered about z=0)

    difference(){
        cylinder(t,d=d,center=true);  // outside
        
        // subtract bore
        cylinder(2*t,d=d_pin,center=true);
    };
}
module torsion_spring(deflection_angle=180,OD=1,wire_d=.1,leg_len=2,coils=5,LH=true,inverse=false) {
    // deflection_angle is not implimented
    
    if (deflection_angle != 180) echo("ONLY 180 DEG TORSION SPRINGS IMPLEMENTED");
        
    turn_sign = LH ? 1 : -1 ;  // used for LH or RH springs
    sp_len = (coils+1)*wire_d;
    //echo(sp_len=sp_len);
    ID = OD-2*wire_d;
    difference () {
        cylinder(h=sp_len,d=OD);
        if (inverse == false) translate([0,0,-sp_len*0.05]) 
            cylinder(h=sp_len*1.1,d=ID);
    }
    
    // straight legs on each end
    x_offset = OD/2 - wire_d/2;
    WD = (inverse != false) ? wire_d*1.25 :  wire_d;
    LL = (inverse != false) ? leg_len*1.25 :  leg_len;
    translate([x_offset,turn_sign*LL/2,0]) 
        rotate([90,0,0]) 
            hull() {
                if (inverse) translate([0,WD/2,0]) cylinder(h=LL,d=WD,center=true,$fn=16);
                cylinder(h=LL,d=WD,center=true,$fn=16);
            }
    translate([x_offset,-1*turn_sign*LL/2,sp_len]) 
        rotate([90,0,0]) 
            hull() {
                cylinder(h=LL,d=WD,center=true,$fn=16);
                if(inverse) translate([0,-WD/2,0]) cylinder(h=LL,d=WD,center=true,$fn=16);
            }
}
module servo_body (vis=true){
    // Create servo body on xy plane, spline shaft center at 0,0,0
    // long body direction along -x axis
    // body from z=0 down
    // used for BOOLEAN SUBTRACTION set vis = false
    $fn=$preview ? 24 : 48; // minimum angle fragment

    difference () {
        union () {
        translate([-svo_shaft,0,-svo_d/2]) // main body
            rounded_cube(size=[svo_l,svo_w,svo_d],r=.8,center=true,$fn=24); 
        
        translate([-svo_shaft,0,-svo_flange_d-svo_flange_t/2])  // flange
            rounded_cube(size=[svo_flange_l,svo_w,svo_flange_t],r=.8,center=true);
        
        translate([-svo_shaft,0,-8.9])
            cube(size=[svo_flange_l,2.54,3.05],center=true); // gussetts
        
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
        if (vis) cylinder (h=4*svo_d,d=4,center=true);
    }
}
module Bearing_Flanged (t=2,flange_t=1,od=3,id=1,flange_od=4) {
    $fn=64; 
    color ("SlateGrey") 
    difference () {
        union() {
            translate ([0,0,-t])
                // main body, slightly undersize for assembly visibility
                cylinder(h=t,d=od*.98,center=false); 
            
            // flange, slightly less thick for assembly visibility
            cylinder(h=flange_t*.98,d=flange_od,center=false);
        }
    cylinder(h=t*3,d=id,center=true); // center hole
    }
}
module Bearing (t=4,od=30,id=25) {
    $fn=64; 
    color ("Grey") 
    difference () {
        translate ([0,0,-t])
        // main body, slightly undersize for assembly visibility
        cylinder(h=t,d=od*.98,center=false); 
        cylinder(h=t*3,d=id,center=true); // center hole
    }
}

//This is used for rotational patterns
//child elements will be centered on 
module Rotation_Pattern(number=3,radius=20,total_angle=360) {
  ang_inc = total_angle/number;
  //echo(ang_inc=ang_inc);
  if (number>1 && radius > 0) {
      for(i = [0 : number-1 ] ) {
        rotate([0,0,i*ang_inc]) translate([radius,0,0])
          children(0);
          }
    } else {
      echo("INVALID ARGUMENTS IN Rotation_Pattern module");
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
module Power_Energy_Meter() {
    // model of bayite DC 6.5-100V 0-100A LCD Digital Current Voltage Power Energy Meter
    color("DarkGray") {
        translate([0,0,-11.15]) cube([45,87,22.3],center=true);
        translate([0,0,1]) cube([50,91,2],center=true);
        translate([0,0,-7]) cube([45,89,10],center=true); // for removal to make lip
    }
}
*Power_Energy_Meter();

module Rocker_Switch () {
    color ("DarkRed") {
        translate([0,0,-10]) cube([10.5,29,20],center=true);
        translate([0,0,1]) cube([13.7,30.7,2],center=true);
    }
}
*translate([40,0,0]) Rocker_Switch();

module Current_Shunt () {
    bolt_center = 86.55;
    color ("DarkCyan") {
        difference() {
            cube([15,105,2],center=true);
            translate([0,bolt_center/2,0]) cylinder(h=20,d=5,center=true);
            translate([0,-bolt_center/2,0]) cylinder(h=20,d=5,center=true);
        }
    }
}
module U_section(Lbase=20,Lleg=15,Tbase=2,Tleg=1) {
    // Create a U section polygon (2D)
    // the origin in the lower left
    if(Lbase>0 && Lleg>0 && Tbase>0 && Tleg>0) {
        polygon([[0,0],[Lbase,0],[Lbase,Lleg],[Lbase-Tleg,Lleg],[Lbase-Tleg,Tbase],[Tleg,Tbase],[Tleg,Lleg],[0,Lleg],[0,0]]);
    }
}
module pt_pt_belt (from=[10,10,10],to=[-10,0,10], d = 1,r_pulley=30,round=true){
    // Create belts from point to point
    
    // default is round belt
    // if round = false, then draw a GT2-6 belt
    gt2t=1; // thickness (1 mm)
    gt2w=6; // width (6 mm)
    
    vec= from - to;
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
module servo_connection(len=100,t1=2,t2=38) {
    // Compliant Beam that connects Claw to Servo
    
    module subtract_1 () { // profile of links to servo
        linear_extrude(height = t2)
        polygon(points=[[0,0],[-len/5,-poly_z/2],[-len/1.2,-poly_z],[-len*2,-poly_z],[-len*2,t2/2],[0,0]]);
    }  
    
    poly_z = t2/2;   
    servo_lug_h = len/2.2; // Smooth transition to lug. manage the stress at the lug.

    difference () {
        union() {
            // Link to the servo
            translate ([servo_lug_h-len,0,0]) rotate ([0,0,90])
                lug (r=1.5*hole_servo_bushing,w=t1,h=servo_lug_h,t=t2,d=hole_servo_bushing,$fn=32);
            translate ([-len+hole_servo_bushing,-t1/2,0]) cube([len-hole_servo_bushing,t1,t2]);
        }
        // subtract lug features
        translate ([0,-t2/2,0]) rotate ([-90,0,0]) subtract_1();
        //translate ([0,link_adjust+t2,t2]) rotate ([90,0,0]) subtract_1();
    }
}
module compliant_claw2(len=160,width=120,t1=2,t2=38,r=18,pre_angle=15) {
    // U shaped claw with a pre angle
    //    t1 = general compliant thickness
    //    t2 = height of part
    // implement corner chamfers
    // Draws half and uses mirror
    $fa=$preview ? 6 : 1; // minimum angle fragment
    $fs=0.05; // minimum size of fragment (default is 2)
    
    poly_z = t2/2;
    module subtract_2 () { // triangle removal on end of claw
        linear_extrude(height = t2)
            polygon(points=[[-t2/3,0],[0,t2/3],[t2/3,0],[-t2/3,0]]);
    }
    
    module half_claw (link_adjust=0) {    
        // The Half Cylinder part of the claw
        rotate([0,0,-90])
            translate([-r-t1,width/2-r,0])
            rotate([0,0,-90]) 
            rotate_extrude(angle=180+pre_angle,convexity = 20)
                translate([r, 0, 0])
                    square([t1,t2],center=false); // on X,Z plane
        
        // Everything else
        // The long Finger and the link to the servo
        // Multiple transformations to the preload
        y_link = link_adjust+r;
        translate([width/2-r,r+t1,0])
        rotate([0,0,pre_angle]) 
        translate ([r,0,0]) { // x=r
            translate([0,link_adjust+r,0]) servo_connection(len=52,t1=t1,t2=t2);
            
            // Long finger thicker section
            cube([2*t1,len/1.5-r,t2],center=false);
            
            difference () {
                // Long finger full length
                cube([t1,len-r,t2],center=false);
                // subtract end chamfers
                translate ([-t2/10,len-r+t1,-t1]) rotate ([90,0,90]) subtract_2();
                translate ([t2/2,len-r+t1,t2+1]) rotate ([-90,0,90]) subtract_2();
            }
        }
    } // end module half_claw
    
    // back plate
    x9 = width-4*r;  // local x
    back_height = t2; // match claw interface
    translate([-x9/2,r-3*t1,0]) cube([x9,6*t1,back_height],center=false);
    
    End_w = 10; // End effector interface width, mm
    // Add a cube to connect the back plate
    translate([-End_w/2,-r,0]) cube([End_w,2*r,t2],center=false);
    
    // DRAW THE CLAW HALVES
    half_claw (link_adjust=24); // modify link location this side
    mirror([1,0,0]) half_claw (link_adjust=9); 
}

//############### END variables, modules and functions from use <Robot_Arm_Parts_lib.scad> */

/*############### variables from include <Part-Constants.scad>
hole_M3=3.1; // hole for M3 (3 mm) joint/bolt
hole_M6=6.1;
hole_no6_screw = 2.5; // hole start diameter for number 6 screw 0.095 inch = 2.413
hole_qtr_inch=6.48;   // hole for 0.25 inch joint/bolt 0.255 inch = 6.477 mm
// 1/4 ID bearing OD if using a bearing (PART FR4-ZZ C3)
Qtr_bearing_od=15.9512; // 0.628 inch = 15.9512 mm
Qtr_bearing_flange_od=18.034; // .71 inch = 18.034 mm
Qtr_bearing_flange_t=1.27; // 0.05 inch = 1.27 mm
Qtr_bearing_t=3.937; // 0.155 inch = 3.937 mm

M6_bearing_od=12; 
M6_bearing_flange_od=13.6;
M6_bearing_flange_t=0.8;
M6_bearing_t=3.15;

// STANDARD 40MM X 20MM X 40MM SERVO DIMENSIONS
svo_l = 40.66; // Servo Length for openings in parts
svo_w = 20.3; // Servo Width for openings in parts
svo_d = 42; // Servo Depth from Horn interface to bottom of part
svo_shaft = 10.6; // Servo dist from shaft to edge of body in length direction, was 10.2
svo_screw_l = 49.5; // Servo screw hole length between
svo_screw_w = 10; // Servo screw hole width between
svo_flange_l = 56; // Servo flange length, was 55
svo_flange_t = 3; // Servo flange thickness
svo_flange_d = 10; // Servo depth from Horn interface to top of flange
servo_horn_l=25;
servo_horn_d1=15.25;
servo_horn_d2=7.5;
servo_horn_t=7.4;
hole_servo_bushing=3.81; // hole for servo bushing 0.15 inch = 3.81 mm

//  McMaster Carr 9271K619, torsion spring
9271K619_angle = 180; // free leg angle
9271K619_OD = 19.35; // mm
9271K619_wd = 1.9;  // wire diameter mm
9271K619_len = 51;  // arm length mm
9271K619_coils = 7;  // number of coils
9271K619_LH = true;  // LH or RH (false)
9271K619_t = (9271K619_coils+1)*9271K619_wd; // thickness of the spring
9271K619_ID = 9271K619_OD-2*9271K619_wd;  // ID of the coil

//############### END variables from include <Part-Constants.scad> */

module torsion_spring_spacer() {
    $fn=$preview ? 64 : 128; // minimum angle fragment
    translate([0,0,9271K619_t/2]) 
        washer(d=9271K619_ID*0.9,t=9271K619_t,d_pin=hole_M6*1.03);
}
*torsion_spring_spacer();  // FOR_PRINT

module AB_offset_link (length=50,w=15,offset=7,d_pin=5) {
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,0], Second joint is at [length,0]
    // link width is w (y dimension), constant along length of part
    // offset is the length of the second. IF NEGATIVE IT REVERSES THE SHAPE
    // Method of making a dog leg link, using law of sines:
    a_svo_boss = 8; // used for the A servo horn
    c_ang=135;
    A = (offset > 0) ? offset : -offset;
    a_angle = law_sines_angle (C=length,a=A,c_ang=c_ang);
    b_ang = 180 - a_angle - c_ang;
    L1 = law_sines_length (C=length,c_ang=c_ang,b_ang=b_ang);
    a_ang = (offset > 0) ? a_angle : -a_angle;
    c_angle = (offset > 0) ? c_ang : -c_ang;
    //echo("AB",offset=offset,a_ang=a_ang,b_ang=b_ang,L1=L1);
    rotate([0,0,a_ang]) {
        
        // collection of operations for the first leg
        difference () {
            union() {
                simple_link (l=L1,w=w,t=w,d=0,cored=25.5); 
                // add the servo horn pad
                rotate([0,0,-90]) translate([0,widthAB*.25,widthAB-a_svo_boss/2])
                    rounded_cube(size=[widthAB,widthAB*1.5,a_svo_boss],r=widthAB/2,center=true);
                // add a small cube for the spring
                translate([30,-widthAB/2.3,5]) cube(10);
            }
            // remove the servo horn shape
            translate([0,0,widthAB-a_svo_boss/2-1]) servo_horn (vis=false);
            
            // remove screw holes, used to hold two halfs together
            translate([0,0,w]) {
                hole_pair (x = length*.13,y=w*0.8,d=hole_M3,h=w);
                hole_pair (x = length*.34,y=w*0.8,d=hole_M3,h=w);
                hole_pair (x = length*.6,y=w*0.8,d=hole_M3,h=w);
                hole_pair (x = length*.8,y=w*0.8,d=hole_M3,h=w);
            }
        }
        // collection of operations for the second leg
        translate([L1,0,0]) rotate([0,0,c_angle-180]) 
            simple_link (l=A,w=w,t=w,d=0,cored=0); 
    }
}

module ABarm (Assy=true,Part1=true) {
    // TWO PART arm, assembled with screws
    $fn=$preview ? 64 : 128; // minimum angle fragment

    color("plum",1) difference () {
        AB_offset_link (length=lenAB,w=widthAB,offset=-widthAB/2.25,d_pin=hole_M6,$fn=48);
            
        // remove joint A bore and donut
        cylinder(h=5*widthAB,d=M6_bearing_od,center=true);
        translate([0,0,widthAB/2])
            filled_donut(t=widthAB*.55,d=widthAB*1.2,r=widthAB*.1);
        
        // remove the spring leg hole
        translate([0,0,22]) rotate([0,180,90-A_Tspr_rot])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        
        // remove joint B bore and end donut
        translate([lenAB,0,widthAB/2]) {
            cylinder(h=4*widthAB,d=hole_M6,center=true);
            filled_donut(t=widthAB*.75,d=widthAB*1.7,r=widthAB*.1);
        }
        //  BIG CUBES TO MAKE TWO PARTS
        split_offset=10.9;
        // USE ONE OR THE OTHER
        if (Part1) {
            translate([0,0,widthAB+split_offset]) cube([lenAB*4,lenAB*2,widthAB],center=true);
        } else {
            translate([0,0,split_offset+.2]) cube([lenAB*4,lenAB*2,widthAB],center=true);
        }
   }
   if (Assy) {
        rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
        
        translate([0,0,wAB_inside]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
        
        // torsion spring at A
        translate([0,0,22]) rotate([0,180,90-A_Tspr_rot]){
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH);
            torsion_spring_spacer();
           }
    }
}
*ABarm(Assy=false,Part1=true); // FOR_PRINT
*ABarm(Assy=false,Part1=false); // FOR_PRINT

module BC_offset_link (length=50,w=15,offset=7,d_pin=5) {
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,0], Second joint is at [length,0]
    // link width is w (y dimension), constant along length of part
    // offset is the length of the second. IF NEGATIVE IT REVERSES THE SHAPE
    // Method of making a dog leg link, using law of sines:
    c_ang=135;
    A = (offset > 0) ? offset : -offset;
    a_angle = law_sines_angle (C=length,a=A,c_ang=c_ang);
    b_ang = 180 - a_angle - c_ang;
    L1 = law_sines_length (C=length,c_ang=c_ang,b_ang=b_ang);
    a_ang = (offset > 0) ? -a_angle : a_angle;
    c_angle = (offset > 0) ? c_ang : -c_ang;
    //echo("BC",offset=offset,a_ang=a_ang,b_ang=b_ang,L1=L1);
       
    // collection of operations for the first leg
    difference () {
        union() {
            // collection of operations for the first leg
            rotate([0,0,180-c_angle]) 
                    simple_link (l=A,w=w,t=w,d=0,cored=0); 
            
            // second leg, the long part
            translate([length,0,0]) {
                translate([0,0,w-4]) cylinder(h=8,d=w/2,center=true); // for the bolt
                
                rotate([0,0,a_ang]) 
                translate([-L1,0,0]) {
                    difference() {
                        simple_link (l=L1,w=w,t=w,d=0,cored=32); 
                        // remove a cube core, for better printing
                        //w_scale = w*.8;
                        //translate([0,-w_scale/2,(w-w_scale)/2]) 
                            //cube([length*2,w_scale,w_scale],center=false);
                        // remove screw holes, used to hold two halfs together
                        hole_pair (x = length*.14,y=w*0.9,d=hole_no6_screw,h=w*.7);
                        hole_pair (x = length*.34,y=w*0.9,d=hole_no6_screw,h=w*.7);
                        hole_pair (x = length*.57,y=w*0.9,d=hole_no6_screw,h=w*.7);
                        hole_pair (x = length*.78,y=w*0.9,d=hole_no6_screw,h=w*.7);
                    }
                    // add the servo horn pad
                    translate([L1-widthAB/4,0,0]) rotate([0,0,90])
                    rounded_cube(size=[widthAB,widthAB*1.5,8],r=widthAB/2,center=true);
                }
            }
        }
    }
}
*BC_offset_link (length=lenBC,w=widthAB,offset=-widthAB/2.25,d_pin=hole_M6,$fn=48);

module BCarm (Assy=false,Part1=false) {
    // TWO PART arm, assembled with screws
    $fn=$preview ? 64 : 128; // minimum angle fragment
    
    hex_h = AB_pulley_t;  // height offset for hex

    color("lightblue",1) difference () {
        union () {
            BC_offset_link (length=lenBC,w=widthAB,offset=-widthAB/2.25,d_pin=hole_M6,$fn=48);

            // union HEX for pulley
            translate ([ 0,0,-hex_h+M6_bearing_flange_t])
                hex (size=22.86,l=hex_h);
            
        }
        // c-bore for B bearing
        cylinder(h=3*widthBC,d=M6_bearing_od,center=true);
        
        // remove donut at joint C
        translate([0,0,widthAB/2])
            filled_donut(t=widthAB*.8,d=widthAB*2,r=widthAB*.05);
        
        // remove joint B bore and end donut
        translate([lenBC,0,widthAB/2]) {
            cylinder(h=4*widthAB,d=hole_qtr_inch,center=true);
            filled_donut(t=widthAB*.8,d=widthAB*1.4,r=widthAB*.1);
        }

        // subtract C servo interface
        translate([lenBC,0,svo_flange_d-4])
            rotate([0,0,0])
                servo_body (vis=false);

        //  BIG CUBES TO MAKE TWO PARTS, FOR PRINTING
        split_offset=-16.1;
        if (Part1) {
            translate([0,0,widthAB+split_offset]) cube([lenAB*4,lenAB*2,widthAB],center=true);
        } else {
            translate([0,0,split_offset+.2]) cube([lenAB*4,lenAB*2,widthAB],center=true);
        }
    } 
    if (Assy) {
       translate([0,0,wall_t]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
       translate([0,0,-AB_pulley_t+Qtr_bearing_flange_t]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
       translate([0,0,wBC_inside+2*wall_t]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
        
        rotate ([180,0,0]) B_Pulley_at_B ();
    }
}
*BCarm (Assy=false,Part1=true); // FOR_PRINT
*BCarm (Assy=false,Part1=false); // FOR_PRINT

module B_Pulley_at_B () {
    // Pulley at Joint B
    color ("green") difference () {
       pulley_gt2_2 ( teeth = AB_pulley_teeth , pulley_t_ht = AB_pulley_t);
        // add hex bore
       hex (size=22.94,l=AB_pulley_t+.5);
    }
}
*B_Pulley_at_B (); // FOR_PRINT

module BA_Pulley (partA=true) {
    // This pulley is on the outside of the AB arm at A    
    // CREATE TWO .STL PARTS BY DISABLING * PART A OR B
    $fn=$preview ? 64 : 128; // minimum angle fragment
    
    pulley_OD = tooth_spacing (AB_pulley_teeth,2,0.254) +2;
    
    difference () {
        union () {
            // PART A, side with the pulley
            if (partA) { 
                pulley_gt2_2(teeth=AB_pulley_teeth,pulley_t_ht=AB_pulley_t ,motor_shaft=hole_qtr_inch);
                translate([0,0,13]) {
                    difference(){
                        cylinder(h=6,d=pulley_OD,center=true); // big boss A side
                        // Positive B rotation side
                        rotate([0,0,-16]) translate([-30,-40,-10]) cube([40,40,20],center=false);
                        // Negative B rotation side
                        rotate([0,0,-40]) translate([-48,-11,-10]) cube([40,40,20],center=false);
                    }
                    translate([0,0,2]) cylinder(h=2,d=pulley_OD/1.5,center=true); // big boss A side
                }
            } else {
                // PART B, side with the servo horn block
                translate([0,0,-3]) 
                    cylinder(h=6,d=pulley_OD,center=true); // big boss B side
                translate([0,0,-10]) 
                    rotate([0,0,-B_Horn_Zero_Angle]) {
                        translate([25,0,-0.4]) cube([15,16,20.8],center=true);
                    }
                }
            }
        // remove the servo horn shape and screw hole for holding screw
        rotate([0,0,-B_Horn_Zero_Angle]) translate([0,0,-21]) {
            servo_horn(vis=false);
        }
        // remove the spring, DONT INCLUDE WITH PART A
        if (!partA) translate([0,0,-2]) rotate([0,0,B_Horn_Zero_Angle-90]) // ADJUSTMENT ANGLE
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd*1.05,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        // remove the A hole!
        cylinder(h=2*shoulder_w,d=M6_bearing_od,center=true);
        // remove the SPRING BORE
        translate([0,0,4.5]) cylinder(h=9271K619_t*1.22,d=9271K619_OD,center=true);
        // remove the screw holes that hold the two parts together
        translate([0,0,-4.1]) rotate([0,0,10])
            Rotation_Pattern(number=4,radius=16)  
                Cbore_Screw_Hole(d=2.8,h=17,cb_d=7,cb_h=2);
    }
}
*BA_Pulley(partA=true);// FOR_PRINT
*translate([0,0,-10]) BA_Pulley(partA=false);// FOR_PRINT

module BA_Pulley_Assy () {
    // This pulley is on the outside of the AB arm at A
    BA_Pulley(partA=true);
    BA_Pulley(partA=false);
    // Bearings
   translate([0,0,-6]) rotate([180,0,0]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
   translate([0,0,AB_pulley_t+6]) Bearing_Flanged (t=M6_bearing_t,flange_t=M6_bearing_flange_t,od=M6_bearing_od,id=hole_M6,flange_od=M6_bearing_flange_od);
    //
}
*BA_Pulley_Assy ();  // Assembly, do not print

module C_horn(){
    // This is the arm end-effector interface
    // center is 0,0,0 on xy plane, length is +x
    $fn=$preview ? 64 : 128; // minimum number of fragements
    End_w = 10; // End effector interface width, mm
    offx = 1;
    t_at_C=wBC_inside-2*Qtr_bearing_flange_t-offx; // thickness at C
    
    difference () {
        union () { 
            translate([0,0,offx/2]) cylinder (t_at_C,d=30,center=true);
            ear(thk=End_w/2);  // ear without horn
            mirror ([0,0,1]) ear(thk=End_w/1.145) ;// horn ear
        }
        // remove bore bearing
        cylinder(2*t_at_C,d=Qtr_bearing_od,center=true);
        translate([0,0,-8]) cylinder(h=14,d=Qtr_bearing_flange_od+4,center=true);
        
        // remove the servo horn
        translate([0,0,-t_at_C/2-.01]) servo_horn(vis=false);
        
        // remove End attach pin
        translate ([claw_radius*1.5,0,0])
            hole_pair (x = 0,y=claw_height*0.7,d=hole_M3,h=100);
    }
    module ear (thk=1) {
        End_angle = atan2(0,50.8 - hole_qtr_inch*1.5);
        translate ([0,-claw_height/2,End_w/2]) 
            rotate([0,0,End_angle])
            cube([claw_radius*2,claw_height,thk],center=false);
    }
}
*C_horn();// FOR_PRINT

module claw_servo_bracket() {
    $fn=$preview ? 64 : 128; // minimum number of fragements
    servo_plate_t = 8;
    shim_t = 7;
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
            rotate([0,90,-90]) servo_body(vis=false,$fn=32);
        
        // remove the screw holes
        translate ([-servo_plate_l/2-4,0,0]) rotate([90,0,0]) 
            hole_pair (x = 0,y=claw_radius*1.7,d=hole_M3,h=100);
        
        translate([-38,0,0]) rotate([90,0,0]) cylinder(h=100,d=hole_M3,center=true);
    }
}
*claw_servo_bracket();// FOR_PRINT

module GoPro_bracket() {
    //$fn=60;
    color("blue") {
        difference () {
            rounded_cube([20,60,6],r=4,center=true);
            // remove the screw holes
            hole_pair (x = 0,y=claw_radius*1.7,d=hole_M3,h=100);
            // remove counter bore
            translate([0,0,-4]) hole_pair (x = 0,y=claw_radius*1.7,d=6,h=4);
            
            // remove a zip tie grouve
            translate([0,0,2]) cube([30,6,4],center=true);
        }
        translate([-9,0,-4]) cube([2,50,2],center=true);
    }
}
*GoPro_bracket();// FOR_PRINT

module Claw(assy=true){
    // DRAW THE COMPLIANT CLAW
    $fn=$preview ? 32 : 64; // minimum angle fragment

    servo_plate_t = 8;
    back_plate_w = claw_width - 4*claw_radius;
    shim_t = 7;
    difference () {
        translate([claw_radius,-claw_height/2,0]) 
            rotate([0,-90,-90])     
                compliant_claw2 (len=claw_length,width=claw_width,t1=claw_t,t2=claw_height,r=claw_radius,pre_angle=15);
        // remove attach pins (screw holes)
        translate ([claw_radius/2,0,0])
            hole_pair (x = 0,y=claw_height*0.7,d=hole_M3,h=100);
        // remove the screw holes
        translate ([2*claw_radius,0,0]) rotate([90,0,0]) 
            hole_pair (x = 0,y=claw_radius*1.7,d=hole_M3,h=100);
        translate([2*claw_radius-6,0,0]) rotate([90,0,0]) cylinder(h=100,d=hole_M3,center=true);
    }
    if (assy) {
        claw_servo_x = 2*claw_radius+32;
        // Remove Servo Bracket for claw print
        color("green") translate([claw_servo_x,claw_height/2+shim_t,0]) claw_servo_bracket();
        
        // Remove Servo for claw print
        color ("red",.7) translate([claw_servo_x,svo_flange_d+shim_t,0]) 
            rotate([0,90,-90]) servo_body();
        // GoPro
        *translate([20,-66,-30]) rotate([90,0,95]) GoPro_model();
        translate([35,-16,0]) rotate([-90,0,0]) GoPro_bracket();
    }
}
*Claw(assy=false); //FOR_PRINT

module shoulder_servo_lug(h_guss=10,thk=2) {
    $fn=$preview ? 64 : 128; // minimum number of fragements
    
    difference () {
        total_h = shoulder_z_top + shoulder_t + 12;
        lug (r=x_guss*1.2,w=shoulder_w/1.3,h=total_h,t=thk,d=0);    
        translate([0,shoulder_z_top+ shoulder_t,svo_flange_d])
            rotate([0,0,90])
                servo_body (vis=false);
        
        hole_space1 = shoulder_t+h_guss/2;
        hole_shift1 = shoulder_t+h_guss/4;

        translate([x_guss+t_guss/2,hole_shift1,0]) hole_pair (x = 0,y=hole_space1,d=hole_no6_screw,h=20);
        translate([-(x_guss+t_guss/2),hole_shift1,0]) hole_pair (x = 0,y=hole_space1,d=hole_no6_screw,h=20);
    }
}

module shoulder_lug(h_guss=10,thk=2) {
    $fn=$preview ? 64 : 128; // minimum number of fragements
    difference() {
        lug (r=x_guss,w=x_guss*2,h=shoulder_z_top,t=thk,d=hole_M6);
        translate([0,h_guss/2,thk/2]) 
            rotate([0,90,0]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=x_guss*3);
   }
}

module shoulder_plate(part_A=true,h_guss=10,shoulder_y_A=0,shoulder_y_1=20,shoulder_y_2=40,shoulder_y_B=60) {
    // Assembly of the shoulder of the arm
    // MAKE STL TWO PARTS (A and B). 
    // XY = HORIZON
    $fn=$preview ? 64 : 128; // minimum number of fragements
    // parameters for the 4 shoulder attach bolts to the bearing
    shoulder_bearing_hole_space = 59;
    x_bs = shoulder_bearing_hole_space/2;
    y_bs = shoulder_bearing_hole_space/2;
    // parameters for the 4 gear attach screws
    hole_space_gear = 38;
    x_g = hole_space_gear/2;
    y_g = hole_space_gear/2;

    difference () {
        union () {
            if (part_A) {
            // PART A
                // shoulder plate
                translate([0,shoulder_l/2,- shoulder_t/2])
                    rounded_cube(size=[shoulder_w,shoulder_l,shoulder_t],r=10,center=true);
                // shoulder gussets
                translate([x_guss,shoulder_y_A,0])
                    cube([t_guss,shoulder_l,h_guss],center=false);
                translate([-x_guss-t_guss,shoulder_y_A,0])
                    cube([t_guss,shoulder_l,h_guss],center=false);
            } else {
                // PART B
                color("blue") translate([0,shoulder_y_shift,- shoulder_t - 5])
                    cylinder(h=10,d=60,center=true);
            }
        }
        // subtract the wire hole
        translate([0,shoulder_y_shift,0])
                cylinder(h=shoulder_t*5,d=15,center=true);
        
        // subtract the 4 shoulder mounting bolt holes
        translate([x_bs,y_bs+shoulder_y_shift,0])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        translate([x_bs,-y_bs+shoulder_y_shift,0])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        translate([-x_bs,-y_bs+shoulder_y_shift,0])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        translate([-x_bs,y_bs+shoulder_y_shift,0])
                cylinder(h=shoulder_t*3,d=2.5,center=true);
        
        // subtract the 4 gear mounting screw holes
        translate([0,y_g+shoulder_y_shift,-12])
                cylinder(h=shoulder_t*4,d=hole_no6_screw,center=true);
        translate([x_g,shoulder_y_shift,-12])
                cylinder(h=shoulder_t*4,d=hole_no6_screw,center=true);
        translate([-x_g,shoulder_y_shift,-12])
                cylinder(h=shoulder_t*4,d=hole_no6_screw,center=true);
        translate([0,-y_g+shoulder_y_shift,-12])
                cylinder(h=shoulder_t*4,d=hole_no6_screw,center=true);
        
        // screw holes long ways in gussets
        hole_space1 = shoulder_t+h_guss/2;
        hole_shift1 = h_guss/4;
        translate([x_guss+t_guss/2,3,hole_shift1]) 
            rotate([90,0,0]) hole_pair (x = 0,y=hole_space1,d=hole_no6_screw,h=10);
        translate([x_guss+t_guss/2,shoulder_y_B-10,hole_shift1]) 
            rotate([90,0,0]) hole_pair (x = 0,y=hole_space1,d=hole_no6_screw,h=10);
        translate([-(x_guss+t_guss/2),3,hole_shift1]) 
            rotate([90,0,0]) hole_pair (x = 0,y=hole_space1,d=hole_no6_screw,h=10);
        translate([-(x_guss+t_guss/2),shoulder_y_B-10,hole_shift1]) 
            rotate([90,0,0]) hole_pair (x = 0,y=hole_space1,d=hole_no6_screw,h=10);
            
        // screw holes perpendicular to gussets
        translate([0,shoulder_y_1-4,h_guss/2]) 
            rotate([90,0,90]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=x_guss*4);
        translate([0,shoulder_y_2-4,h_guss/2]) 
            rotate([90,0,90]) hole_pair (x = 0,y=h_guss/2,d=hole_no6_screw,h=x_guss*4);
        
        // subtract the B pulley arm torsion spring leg hole
        translate([0,shoulder_y_1+9271K619_t+4,shoulder_z_top]) rotate([90,180,0])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
        // subtract the AB torsion spring leg hole
        translate([0,shoulder_y_2-8,shoulder_z_top]) rotate([90,180-A_Tspr_rot,0])
            torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH,inverse=true);
    }
}
h_gusset = 18; // Gusset height
shoulder_y_A = 0; // A servo position
shoulder_y_1 = 18;
shoulder_y_2 = 72;
shoulder_y_B = 95; // B servo length, was 93

*shoulder_servo_lug(h_guss=h_gusset,thk=shoulder_t);//FOR_PRINT

*shoulder_lug(h_guss=h_gusset,thk=shoulder_t);//FOR_PRINT

// Part A
*shoulder_plate(part_A=true,h_guss=h_gusset,shoulder_y_A=shoulder_y_A,shoulder_y_1=shoulder_y_1,shoulder_y_2=shoulder_y_2,shoulder_y_B=shoulder_y_B);// FOR_PRINT

// Part B
*shoulder_plate(part_A=false,h_guss=h_gusset,shoulder_y_A=shoulder_y_A,shoulder_y_1=shoulder_y_1,shoulder_y_2=shoulder_y_2,shoulder_y_B=shoulder_y_B);// FOR_PRINT

module shoulder_assy (h_guss=h_gusset,shoulder_y_A=shoulder_y_A,shoulder_y_1=shoulder_y_1,shoulder_y_2=shoulder_y_2,shoulder_y_B=shoulder_y_B) {
    // Assembly of the shoulder of the arm
    // XY = HORIZON
    $fn=$preview ? 64 : 128; // minimum number of fragements

    translate([0,0,-shoulder_z_top]) {
    shoulder_plate(part_A=true,h_guss=h_gusset,shoulder_y_A=shoulder_y_A,shoulder_y_1=shoulder_y_1,shoulder_y_2=shoulder_y_2,shoulder_y_B=shoulder_y_B);
    shoulder_plate(part_A=false,h_guss=h_gusset,shoulder_y_A=shoulder_y_A,shoulder_y_1=shoulder_y_1,shoulder_y_2=shoulder_y_2,shoulder_y_B=shoulder_y_B);
    }
    
    // A Servo support (lug)
    color("yellow") translate([0,shoulder_y_A-shoulder_t,-shoulder_z_top-shoulder_t])
        rotate([90,0,180]) 
            shoulder_servo_lug(h_guss=h_gusset,thk=shoulder_t);    
    // First lug
    color("lime") translate([0,shoulder_y_1,-shoulder_z_top])
        rotate([90,0,0]) 
            shoulder_lug(h_guss=h_gusset,thk=shoulder_t);    
    // Second lug
    color("turquoise") translate([0,shoulder_y_2,-shoulder_z_top])
        rotate([90,0,0]) 
            shoulder_lug(h_guss=h_gusset,thk=shoulder_t);
    // B Servo support (lug)
    color("blueviolet") translate([0,shoulder_y_B,-shoulder_z_top-shoulder_t])
        rotate([90,0,0]) 
            shoulder_servo_lug(h_guss=h_gusset,thk=shoulder_t);    
}
*shoulder_assy ();// not for print

module zip_loop() {
    translate([0,0,-12]) rotate([90,0,0]) linear_extrude(4) 
        U_section(Lbase=14,Lleg=8,Tbase=4,Tleg=4);
}
*zip_loop();// not for print

module Electronics_Board (Assy=true) {
    $fn=$preview ? 32 : 64; // minimum number of fragements
    board_l = 170;
    board_w = 140;
    board_t = 4;
    board_shift = 50;
    y_w = (board_w-20)/2;
    x_w = (board_l-20)/2;
    difference() {
        translate([board_shift,0,-2.1]) rounded_cube([board_l,board_w,board_t],r=10,center=true);
        Power_Energy_Meter();
        translate([50,-30,0]) scale([1.01,1.01,1]) Rocker_Switch();
        *translate([80,30,-8]) rotate([0,0,90]) Current_Shunt();
        
        // subtract the 4 2X4 screw mounting holes
        translate([x_w+board_shift,y_w,0]) cylinder(h=board_t*3,d=3,center=true);
        translate([x_w+board_shift,-y_w,0]) cylinder(h=board_t*3,d=3,center=true);
        translate([-x_w+board_shift,-y_w,0])cylinder(h=board_t*3,d=3,center=true);
        translate([-x_w+board_shift,y_w,0]) cylinder(h=board_t*3,d=3,center=true);
        
        // arduino board holes (see arduino.scad)
        // off for thingiverse, purchased part
        *translate([140,10,10]) rotate([180,0,-90]) holePlacement()
            union() {
                cylinder(d=4, h = board_t*3, $fn=32);
              };
       translate([125,40,-1]) rotate([0,0,-180]) linear_extrude(height = 1) 
           text("SACC MAKE 2", font = "Arial", size = 10);
    }
    // zip tie loops for the current shunt
    translate([60,40,0]) zip_loop();
    translate([80,40,0]) zip_loop();
    // zip tie loops for the arduino wires
    translate([85,18,0]) zip_loop();
    translate([85,-47,0]) zip_loop();
    // zip tie loops for other wires
    translate([40,10,0]) zip_loop();

    if (Assy) {
        Power_Energy_Meter();
        translate([50,-30,0]) Rocker_Switch();
        translate([80,40,-12]) rotate([0,0,90]) Current_Shunt();
        // off for thingiverse, purchased part
        *translate([140,10,-8]) rotate([180,0,-90]) arduino(); 
    }
}
*Electronics_Board(Assy=false);//FOR_PRINT

module base_assy() {
    $fn=$preview ? 64 : 72; // minimum angle fragment
    // Base plate size
    base_x = 130;  // base width
    base_y = 150;  // base length
    base_t = 10;
    // location for the holes that screw the base to the wood
    x_w = (base_x-20)/2;
    y_w = (base_y-20)/2;
    // parameters for the 4 base attach bolts to the bearing
    base_bearing_hole_space = 77;
    x_bb = base_bearing_hole_space/2;
    y_bb = base_bearing_hole_space/2;
    
    difference() {
        union() {
         rounded_cube(size=[base_x,base_y,base_t],r=12,center=true);
            translate([base_x/2-15,0,-svo_flange_d/2]) 
            cube([svo_flange_l*1.2,svo_w*1.4,svo_flange_d+base_t],center=true);
        }
        // big middle hole
        cylinder(h=shoulder_t*2.,d=94,center=true); 
        
        // hole for small gear on servo
        translate([(64+32)/2*(2.54/3.14159)-gear_space_adjustment,0,1]) 
            cylinder(h=shoulder_t*2.1,d=30,center=true);
        
        // the distance between gears is the teeth*pitch/pi
        translate([(64+32)/2*(2.54/3.14159)-gear_space_adjustment,0,-3]) 
            rotate([0,0,180]) 
            servo_body(vis=false);  // shoulder servo
        
        // subtract the 4 bearing mounting screw holes
        translate([x_bb,y_bb,0]) cylinder(h=base_t*3,d=2.5,center=true);
        translate([x_bb,-y_bb,0]) cylinder(h=base_t*3,d=2.5,center=true);
        translate([-x_bb,-y_bb,0])cylinder(h=base_t*3,d=2.5,center=true);
        translate([-x_bb,y_bb,0]) cylinder(h=base_t*3,d=2.5,center=true);
        
        // subtract the 4 2X4 screw mounting holes
        translate([x_w,y_w,0]) cylinder(h=base_t*3,d=4,center=true);
        translate([x_w,-y_w,0]) cylinder(h=base_t*3,d=4,center=true);
        translate([-x_w,-y_w,0])cylinder(h=base_t*3,d=4,center=true);
        translate([-x_w,y_w,0]) cylinder(h=base_t*3,d=4,center=true);
    }
    // Representation of bearing
    // 4 inch Lazy Susan Heavy Duty Aluminium Alloy Rotating Bearing
    translate([0,0,9+base_t/2]) Bearing(t=9,od=120,id=70); 
    
    // Representation of 2x4 wood
    l_wood = 400;
    color("Khaki") {
        translate([-base_x/2,base_x/2-10,-base_t/2]) rotate([0,90,0]) cube([3.5*25.4,1.5*25.4,l_wood]);
        translate([-base_x/2,-base_x/2+10-1.5*25.4,-base_t/2]) rotate([0,90,0]) cube([3.5*25.4,1.5*25.4,l_wood]);
    }
    
    // Representation of electronics board
    translate([280,0,0]) rotate([0,0,180])Electronics_Board();
}
*base_assy(); // not for print

module base_and_shoulder_assy(base_ang=0,A_angle=0,B_angle=0){
    base_z_top = -62; // base offset down from zero, mm
    // shoulder, adjust z translation as required
    rotate([-90,base_ang,0]) 
        translate([0,-shoulder_y_shift,0]) { 
            color("green",.5) shoulder_assy (); 
            // B Servo (was A)
            color ("red",.5) rotate([-90,-90,0])
                translate([0,0,shoulder_y_A+shoulder_t/2]) {
                    servo_body();
                    rotate([0,0,-B_angle+90-B_Horn_Zero_Angle])
                        servo_horn(vis=true);
                }
            // A Servo  (was B)
            color ("darkred",.5) 
                translate([0,shoulder_y_B-svo_flange_d,0]) 
                rotate([90,-90,0]) {
                    servo_body();
                    rotate([0,0,A_angle-90])
                        servo_horn(vis=true);
                }
        }
    // sholder big gear  64 tooth
    // off for thingiverse, purchased part
    translate([0,-1.42*shoulder_z_top,0]) rotate([90,0,0]) 64T_32P_Actobotics();
    // the distance between gears is the teeth*pitch/pi
    color ("red",.5) 
        translate([-(64+32)/2*(2.54/3.14159)+gear_space_adjustment,base_z_top-2,0])
            rotate([-90,0,0]) {
                servo_body();  // shoulder servo
                // off for thingiverse, purchased part
                32T_32P_Actobotics();   // servo gear 32 tooth
    }
        
    // Base
    translate([0,base_z_top,0]) rotate([-90,180,0]) base_assy();
}
*base_and_shoulder_assy(base_ang=TT,A_angle=AA,B_angle=BB); // not for print

module end_effector_assy() {
    color("SpringGreen") C_horn (); 
    rotate([180,0,0]) translate([claw_radius,0,0]) Claw();
}
*end_effector_assy(); // not for print

module BC_arm_and_End (C_angle=0) {
    
    BCarm (Assy=true,Part1=true);
    BCarm (Assy=false,Part1=false);
    
    // Draw the end effector
    translate([lenBC,0,0]) {
        
        translate([0,0,6])color ("red",.5) servo_body();  // C Servo
        
        rotate ([0,0,-C_angle]) {
            translate([0,0,widthBC/2]) end_effector_assy();
            translate([0,0,6])color ("darkred",.5) servo_horn(vis=true);
        }
    }
}
*BC_arm_and_End (C_angle=CC); // not for print

module draw_assy (A_angle=0,B_angle=0,C_angle=0,base_ang=0) {
    // XZ = HORIZON
    // calculate b and c positions from angles
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  // B location
    c = [(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),0];
    // lower arm vector
    //vecAB=[b[0]/lenAB,b[1]/lenAB,b[2]/lenAB];
    rotate([0,base_ang,0]) {
        translate([0,0,5]) {     // There are a few unfortunate fixed rotations 
            // draw the upper and lower arms
            rotate([0,0,A_angle]) {
                // Draw the AB link
                translate ([0,0,-1]) rotate([180,0,0]) {
                    ABarm (Assy=true,Part1=true);
                    ABarm (Assy=false,Part1=false);
                }
                // Draw the BC link and End
                translate([lenAB,0,5]) {
                    rotate([0,0,B_angle-A_angle]) rotate([180,0,0]) BC_arm_and_End(C_angle);
                    // B joint 6 MM shaft,  LENGTH = 70 MM
                    translate([0,0,-widthAB/2]) cylinder(h=70,d=5.5,center=true);
                }

            }
            
            // Draw the B drive pulley at A
            color("navy",0.5)  
                rotate([0,0,(B_angle)]) 
                    translate([0,0,16]) rotate([180,0,0]) BA_Pulley_Assy ();
            translate([0,0,5]) rotate([0,0,180])
        color("blue") torsion_spring (deflection_angle=9271K619_angle,OD=9271K619_OD,wire_d=9271K619_wd,leg_len=9271K619_len,coils=9271K619_coils,LH=9271K619_LH);
        color("red") torsion_spring_spacer();

                    
            // B drive belt (displayed with shoulder assembly)
            belt_z = 11;
            color("blue") 
                pt_pt_belt([0,0,belt_z],[b[0],b[1],belt_z],d=6,r_pulley=AB_pulley_d/2,round=false);
        }
    }
    // Draw Base and Shoulder assembly adjust z translation as required
    base_and_shoulder_assy(base_ang=base_ang,A_angle=A_angle,B_angle=B_angle);
    
    // A joint 6 MM shaft,  LENGTH = 65 MM
    translate([0,0,1]) cylinder(h=65,d=5.5,center=true);
} 

if (display_assy) {
    rotate([90,0,0]) difference () {
        // Draw the SACC robot arm assembly
        draw_assy(A_angle=AA,B_angle=BB,C_angle=CC,base_ang=TT);
        if (clip_yz) { // x plane cut 
            translate ([0,-lenAB*4,-lenAB*4]) cube (lenAB*8,center=false);
        }
        if (clip_xy) // z plane cut 
            // z = 38 mm is through B horn
            // z = 10 mm is through B Spring Inside leg
            // z = 0 mm shows how the AB and BC arms mesh
            translate ([-lenAB*4,-lenAB*4,-20]) cube (lenAB*8,center=false);
    }
    rotate([90,0,0]) if (clip_yz) { // Display rulers on the x plane cut
        translate ([1,-shoulder_z_top,shoulder_y_shift]) rotate([0,90,0]) ruler(100);
        translate ([1,0,shoulder_y_shift]) rotate([0,90,0]) ruler(100);
    }
}   

*rotate([90,0,0]) draw_assy (A_angle=AA,B_angle=BB,C_angle=CC,base_ang=TT); // not for print