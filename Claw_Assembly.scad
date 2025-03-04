/*
  CLAW ASSEMBLY MODEL, by SrAmo  March, 2023
  Design tool for a two fingered Claw
  Both fingers are driven by a common Horn, attached to a Servo
  
  Assumes a rocker-rocker type 4-bar linkage for each finger
  Links: BASE, CLAW, HORN, ROD
  Joints: SVO (base-horn), HR (horn-rod), CB (claw-base), RC (rod-claw)
  Built on XZ plane, because the 
    
  USES LIST COMPREHENSIONS TO FILL ARRAYS
  */
use <Robot_Arm_Parts_lib.scad> // using inverse and other functions
use <ME_lib.scad>  // using draw_3d_list function
// use 140 for printing, 40 for display
FACETS = 140; // [40,140]
// Distance Servo Axis is along X axis (mm) 
D_SVO_CB = 50; // [30:1:100]
// Distance Finger Base (FB) along Y axis (mm)
D_FB_Y = 35;  // [5:1:60]
// Distance from Finger Base (FB) to Distal Link along Y axis (mm)
D_FB_DL_Y = 10;  // [5:1:15]
// Distance between Servo Axis and Horn-Rod (horn Radius) (mm) 
R_HORN = 17; // [10:1:30]
// Distance between Finger Base and Rod-connector (mm) 
D_FINGER_ROD = 50; // [40:0.5:70]
// Distance between Finger Base and Distal joint (mm) (L)
D_FINGER_DISTAL = 138; // [110:1:160]
// Distal Length (mm) 
D_DISTAL = 30; // [20:1:40]
// Rod Length, connects Horn to Finger (mm) 
D_ROD = 50; // [30:0.5:70]
// Rotation about Servo (Clocking, if used)
ROT_SVO = 0; // [-20:1:20]
// Number of position steps, for arrays
steps = 6;  // [1:1:100]
//  Block Size
BLOCK = 50; 

BAR_THK = 5;
BAR_HGT = 16;
PIN_DIA = 2.6; // this is for Spring Steel Slotted Spring Pin, 5/64" Diameter

ROD_THK = 6;
origin=[0,0,0];
// conversions
mm_inch = 1/25.4;

module rod(L=30,R=30,THK=5,HGT=10) {
    intersection() {
        arc_link(L=L,R=R,THK=THK,HGT=HGT+4);
        translate([HGT,0,-(HGT+4)/2]) rotate([0,-90,0]) 
            linear_extrude(2*HGT,convexity=10) // triangular side profile
            polygon([[0,-THK],[1.9,-THK],[1.9,1.6*THK],[2.1+HGT,1.6*THK],
            [2.1+HGT,-THK],
            [4+HGT,-THK],[4+HGT,2.5*THK],[4,L-THK],[4,2*L],[0,2*L]]);
    }
}
*color("green") rod(L=D_ROD,R=D_ROD,THK=ROD_THK,HGT=BAR_HGT); // PRINT TWO

module arc_link(L=30,R=30,THK=5,HGT=10) {
    ALPHA = asin((L/2)/R);  // half angle of the arc
    //echo(ALPHA=ALPHA);
    difference() {  // subtrack holes from link body
        union() {  // body of link
            translate([-R*cos(ALPHA),L/2,0]) rotate([0,0,-ALPHA]) 
                rotate_extrude(angle=2*ALPHA,convexity = 10,$fn=FACETS*2) {
                    translate([R,0,0]) square([THK,HGT],center=true);
                }
            translate([0,0,-HGT/2])cylinder(HGT,d=THK,$fn=FACETS);  // LUG1
            translate([0,L,-HGT/2]) cylinder(HGT,d=THK,$fn=FACETS);  // LUG2
        }
        translate([0,0,-HGT])cylinder(2*HGT,d=PIN_DIA,$fn=FACETS);  // hole1
        translate([0,L,-HGT]) cylinder(2*HGT,d=3,$fn=FACETS);  // hole2 for bushing
    }
}

module guide() {  // BLOCK GUIDE
    THK = 2;  // guide thickness
    SWP = 14;  // sweep
    linear_extrude(BAR_HGT/2,convexity=10) {
        the_poly();
        mirror([0,1,0]) the_poly();
    }
    module the_poly() {
        polygon([[0,0],[0,0.1+BAR_HGT/2],[-BAR_THK,0.1+BAR_HGT/2],
        [-BAR_THK,BAR_HGT/2.5],[-1.5*BAR_THK,BAR_HGT/2.5],
        [-1.5*BAR_THK,BAR_HGT/1.9],[0,BAR_HGT/1.3],
        [0,BLOCK/2+THK],[BLOCK/2,BLOCK/2+SWP],[BLOCK/2+1,BLOCK/2+SWP-THK],
        [THK,BLOCK/2],[THK,0]]);
    };
}
*guide(); // PRINT TWO

module claw_bar(L=100,L_ROD=20) {
    difference() { // subtract pin holes from polygon
        union() {
            translate([0,0,-BAR_HGT/2]) linear_extrude(BAR_HGT,convexity=10)
                polygon([[0,BAR_THK/1.9],[L_ROD,BAR_THK/1.5],
                [L,BAR_THK/2],[L,-BAR_THK],
                [L-2,-BAR_THK],[L-2,-BAR_THK/2],[L-2.2-BAR_HGT/2,-BAR_THK/2],
                [L-2.2-BAR_HGT/2,-BAR_THK],[L-6-BAR_HGT/2,-BAR_THK],
                [L-8-BAR_HGT/2,-BAR_THK/2],[0,-BAR_THK/1.9]]);
            cylinder(BAR_HGT,d=BAR_THK,center=true,$fn=FACETS);
        }
        translate([0,0,-25]) cylinder(50,d=PIN_DIA,$fn=FACETS);  // PIN HOLE
        translate([L_ROD,0,-25]) cylinder(50,d=PIN_DIA,$fn=FACETS); // PIN HOLE
    }
}
*claw_bar(L=130,L_ROD=D_FINGER_ROD);  // PRINT TWO

module claw_assy(L=130,L_ROD) { // Combines bar and guide, Click Together!
    color("Plum") claw_bar(L,L_ROD=D_FINGER_ROD);
    translate([L-2,-BAR_THK/2,0]) rotate([90,0,-90]) guide();
}
*claw_assy(); // DON'T PRINT

module base() {
    difference() { // cube that holds the servo
        translate([-4,-15,-26]) cube([74,30,8],center=false);
        translate([D_SVO_CB,0,-16]) servo_body(vis=false);
    }
    difference() {
        union() {
            translate([-4,-D_FB_Y,-26]) cube([8,2*D_FB_Y,38],center=false);
            translate([0,D_FB_Y,-7]) cylinder(38,d=8,center=true,$fn=FACETS);
            translate([0,-D_FB_Y,-7]) cylinder(38,d=8,center=true,$fn=FACETS);
        }
        translate([0,D_FB_Y,0]) cube([10,10,BAR_HGT+.1],center=true);
        translate([0,-D_FB_Y,0]) cube([10,10,BAR_HGT+.1],center=true);
        translate([0,D_FB_Y,-2*BAR_HGT]) cylinder(3*BAR_HGT,d=PIN_DIA,$fn=FACETS);  // hole1
        translate([0,-D_FB_Y,-2*BAR_HGT]) cylinder(3*BAR_HGT,d=PIN_DIA,$fn=FACETS);  // hole2
      rotate([0,-90,0]) {
        cylinder(h=10,d=10,center=true,$fn=FACETS);
        Rotation_Pattern(number=8,radius=0.385/mm_inch,total_angle=360) 
            cylinder(h=10,d=0.135/mm_inch,center=true,$fn=FACETS);
      }
    }
}
*base();  // PRINT

module draw_assy (angClaw=90,angRod=0,claw=10,rod=5,AY=0,Y2=10,L=120,RODS=0.9) {
    // Calculate position of second "distal finger" four-bar
    M=L+0;  // M is the Distal Link length, slightly longer than the Finger
    N=Y2-4;  // N is the Distal Link Attache Hord length, slightly less than Y2
    Gamma1 = 90-angClaw;
    P = [L-Y2*cos(Gamma1),-Y2*sin(Gamma1),0]; // Point P, for IK 
    distalAngles = ik_xy(P,lenAB=N,lenBC=M);  // Call IK for angles
    //echo(angClaw=angClaw,Gamma1=Gamma1,P=P,distalAngles=distalAngles);
    
    // Main-Middle Four Bar
    translate([0,AY,0]) rotate([0,0,angClaw]) { // Claw and Rod
         
        claw_assy(L,D_FINGER_ROD); // Claw
        
        translate([claw,0,0]) rotate([0,0,angRod]) { // Rod
            color("Blue")  rotate([0,0,-90]) rod(L=D_ROD,R=RODS*D_ROD,THK=ROD_THK,HGT=BAR_HGT);
        }
    }
    
    // Distal Four Bar, rides on Main-Middle Four Bar 
    /*
    translate([0,AY,0]) rotate([0,0,angClaw])
    translate([L,0,0]) rotate([0,0,distalAngles[0]-180]) {
        color("PeachPuff") {
            pt_pt_bar(from=origin, to=[N,0,0], d=3); // N
            rotate([0,0,15]) pt_pt_bar(from=origin, to=[D_DISTAL,0,0], d=3);
            }
        translate([N,0,0]) rotate([0,0,distalAngles[1]]) 
            color("Khaki") pt_pt_bar(from=origin, to=[M,0,0], d=3); // M
    } */
}

// Create HR (horn-rod) joint CIRCLE of points
// LEFT CLAW
leftPtHR = [ for (a = [-90+ROT_SVO : 180/steps : 90+ROT_SVO])
    [R_HORN*cos(a)+D_SVO_CB,R_HORN*sin(a),0] ];
//draw_3d_list(the3dlist=leftPtHR,size=2);
// RIGHT CLAW
rightPtHR = [ for (a = [270-ROT_SVO : -180/steps : 90-ROT_SVO])
    [R_HORN*cos(a)+D_SVO_CB,R_HORN*sin(a),0] ];
//mirror([0,1,0]) draw_3d_list(the3dlist=rightPtHR,dot_color="red",size=2);


// LEFT CLAW
leftAngAB = [ for (a = [0 : steps]) ik_xy(leftPtHR[a],D_FINGER_ROD,D_ROD,AY=D_FB_Y) ];
echo(leftAngAB=leftAngAB);
leftPtRC = [ for (a = [0 : steps]) 
    [D_FINGER_ROD*cos(leftAngAB[a][0]),D_FINGER_ROD*sin(leftAngAB[a][0]),0] ];
//echo(leftPtRC,leftPtRC);
//translate([0,D_FB_Y,0]) draw_3d_list(the3dlist=leftPtRC,dot_color="green",size=3);

module leftDrawMulti() {
    for (a = [0 : steps]) {
        draw_assy(leftAngAB[a][0],leftAngAB[a][1],claw=D_FINGER_ROD,rod=D_ROD,AY=D_FB_Y,Y2=D_FB_DL_Y,L=D_FINGER_DISTAL,RODS=-0.9); 
    }
}
*leftDrawMulti(); 

// RIGHT CLAW
rightAngAB = [ for (a = [0 : steps]) ik_xy(rightPtHR[a],D_FINGER_ROD,D_ROD,AY=D_FB_Y) ];
echo(rightAngAB=rightAngAB);
rightPtRC = [ for (a = [0 : steps]) 
    [D_FINGER_ROD*cos(rightAngAB[a][0]),D_FINGER_ROD*sin(rightAngAB[a][0]),0] ];
//echo(rightPtRC,rightPtRC);
//mirror([0,1,0]) translate([0,D_FB_Y,0]) draw_3d_list(the3dlist=rightPtRC,dot_color="yellow",size=3);

module rightDrawMulti() {
    for (a = [0 : steps]) {
        draw_assy(rightAngAB[a][0],rightAngAB[a][1],claw=D_FINGER_ROD,rod=D_ROD,AY=D_FB_Y,Y2=D_FB_DL_Y,L=D_FINGER_DISTAL); 
    }
}
*mirror([0,1,0]) rightDrawMulti(); 

// add up the angle error from the Left CB to the Right CB
Ang_Error = [for (a=[0:steps]) (leftAngAB[a][0]-rightAngAB[a][0])];
AddError = add(Ang_Error);
offset_ang = AddError/steps;
range_ang = max(Ang_Error)-min(Ang_Error);
//echo(offset_ang=offset_ang);
//echo(range_ang=range_ang);


module single_claw_assy() {
    draw_assy(-1.91,-88.05,claw=D_FINGER_ROD,rod=D_ROD,AY=D_FB_Y,Y2=D_FB_DL_Y,L=D_FINGER_DISTAL,RODS=-0.9);
    mirror([0,1,0]) draw_assy(-1.91,-88.05,claw=D_FINGER_ROD,rod=D_ROD,AY=D_FB_Y,Y2=D_FB_DL_Y,L=D_FINGER_DISTAL); 
    base(); 
    color("GREY") translate([0,0,-12]) pt_pt_bar(from=leftPtHR[0], to=leftPtHR[steps], d=4); // horn
    translate([D_SVO_CB,0,-16]) color("pink") servo_body();
}
single_claw_assy();

translate([90,-25,-25]) color("yellow") cube(BLOCK);