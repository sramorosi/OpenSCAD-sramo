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
use <force_lib.scad>  // using draw_3d_list function
// Distance Servo Axis is along X axis (mm) 
D_SVO_CB = 60; // [30:1:100]
// Distance Finger Base (FB) along Y axis (mm)
D_FB_Y = 35;  // [5:1:60]
// Distance from Finger Base (FB) to Distal Link along Y axis (mm)
D_FB_DL_Y = 10;  // [5:1:15]
// Distance between Servo Axis and Horn-Rod (horn Radius) (mm) 
R_HORN = 17; // [10:1:30]
// Distance between Finger Base and Rod-connector (mm) 
D_FINGER_ROD = 60; // [40:0.5:70]
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
// Bar Thickness for visulization
BAR_T = 2; // [1:1:5]

origin=[0,0,0];

module draw_assy (angClaw=90,angRod=0,claw=10,rod=5,AY=0,Y2=10,L=120) {
    // Calculate position of second "distal finger" four-bar
    M=L+0;  // M is the Distal Link length, slightly longer than the Finger
    N=Y2-4;  // N is the Distal Link Attache Hord length, slightly less than Y2
    Gamma1 = 90-angClaw;
    P = [L-Y2*cos(Gamma1),-Y2*sin(Gamma1),0]; // Point P, for IK 
    distalAngles = ik_xy(P,lenAB=N,lenBC=M);  // Call IK for angles
    echo(angClaw=angClaw,Gamma1=Gamma1,P=P,distalAngles=distalAngles);
    
    // Main-Middle Four Bar
    translate([0,AY,0]) rotate([0,0,angClaw]) { // Claw and Rod
        color("Plum",.5)  
            pt_pt_bar(from=origin, to=[L,0,0], d=BAR_T); // Claw
        
        translate([claw,0,0]) rotate([0,0,angRod]) { // Rod
            color("Blue",.5)  pt_pt_bar(from=origin, to=[rod,0,0], d=BAR_T); 
        }
    }
    
    // Distal Four Bar, rides on Main-Middle Four Bar
    translate([0,AY,0]) rotate([0,0,angClaw])
    translate([L,0,0]) rotate([0,0,distalAngles[0]-180]) {
        color("PeachPuff") {
            pt_pt_bar(from=origin, to=[N,0,0], d=BAR_T); // N
            rotate([0,0,15]) pt_pt_bar(from=origin, to=[D_DISTAL,0,0], d=BAR_T);
            }
        translate([N,0,0]) rotate([0,0,distalAngles[1]]) 
            color("Khaki") pt_pt_bar(from=origin, to=[M,0,0], d=BAR_T); // M
    }
}

// Create HR (horn-rod) joint CIRCLE of points
// LEFT CLAW
leftPtHR = [ for (a = [-90+ROT_SVO : 180/steps : 90+ROT_SVO])
    [R_HORN*cos(a)+D_SVO_CB,R_HORN*sin(a),0] ];
draw_3d_list(the3dlist=leftPtHR,size=2);
// RIGHT CLAW
rightPtHR = [ for (a = [270-ROT_SVO : -180/steps : 90-ROT_SVO])
    [R_HORN*cos(a)+D_SVO_CB,R_HORN*sin(a),0] ];
mirror([0,1,0]) draw_3d_list(the3dlist=rightPtHR,dot_color="red",size=2);

color("GREY")  pt_pt_bar(from=leftPtHR[0], to=leftPtHR[steps], d=3); // horn

// LEFT CLAW
leftAngAB = [ for (a = [0 : steps]) ik_xy(leftPtHR[a],D_FINGER_ROD,D_ROD,AY=D_FB_Y) ];
//echo(leftAngAB=leftAngAB);
leftPtRC = [ for (a = [0 : steps]) 
    [D_FINGER_ROD*cos(leftAngAB[a][0]),D_FINGER_ROD*sin(leftAngAB[a][0]),0] ];
//echo(leftPtRC,leftPtRC);
translate([0,D_FB_Y,0]) draw_3d_list(the3dlist=leftPtRC,dot_color="green",size=3);

module leftDrawMulti() {
    for (a = [0 : steps]) {
        draw_assy(leftAngAB[a][0],leftAngAB[a][1],claw=D_FINGER_ROD,rod=D_ROD,AY=D_FB_Y,Y2=D_FB_DL_Y,L=D_FINGER_DISTAL); 
    }
}
leftDrawMulti(); 

// RIGHT CLAW
rightAngAB = [ for (a = [0 : steps]) ik_xy(rightPtHR[a],D_FINGER_ROD,D_ROD,AY=D_FB_Y) ];
//echo(rightAngAB=rightAngAB);
rightPtRC = [ for (a = [0 : steps]) 
    [D_FINGER_ROD*cos(rightAngAB[a][0]),D_FINGER_ROD*sin(rightAngAB[a][0]),0] ];
//echo(rightPtRC,rightPtRC);
mirror([0,1,0]) translate([0,D_FB_Y,0]) draw_3d_list(the3dlist=rightPtRC,dot_color="yellow",size=3);

module rightDrawMulti() {
    for (a = [0 : steps]) {
        draw_assy(rightAngAB[a][0],rightAngAB[a][1],claw=D_FINGER_ROD,rod=D_ROD,AY=D_FB_Y,Y2=D_FB_DL_Y,L=D_FINGER_DISTAL); 
    }
}
mirror([0,1,0]) rightDrawMulti(); 

translate([D_SVO_CB,0,0]) color("pink") servo_body();

translate([85,-25,-25]) color("yellow") cube(50);

End_w = 10; // End effector interface width, mm
t2=25;
r=18;
// Add a cube to connect the back plate
translate([-r,-End_w/2,-t2]) cube([2*r,End_w,t2],center=false);

// add up the angle error from the Left CB to the Right CB
Ang_Error = [for (a=[0:steps]) (leftAngAB[a][0]-rightAngAB[a][0])];
AddError = add(Ang_Error);
offset_ang = AddError/steps;
range_ang = max(Ang_Error)-min(Ang_Error);
echo(offset_ang=offset_ang);
echo(range_ang=range_ang);