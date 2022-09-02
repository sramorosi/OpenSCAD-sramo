/*
  A VERY BASIC ARM ASSEMBLY MODEL, by SrAmo  August, 2022
  
  Contains a method to generate an array of angles from a path
  
  */
use <Robot_Arm_Parts_lib.scad>
use <force_lib.scad>  // using draw_3d_list function
s_TA=15;  // 15 mm
s_AB=350; 
s_BC=380;
s_CG_x=180;
s_CG_y=38;
s_CG_z=40;
cyl_d = 10;
// alpha = global angles,  theta = local angles
// Joint A angle 
alphaA = 90; // [0:1:180]
// Joint B angle (local-between AB and BC arms)
thetaB = -90; // [-180:1:0]
// Joint C angle (global)
alphaC = 30; // [-145:1:145]
// Joint D angle (local to end of arm at C)
thetaD = 0; // [-145:1:145]
// Turntable angle
alphaT = 0; // [-90:90]
// Number of position steps, for arrays
steps = 10;  // [1:1:100]
// radius for drawing circle
r=150; // [10:1:300]

/*
pts=pt_from_angles(alphaA,alphaB,alphaC,alphaT,s_AB,s_BC,s_CD);
echo(pts=pts);

invAngles = inverse_arm_kinematics(pts[0],s_AB,s_BC);
echo(invAngles=invAngles);
*/
module basicClaw(x=100,z=50) {
    color("SpringGreen") {
        pt_pt_bar(from=[0,0,0], to=[0,0,z], d=10);
        pt_pt_bar(from=[0,0,z], to=[x,0,z], d=10);
        pt_pt_bar(from=[x,0,z], to=[x,30,z], d=10);
    }
}

module clawWristAssy(x=100,y=30,z=50,alphaC=0,thetaD=0) {
    color("DarkGreen") {
        pt_pt_bar(from=[0,0,0], to=[0,-y,0], d=10);
    }
    rotate([0,alphaC,0]) translate([0,-y,0]) rotate([thetaD,0,0]) basicClaw(x,z);
}
clawWristAssy(x=s_CG_x,y=s_CG_y,z=s_CG_z,alphaC=alphaC,thetaD=thetaD);

function clawToC(G=[180,50,40],alphaC=0,thetaD=0,s_CG_x=180,s_CG_y=38,s_CG_z=40) =
// Claw pickup point is G.  Determine point C based on angles C and D.
let (pointC = [-s_CG_x,0,-s_CG_z])  // point C relative to G
let (pointD = [0,s_CG_y,0])
let (C1= rot_pt_x (pointC,thetaD))  // rotate thetaD about Gx
let (C2 = C1 + pointD)
let (C3= rot_pt_y (C2,-alphaC))  // rotate alphaC
let (thetaT=atan2(G[1]+s_CG_y,G[0]))  // expected turtable angle... TO DO, check for zeros
let (C4=rot_pt_z(C3,thetaT)) // correct for turtable
//echo(thetaT=thetaT)
[G[0]+C4[0],G[1]+C4[1],G[2]+C4[2]];

//NEWC=clawToC(G=[500,-38,300],alphaC=0,thetaD=10);
//echo(NEWC=NEWC);

module draw_assy (alphaA=90,thetaB=0,alphaC=0,thetaD=0,alphaT=0) {
    //  A, zero = horizontal, positive rotation up
    //  B, zero = inline with AB, positive = same sense as A
    //  C, zero = inline with BC, positive = same sense as A and B
    //  T, positive = CCW looking down
    sqr = 25.4; // 1 inch = 25.4 mm
    origin=[0,0,0];
    rotate([0,0,alphaT]) translate([s_TA,0,0]) { // T
        rotate([0,-alphaA,0]) { // A
            color("Plum",.5)  pt_pt_bar(from=origin, to=[s_AB,0,0], d=sqr); // AB link
            
            translate([s_AB,0,0]) rotate([0,-thetaB,0]) { // B
                color("Blue",.5)  pt_pt_bar(from=origin, to=[s_BC,0,0], d=sqr); // BC link
                
                translate([s_BC,0,0]) 
                    clawWristAssy(x=s_CG_x,y=s_CG_y,z=s_CG_z,alphaC=alphaC,thetaD=thetaD);
            }
        }
    }
}

// USE LIST COMPREHENSIONS TO FILL ARRAYS
// ELLIPSE/CIRCLE
pointsG = [ for (a = [0 : steps-1]) [470+r*cos(360*(a/steps)),r*sin(360*(a/steps)),100] ];
draw_3d_list(the3dlist=pointsG,size=20);
//echo(pointsG=pointsG);

pointsC = [ for (a = [0 : steps-1]) clawToC(pointsG[a],alphaC=alphaC,thetaD=thetaD,s_CG_x=s_CG_x,s_CG_y=s_CG_y)];
*for (a=[0:steps-1]) {
    translate(pointsC[a]) clawWristAssy(x=s_CG_x,y=s_CG_y,z=s_CG_z,alphaC=alphaC,thetaD=thetaD);
}
//echo(pointsC=pointsC);
// LINE
//c_pts = [ for (a = [0 : steps-1]) [200,-300+600*(a/steps),180] ];
// angle[0] = A, angle[1] = B
//echo(c_pts=c_pts);
angABT = [ for (a = [0 : steps-1]) inverse_arm_kinematics(pointsC[a],s_AB,s_BC,s_TA) ];
//echo(angABT=angABT);
// angCD: converts angles from global into local angles thetaD+angABT[a][2]*sin(alphaC)
angCD = [ for (a = [0 : steps-1]) [alphaC+angABT[a][0]+angABT[a][1],thetaD] ];
//echo(angCD=angCD);

module draw_multi() {
    for (a = [0 : steps-1]) {
        draw_assy(angABT[a][0],angABT[a][1],angCD[a][0],angCD[a][1],angABT[a][2]); // a,b,c,d,t
    }
}
draw_multi();
