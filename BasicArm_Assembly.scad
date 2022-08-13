/*
  A VERY BASIC ARM ASSEMBLY MODEL, by SrAmo  August, 2022
  
  Contains a method to generate an array of angles from a path
  
  */
use <Robot_Arm_Parts_lib.scad>
use <force_lib.scad>  // using draw_3d_list function
s_TA=15;
s_AB=350; 
s_BC=380;
s_CG_x=180;
s_CG_y=40;
cyl_d = 10;
// alpha = global angles,  theta = local angles
// Joint A angle 
alphaA = 90; // [0:1:180]
// Joint B angle (local-between AB and BC arms)
thetaB = -90; // [-180:1:0]
// Joint C angle
alphaC = -90; // [-145:1:145]
// Joint D angle
alphaD = 20; // [-145:1:145]
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

module draw_assy (alphaA=90,thetaB=0,alphaC=0,alphaD=0,alphaT=0,s_AB=100,s_BC=200,s_CG_x=50,s_CG_y=10) {
    //  A, zero = horizontal, positive rotation up
    //  B, zero = inline with AB, positive = same sense as A
    //  C, zero = inline with BC, positive = same sense as A and B
    //  T, positive = CCW looking down
    sqr = 25.4; // 1 inch = 25.4 mm
    origin=[0,0,0];
    rotate([0,0,alphaT]) { // T
        rotate([0,-alphaA,0]) { // A
            color("Plum",.5)  pt_pt_bar(from=origin, to=[s_AB,0,0], d=sqr); // AB link
            
            translate([s_AB,0,0]) rotate([0,-thetaB,0]) { // B
                color("Blue",.5)  pt_pt_bar(from=origin, to=[s_BC,0,0], d=sqr); // BC link
                
                translate([s_BC,0,0]) rotate([0,-alphaC,0]) { // C
                    rotate([alphaD,0,0]) { // D
                        color("Green",.5) pt_pt_bar(from=origin, to=[0,0,s_CG_y], d=sqr);
                        color("SpringGreen") translate([0,0,s_CG_y]) {
                            pt_pt_bar(from=origin, to=[s_CG_x,0,0], d=sqr);
                            pt_pt_bar(from=[s_CG_x,40,0], to=[s_CG_x,-40,0], d=sqr);
                        }
                    }
                }
            }
        }
    }
}
alphaB=alphaA+thetaB;  // global B
thetaC=alphaC-alphaB;  // local C
thetaD=alphaD+alphaT;  // local D   NOT RIGHT
*draw_assy(alphaA,thetaB,thetaC,thetaD,alphaT,s_AB,s_BC,s_CG_x,s_CG_y);

// USE LIST COMPREHENSIONS TO FILL ARRAYS
// ELLIPSE/CIRCLE
pointsG = [ for (a = [0 : steps-1]) [450+r*cos(360*(a/steps)),r*sin(360*(a/steps)),150] ];
draw_3d_list(the3dlist=pointsG,size=20);
echo(pointsG=pointsG);
pointsC = [ for (a = [0 : steps-1]) clawToC(pointsG[a],alphaC=alphaC,alphaD=alphaD,s_CG_x=s_CG_x,s_CG_y=s_CG_y)];
echo(pointsC=pointsC);
// LINE
//c_pts = [ for (a = [0 : steps-1]) [200,-300+600*(a/steps),180] ];
// angle[0] = A, angle[1] = B
//echo(c_pts=c_pts);
angABT = [ for (a = [0 : steps-1]) inverse_arm_kinematics(pointsC[a],s_AB,s_BC) ];
echo(angABT=angABT);
//angCD = [ for (a = [0 : steps-1]) [alphaC-angABT[a][0]-angABT[a][1],alphaD+angABT[a][2]] ];
angCD = [ for (a = [0 : steps-1]) [alphaC-angABT[a][0]-angABT[a][1],alphaD] ];
echo(angCD=angCD);

module draw_multi() {
    for (a = [0 : steps-1]) {
        draw_assy(angABT[a][0],angABT[a][1],angCD[a][0],angCD[a][1],angABT[a][2],s_AB,s_BC,s_CG_x,s_CG_y);
    }
}
draw_multi();
