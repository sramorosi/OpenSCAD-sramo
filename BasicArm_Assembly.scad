/*
  A VERY BASIC ARM ASSEMBLY MODEL
  */
use <Robot_Arm_Parts_lib.scad>
lenAB=195;
lenBC=240;
lenCD=200;
cyl_d = 10;
// Joint A angle
A_angle = 90; // [0:1:180]
// Joint B angle
B_angle = -90; // [-180:1:0]
// Joint C angle
C_angle = 20; // [-145:1:145]
// Turntable angle
T_angle = 30; // [-90:90]
// Number of position steps, for arrays
steps = 40;  // [1:1:100]


pts=pt_from_angles(A_angle,B_angle,C_angle,T_angle,lenAB,lenBC,lenCD);
echo(pts=pts);

invAngles = inverse_arm_kinematics(pts[0],lenAB,lenBC);
echo(invAngles=invAngles);

module draw_assy (A_angle=90,B_angle=0,C_angle=0,T_angle=0,lenAB=100,lenBC=200,lenCD=50) {
    //  A, zero = horizontal, positive rotation up
    //  B, zero = inline with AB, positive = same sense as A
    //  C, zero = inline with BC, positive = same sense as A and B
    //  T, positive = CCW looking down
    sqr = 25.4; // 1 inch = 25.4 mm
    origin=[0,0,0];
    rotate([0,0,T_angle]) {
        rotate([0,-A_angle,0]) {
            // AB link
            color("Plum",.5)  pt_pt_bar(from=origin, to=[lenAB,0,0], d=sqr);
            translate([lenAB,0,0]) {
                rotate([0,-B_angle,0]) {
                    translate([0,0,sqr]) {
                    // BC link
                    color("Blue",.5)  pt_pt_bar(from=origin, to=[lenBC,0,0], d=sqr);
                    translate([lenBC,0,0]) {
                        rotate([0,-C_angle,0])
                            translate([0,sqr,])
                            // CD end effector
                            color("Green",.5)  
                            pt_pt_bar(from=origin, to=[lenCD,0,0], d=sqr);
                    }
                }
                }
            }
        }
    }
}
draw_assy(A_angle,B_angle,C_angle,T_angle,lenAB,lenBC,lenCD);

// USE LIST COMPREHENSIONS TO FILL ARRAYS
// ELLIPSE
//c_pts = [ for (a = [0 : steps-1]) [270+100*cos(360*(a/steps)),200*sin(360*(a/steps)),180] ];
// LINE
c_pts = [ for (a = [0 : steps-1]) [200,-300+600*(a/steps),180] ];
// angle[0] = A, angle[1] = B
echo(c_pts=c_pts);
Ang_array = [ for (a = [0 : steps-1]) inverse_arm_kinematics(c_pts[a],lenAB,lenBC) ];
echo(Ang_array=Ang_array);

module draw_multi() {
    for (a = [0 : steps-1]) {
        draw_assy(Ang_array[a][0],Ang_array[a][1],C_angle,Ang_array[a][2],lenAB,lenBC,lenCD);
    }
}
*draw_multi();
