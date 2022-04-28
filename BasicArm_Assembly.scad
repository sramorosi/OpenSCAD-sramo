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
B_angle = -90; // [-145:1:145]
// Joint C angle
C_angle = 0; // [-145:1:145]
// Turntable angle
T_angle = 30; // [-90:90]
// Number of position steps, for arrays
steps = 20;  // [1:1:100]


pts=pt_from_angles(A_angle,B_angle,C_angle,T_angle,lenAB,lenBC,lenCD);
echo(pts=pts);

invAngles = inverse_arm_kinematics(pts[0],lenAB,lenBC);
echo(invAngles=invAngles);

module draw_assy (A_angle=90,B_angle=0,C_angle=0,T_angle=0) {
    origin=[0,0,0];
    rotate([0,0,T_angle]) {
        rotate([0,-A_angle,0]) {
            // AB link
            color("Plum",.5)  pt_pt_cylinder(from=origin, to=[lenAB,0,0], d=cyl_d);
            translate([lenAB,0,0]) {
                rotate([0,-B_angle,0]) {
                    // BC link
                    color("Blue",.5)  pt_pt_cylinder(from=origin, to=[lenBC,0,0], d=cyl_d);
                    translate([lenBC,0,0]) {
                        rotate([0,-C_angle,0])
                            // CD end effector
                            color("Green",.5)  
                            pt_pt_cylinder(from=origin, to=[lenCD,0,0], d=cyl_d);
                    }
                }
            }
        }
    }
}
//draw_assy(A_angle,B_angle,C_angle,T_angle);

// USE LIST COMPREHENSIONS TO FILL ARRAYS
c_pts = [ for (a = [0 : steps-1]) [300+100*cos(360*(a/steps)),200*sin(360*(a/steps)),100] ];
// angle[0] = A, angle[1] = B
echo(c_pts=c_pts);
Ang_array = [ for (a = [0 : steps-1]) inverse_arm_kinematics(c_pts[a],lenAB,lenBC) ];
echo(Ang_array=Ang_array);

for (a = [0 : steps-1]) {
    draw_assy(Ang_array[a][0],Ang_array[a][1],C_angle,Ang_array[a][2]);
}
