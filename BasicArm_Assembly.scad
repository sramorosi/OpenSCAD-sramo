/*
  A VERY BASIC ARM ASSEMBLY MODEL
  */
use <Robot_Arm_Parts_lib.scad>
lenAB=200;
lenBC=400;
LengthEnd=[100,0,0];
cyl_d = 10;
module draw_assy (A_angle=90,B_angle=0,C_angle=0,full=true) {
    origin=[0,0,0];
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  
    c=[(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),0];
    // AB link
    color("Plum",.5)  pt_pt_cylinder(from=origin, to=b, d=cyl_d);
    // BC link
    color("Blue",.5)  pt_pt_cylinder(from=b, to=c, d=cyl_d);
    // CD end effector
    color("Green",.5)  pt_pt_cylinder(from=c, to=c+LengthEnd, d=cyl_d);
}
draw_assy();