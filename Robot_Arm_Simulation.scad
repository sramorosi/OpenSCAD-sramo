// Robot Arm Simulation tool
//  SACC = Servo Arm with Compliant Claw
//  Started on 3/24/2020 by SrAmo
//  last modified FEB 7 2021 by SrAmo
/*  >>>>> Introduction <<<<<

    The purpose of this tool is to:
    1) Parametrically draw the SACC 2 degree-of-freedom Robot Arm
    2) Draw the forces and torques on the arm
    3) Draw the reach envelope for the arm
    4) Animate the arm through the reach envelope
    5) Provide design guidance on setting spring rates to minimize the torques on the motors
    
### Nomenclature  ###
    
    Arm logical diagram (links connected by joints):
    Base -A- AB_arm -B- BC_arm -C- End -D- Claw
    
### Global Parameters are in the Configuration include file  ###

    Animation: use View/Animate.
    The OpenSCAD variable $t = time and ranges from 0 to .9999
    When animating the code is recompiled for each step.
    
    Parameters can be modified during animation!
*/

use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
use <Robot_Arm_Assembly.scad>
//###### USE ONE CONFIG FILE AT A TIME #######//
//include <SACC-26-Configuration.scad>
include <InputArm-Configuration.scad>

// Check to display assembly
display_assembly = true;
// Check to display all of the assembly
display_full = false;
// Check to display and echo forces and torques 
display_forces = false; 
// Check to display reach path 
display_reach = true; 
// Check to ECHO Calculations
Echo_Calculations = false;

// angle limits at the B joint. Angle is 0 when BC in line with AB
max_B_to_A = 45; // max angle of B to AB arm
min_B_to_A = -150; // min angle of B to AB arm

A_range = 200; // max angle range degrees of A servo
B_range = 200; // max angle range degrees of B servo
A_rigging = 65; // middle of A range used for rigging
B_rigging = 45; // middle of B range used for rigging
min_A=A_rigging-(A_range/2);
max_A=A_rigging+(A_range/2);

min_B=B_rigging-(B_range/2);
max_B=B_rigging+(B_range/2);

// servo control values for the limit points
lim1x = ((max_B-max_B_to_A)-min_A)/A_range;
lim1y = ((min_A+max_B_to_A)-min_B)/B_range;
lim2x = ((min_B-min_B_to_A)-min_A)/A_range;
lim2y = ((max_A+min_B_to_A)-min_B)/B_range;
// slope M and y Intercept B for limits for servo control
lim1_M = (1-lim1x)/lim1y;
lim2_M = lim2y/(1-lim2x);
lim1_B = lim1y;
lim2_B = -lim2_M*lim2x;

//echo(min_A=min_A,max_A=max_A,min_B=min_B,max_B=max_B);
//echo(lim1x=lim1x,lim1y=lim1y,lim2x=lim2x,lim2y=lim2y);
//echo(lim1_M=lim1_M,lim2_M=lim2_M,lim1_B=lim1_B,lim2_B=lim2_B);

throw_max_A = max_A; // max A back position for throw
throw_min_A = throw_max_A - 100; // min A for throw

// Angle of End Effector
C_angle = 0;

// Maximum payload weight (thing being lifted)
payload=1.5;         
// weight of end effector with no payload
end_weight=0.5;   
combined_weight = payload+end_weight;

// A spring free length
A_spr_free_len = 4.5;
// A spring rate K (force/distance)
A_spr_k = 1.4;
A_spr_pt_gnd = [0,spr_dist_ground,0];   // spring attach point on ground
// B spring free length
B_spr_free_len = 1.0; 
// B spring rate K (force/distance)
B_spr_k = 5; 
    
// optimized spring rate
optimum_A_spr_k=1*(combined_weight*lenAB/2)/((sqrt(0.751*lenAB*lenAB)-lenAB/2)*(lenAB/4));  

if (Echo_Calculations) {
    echo (min_A=min_A,max_A=max_A,A_range=(max_A-min_A));
    echo (min_B=min_B,max_B=max_B,B_range=(max_B-min_B));
    echo (end_weight=end_weight,payload=payload,combined_weight=combined_weight);
    echo (lenAB=lenAB,lenBC=lenBC);
    
    steps = 20;
    
    // USE LIST COMPREHENSIONS TO FILL ARRAYS
    angles = [ for (a = [0 : steps-1]) get_angles_from_t(a/steps,min_A,max_A,min_B,max_B)];
    // angle[0] = A, angle[1] = B
    //echo(angles=angles);

    b = [ for (a = [0 : steps-1]) [lenAB*cos(angles[a][0]),lenAB*sin(angles[a][0]),0] ];
        
    cx = [ for (a = [0 : steps-1]) get_CX(angles[a])];
    cx_min=min(cx);
    cx_max=max(cx);
    echo (cx_min=cx_min,cx_max=cx_max,x_range=cx_max-cx_min);
    cy = [ for (a = [0 : steps-1]) get_CY(angles[a])];
    cy_min=min(cy);
    cy_max=max(cy);
    echo (cy_min=cy_min,cy_max=cy_max,y_range=cy_max-cy_min);
    
    //
    // A SPRING, HELPS THE A MOTOR
    A_spr_pt_AB = [ for (a = [0 : steps-1]) [spr_dist_AB*cos(angles[a][0]),spr_dist_AB*sin(angles[a][0]),0] ];
    A_spr_length = [ for (a = [0 : steps-1]) norm(vector_subtract(A_spr_pt_AB[a],A_spr_pt_gnd)) ];
    A_spr_len_min=min(A_spr_length);
    A_spr_len_max=max(A_spr_length);
    A_spr_force = [ for (a = [0 : steps-1]) A_spr_k*(A_spr_length[a]-A_spr_free_len) ];
    A_spr_torque = [ for (a = [0 : steps-1]) A_spr_force[a]*dist_line_origin(A_spr_pt_gnd,A_spr_pt_AB[a]) ];  
    A_spr_torque_min=min(A_spr_torque);
    A_spr_torque_max=max(A_spr_torque);
    
    echo(A_spr_k=A_spr_k,optimum_A_spr_k=optimum_A_spr_k,A_spr_free_len=A_spr_free_len,A_spr_len_min=A_spr_len_min,A_spr_len_max=A_spr_len_max);
    echo (A_spr_torque_min=A_spr_torque_min,A_spr_torque_max=A_spr_torque_max);

    //
    // B SPRING, HELPS THE B MOTOR
    B_spr_pt = [ for (a = [0 : steps-1]) [B_spr_r*cos(angles[a][1]),B_spr_r*sin(angles[a][1]),0] ];
    B_spr_length = [ for (a = [0 : steps-1]) norm(vector_subtract(B_spr_pt[a],spr_pt_gnd)) ];
    B_spr_len_min=min(B_spr_length);
    B_spr_len_max=max(B_spr_length);
    B_spr_force = [ for (a = [0 : steps-1]) B_spr_k*(B_spr_length[a]-B_spr_free_len) ];
    B_spr_force_min=min(B_spr_force);
    B_spr_force_max=max(B_spr_force);
    
    B_spr_torque = [ for (a = [0 : steps-1]) B_spr_force[a]*dist_line_origin(spr_pt_gnd,B_spr_pt[a]) ];  
    //echo (B_spr_torque=B_spr_torque);
   
    A_payload_torque = [ for (a = [0 : steps-1])combined_weight*lenAB*cos(angles[a][0]) ]; 
    
    A_noload_torque = [ for (a = [0 : steps-1]) end_weight*lenAB*cos(angles[a][0]) ]; 
       
    a_torq_payload = [ for (a = [0 : steps-1]) A_spr_torque[a]-A_payload_torque[a] ];
    A_torq_payload_min=min(a_torq_payload);
    A_torq_payload_max=max(a_torq_payload);
    echo (A_torq_payload_min=A_torq_payload_min,A_torq_payload_max=A_torq_payload_max);

    a_torq_noload = [ for (a = [0 : steps-1]) A_spr_torque[a]-A_noload_torque[a] ];
    A_torq_noload_min=min(a_torq_noload);
    A_torq_noload_max=max(a_torq_noload);
    echo (A_torq_noload_min=A_torq_noload_min,A_torq_noload_max=A_torq_noload_max);

    echo(B_spr_k=B_spr_k,B_spr_free_len=B_spr_free_len,B_spr_len_min=B_spr_len_min,B_spr_len_max=B_spr_len_max);
    echo (B_spr_force_min=B_spr_force_min,B_spr_force_max=B_spr_force_max);
    
    B_torq_payload = [ for (a = [0 : steps-1]) -combined_weight*lenBC*cos(angles[a][1])+B_spr_torque[a] ];
    B_torq_payload_min=min(B_torq_payload);
    B_torq_payload_max=max(B_torq_payload);
    echo (B_torq_payload_min=B_torq_payload_min,B_torq_payload_max=B_torq_payload_max);
    //echo(B_torq_payload=B_torq_payload);

    B_torq_noload = [ for (a = [0 : steps-1]) -end_weight*lenBC*cos(angles[a][1])+B_spr_torque[a] ];
    B_torq_noload_min=min(B_torq_noload);
    B_torq_noload_max=max(B_torq_noload);
    echo (B_torq_noload_min=B_torq_noload_min,B_torq_noload_max=B_torq_noload_max);
}

// #### pt used with animation ####

//pt = [3,3,0];  // LengthEnd point, No animation. change to position arm

alphas=get_angles_from_t($t,min_A,max_A,min_B,max_B);
//alphas=throw_from_t($t,min_A,max_A,min_B,max_B);
new_end = rotZ_pt(C_angle,LengthEnd);
pt = vector_add(get_pt_from_angles(alphas),new_end);
A_angle = alphas[0];
B_angle = alphas[1];

if (display_assembly) {
    difference () {
        draw_assy (A_angle,B_angle,C_angle,full=display_full); 
        // x = 0 cut 
        //translate ([-20,-10,-10])
        //cube (20,center=false);
        // z = 0 cut 
        //translate ([-12,-20,.1])
        //cube (40,center=false);
    }
}

if (display_forces) internal_loads (A_angle,B_angle,C_angle);
    
if (display_reach) plot_limits (80); // turn off for 3d rendering
    
if (display_reach) plot_throw (40); // turn off for 3d rendering

//plot_circle (6,30,[10,10,0]);   // turn off for 3d rendering
//$vpr = [0, $t * 360,0];   // view point rotation

//$vpt = [c[0],c[1],c[2]];   // view point translation

//**************** END OF MAIN ******************

function get_CX (a) = (cos(a[0])*lenAB+cos(a[1])*lenBC);
function get_CY (a) = (sin(a[0])*lenAB+sin(a[1])*lenBC);

module internal_loads (A_angle=0,B_angle=0,C_angle=0) {
    r_pulley = end_pulley_d/2;  // end pully is in configuration file
    
    // Calculate and Draw the forces and torques 
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  
    c=[(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),0];
    
    // upper arm vector
    vecBC=[(c[0]-b[0])/lenBC,(c[1]-b[1])/lenBC,(c[2]-b[2])/lenBC];
    angle_BC = atan2(vecBC[1],vecBC[0]);  // angle for upper arm motor
    // lower arm vector
    vecAB=[b[0]/lenAB,b[1]/lenAB,b[2]/lenAB];
    // tangent points of belt to pulley
    LengthEnd_t=[c[0]-r_pulley*vecBC[1],c[1]+r_pulley*vecBC[0],c[2]];
    pulley_t1=[b[0]-r_pulley*vecBC[1],b[1]+r_pulley*vecBC[0],-0.2];
    pulley_t2=[b[0]-r_pulley*vecAB[1],b[1]+r_pulley*vecAB[0],-0.2];
    ground_t=[-r_pulley*vecAB[1],r_pulley*vecAB[0],0];
    A_spr_pt_AB = spr_dist_AB*vecAB;    
    
    // payload force on LengthEnd
    force_arrow(pt,[0,-1,0],combined_weight);
    
    // Sum moments about C to determine belt force on LengthEnd
    belt_force=combined_weight*(LengthEnd[0]/r_pulley); // ratio of distances
    //force_arrow(LengthEnd_t,-vecBC,belt_force); 
    
    // Sum forces to determine force on joint C using a force polygon
    c_vec=vector_subtract(combined_weight*[0,-1,0],belt_force*vecBC);
    c_to=vector_add(c,c_vec);  
    c_force=norm(c_vec);
    //force_arrow(c,c_vec,c_force);
    //force_arrow(b,-c_vec,c_force); // equal & opp on b
    
    // Determine torque on joint B.  Link BC is a cantilever beam.
    // The upper arm motor holds joint B in (fixed) rotation.
    // The torque is the joint c force x the distance to joint b.
    //
    // translate the c vectors to the origin for distance calc.
    c1=vector_subtract(c,b);
    c2=vector_subtract(c_to,b);
    cforce_to_b_arm=dist_line_origin([c1[0],c1[1]],[c2[0],c2[1]]);
    torque_at_B=cforce_to_b_arm*c_force;
    
    // torque of upper arm motor at A
    color("Blue",1) torque_arrow([0,0,0],B_mtr_trq);
    
    // calculate and draw B joint pulley (belt) forces
    // there are two belt forces on the B joint pulley (_t1 and _t2)
    //force_arrow(pulley_t1,vecBC,-belt_force); 
    //force_arrow(pulley_t2,-vecAB,belt_force); 
    // Do vector polygon to sum forces on pulley at B
    p_vec=vector_subtract(-belt_force*vecBC,belt_force*-vecAB);
    p_force=norm(p_vec);
    //force_arrow(b,p_vec,p_force); 
    
    // Determine torque on joint A.  
    // Link AB is like a cantilever beam with multiple forces on it (1,2,3)
    // 1) calculate and draw torque due to pulley force at B
    p_end = vector_add(b,p_vec);
    pforce_to_A_arm=dist_line_origin([b[0],b[1]],[p_end[0],p_end[1]]);
    p_torque_at_A=(abs(p_force) > 0.01) ? -p_force*pforce_to_A_arm : 0 ;
    
    // 2) calculate and draw torque due to upper arm force (uaf) at B
    uaf_end = vector_add(b,-c_vec);
    uaf_to_A_arm=dist_line_origin([b[0],b[1]],[uaf_end[0],uaf_end[1]]);
    uaf_torque_at_A=-c_force*uaf_to_A_arm;
     
    // 3) Spring to assist the lower arm motor
    // To minimize the torque on the lower arm we add a spring
    //   between ground and the AB arm.
    //   The spring has the least force when AB arm is vertical
    A_spr_len = norm(A_spr_pt_AB-A_spr_pt_gnd); 
    A_spr_force = A_spr_k*(A_spr_len-A_spr_free_len);
    force_arrow(A_spr_pt_AB,A_spr_pt_gnd-A_spr_pt_AB,A_spr_force);
    A_spr_to_origin=dist_line_origin(A_spr_pt_gnd,A_spr_pt_AB);
    A_spr_torque = A_spr_force*A_spr_to_origin;
    
    B_spr_pt = [B_spr_r*cos(B_angle),B_spr_r*sin(B_angle),0];
    B_spr_len = norm(B_spr_pt-A_spr_pt_gnd);
    B_spr_force = B_spr_k*(B_spr_len-B_spr_free_len);
    force_arrow(B_spr_pt,A_spr_pt_gnd-B_spr_pt,B_spr_force);
    B_spr_torque = B_spr_force*dist_line_origin(A_spr_pt_gnd,B_spr_pt);
    
    B_mtr_trq = torque_at_B+B_spr_torque;
    
    // draw basic shapes to represent the arm, includes springs
    draw_basic();
    
    // Total Lower Arm Torque
    A_mtr_trq=p_torque_at_A+uaf_torque_at_A+A_spr_torque;
    color("Plum",1) torque_arrow([0,0,0],A_mtr_trq);
    
    // Output to console.  Used to get data into spreadsheet
    echo ($t=$t,c=c,A_angle=A_angle,B_angle=B_angle,
    B_spr_torque=B_spr_torque,B_mtr_trq=B_mtr_trq,
    p_torque_at_A=p_torque_at_A,uaf_torque_at_A=uaf_torque_at_A,
    A_spr_force=A_spr_force,A_spr_to_origin=A_spr_to_origin,
    A_spr_torque=A_spr_torque,A_mtr_trq=A_mtr_trq); 
    module draw_basic () {
        // AB link
        color("Plum",.5)  pt_pt_cylinder(from=origin, to=b, d=0.2);
        // BC link
        color("Blue",.5)  pt_pt_cylinder(from=b, to=c, d=0.2);
        // CD end effector
        color("Green",.5)  pt_pt_cylinder(from=c, to=c+LengthEnd, d=0.2);
        // AB link sping
        color("Plum",.7)  pt_pt_cylinder(from=A_spr_pt_AB,to=A_spr_pt_gnd,d=0.2);
        // BC link sping
        color("Blue",.7)  pt_pt_cylinder(from=B_spr_pt,to=A_spr_pt_gnd,d=0.2);
    }
}


function get_pt_from_angles (A)= ([C_x_ang(A[0],A[1]),C_y_ang(A[0],A[1]),0]); 

function get_angles_from_t 
(t=0.5,min_A=-10,max_A=150,min_B=-50,max_B=90)= 
(t<0.15) ? ([min_A,interp(min_B,min_A+max_B_to_A,t,0,0.15),0]) : 
 (t<0.38) ? ([interp(0,100,t,0.15,0.38),interp(0,100,t,0.15,0.38),0]): 
 (t<0.5) ? ([interp(max_B-max_B_to_A,max_A,t,0.38,0.5),max_B,0]): 
(t<0.63) ? ([max_A,interp(max_B,max_A+min_B_to_A,t,0.5,0.63),0]): 
(t<0.75) ?([interp(max_A,min_B-min_B_to_A,t,0.63,0.75),interp(max_A+min_B_to_A,min_B,t,0.63,0.75),0]) :
([interp(min_B-min_B_to_A,min_A,t,0.75,1),min_B,0]); 
/*  old version of get_angles_from_t
 (t<0.38) ? ([interp(min_A,max_B-max_B_to_A,t,0.15,0.38),interp(min_A+max_B_to_A,max_B,t,0.15,0.38),0]): 
function get_angles_from_t 
(t=0.5,min_A=-10,max_A=150,min_B=-50,max_B=90)= 
(t<0.25) ? ([interp(min_A,90,t,0,0.25),interp(min_A,90,t,0,0.25),0]) : 
 (t<0.38) ? ([interp(90,max_A,t,0.25,0.38),interp(90,max_B,t,0.25,0.38),0]): 
 (t<0.5) ? ([max_A,interp(max_B,max_A+min_B_to_A,t,0.38,0.5),0]): 
(t<0.63) ? ([interp(max_A,70,t,0.5,0.63),interp(max_A+min_B_to_A,70+min_B_to_A,t,0.5,0.63),0]): 
(t<0.75) ?([interp(70,min_A,t,0.63,0.75),interp(70+min_B_to_A,min_B,t,0.63,0.75),0]) :
([min_A,interp(min_B,min_A,t,0.75,1),0]); 
*/
function throw_from_t 
(t=0.5,min_A=-10,max_A=150,min_B=-50,max_B=90)= 
(t<0.25) ? ([interp(0,max_A,t,0,0.25),interp(0,max_B,t,0,0.25),0]) : 
 (t<0.9) ? ([interp(max_A,throw_min_A,t,0.25,0.9),interp(max_B,min_B+50,t,0.25,0.9),0]): 
([interp(throw_min_A,0,t,0.9,1),interp(min_B+50,0,t,0.9,1),0]); 

function C_x_ang (A,B) = (cos(A)*lenAB+cos(B)*lenBC);

function C_y_ang (A,B) = (sin(A)*lenAB+sin(B)*lenBC);

function interp (A,B,t,t_l,t_h) = (A+((t-t_l)/(t_h-t_l))*(B-A));

module inverse (c=[10,10,0]) {
    // calculate the angles from pt ***Inverse Kinematics***

    vt = norm(c);  // vector length from A to C

    sub_angle1 = atan2(c[1],c[0]);  // atan2 (Y,X)!
    sub_angle2 = acos((vt*vt+lenAB*lenAB-(lenBC*lenBC))/(2*vt*lenAB));
    //echo(vt=vt,sub_angle1=sub_angle1,sub_angle2=sub_angle2);
    A_angle = sub_angle1 + sub_angle2;
    B_angle = acos((lenBC*lenBC+lenAB*lenAB-vt*vt)/(2*lenBC*lenAB));
} 
module plot_limits(n=20){
    // plot the expected limits of range of motion
    $fs=.15;
    r=0.15;
    
    for (i=[0:1/n:1]){
        color("salmon") 
        translate(get_pt_from_angles(get_angles_from_t(i,min_A,max_A,min_B,max_B))) 
        circle(r);
    }
}
module plot_throw(n=20){
    // plot the expected limits of range of motion
    $fn=8;
    r=0.1;
    
    for (i=[0:1/n:1]){
        color("blue") 
        translate(get_pt_from_angles(throw_from_t(i,min_A,max_A,min_B,max_B))) 
        circle(r);
    }
}
