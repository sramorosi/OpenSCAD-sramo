// MK2 EEZY Robot Arm Simulation tool
//  last modified 5/20/2020 by SrAmo
/*  >>>>> Introduction <<<<<

    The purpose of this tool is to:
    1) Parametrically draw the MK2 EEKY style 2 degree-of-freedom Robot Arm
    2) Draw the forces and torques on the arm
    3) Draw the reach envelope for the arm
    4) Animate the arm through the reach envelope
    5) Provide design guidance on setting spring rate to minimize the torques on the motors
    
### Nomenclature  ###
    
    Arm logical diagram (links connected by joints):
    Base -A1- AB_arm -B- BCE_arm -C- 
         -A2- AD_arm -D- DE_link -E- BCE_arm
         
    Length AB = DE
    Length AD = BE
    
### Global Parameters are in the Configuration include file  ###

    Animation: use View/Animate.
    The OpenSCAD variable $t = time and ranges from 0 to .9999
    When animating the code is recompiled for each step.
    
    Parameters can be modified during animation!
*/

use <force_lib.scad>
use <Robot_Arm_Parts_lib.scad>
include <MK2-EEZY-Configuration.scad>

// Check to display assembly
display_assembly = true;
// Check to display and echo forces and torques 
display_forces = true; 
// Check to display reach path 
display_reach = true; 
// Check to ECHO Calculations
Echo_Calculations = true;

min_A=10;
max_A=150;
min_B=-65;
max_B=60;

// Maximum payload force (weight of thing being lifted)
payload=2.2;         
// weight of end effector with no payload
//payload_min=0.5;   

// spring free length
spr_free_len = 4.5;
// spring rate K (force/distance)
spr_k = 1.5;
spr_pt_gnd = [0,spr_dist_ground,0];   // spring attach point on ground

opt_spr_k=(payload*lenAB/2)/((sqrt(0.751*lenAB*lenAB)-lenAB/2)*(lenAB/4));  // optimized spring rate
//opt_balance_w=(((payload+payload_min)*lenBC)/2)/lenBal; // optimized 
if (Echo_Calculations) {
    echo (min_A=min_A,max_A=max_A,A_range=(max_A-min_A));
    echo (min_B=min_B,max_B=max_B,B_range=(max_B-min_B));
    
    steps = 60;
    cx = [ for (a = [0 : steps-1]) get_CX(get_angles_from_t(a/steps,min_A,max_A,min_B,max_B))];
    cx_min=min(cx);
    cx_max=max(cx);
    echo (cx_min=cx_min,cx_max=cx_max,x_range=cx_max-cx_min);
    cy = [ for (a = [0 : steps-1]) get_CY(get_angles_from_t(a/steps,min_A,max_A,min_B,max_B))];
    cy_min=min(cy);
    cy_max=max(cy);
    echo (cy_min=cy_min,cy_max=cy_max,y_range=cy_max-cy_min);

    echo(spr_k=spr_k,opt_spr_k=opt_spr_k);
}

alphas=get_angles_from_t($t,min_A,max_A,min_B,max_B);
A_angle = alphas[0];
B_angle = alphas[1];

if (display_assembly) {
    difference () {
        draw_assy (A_angle,B_angle); 
        // x = 0 cut 
        //translate ([-10,-5,-5])
        //cube (10,center=false);
        // z = 0 cut 
        //translate ([-5,-5,.1])
        //cube (10,center=false);
    }
}

if (display_forces) internal_loads (A_angle,B_angle,C_angle,lenAB,lenBC,r_pulley=end_pulley_d/2);
    
if (display_reach) plot_limits (80); // turn off for 3d rendering

//plot_circle (6,30,[10,10,0]);   // turn off for 3d rendering
//$vpr = [0, $t * 360,0];   // view point rotation


//$vpt = [c[0],c[1],c[2]];   // view point translation

//**************** END OF MAIN ******************
function get_CX (a) = (cos(a[0])*lenAB+cos(a[1])*lenBC);
function get_CY (a) = (sin(a[0])*lenAB+sin(a[1])*lenBC);

module internal_loads (A_angle=0,B_angle=0,C_angle=0,lenAB=9,lenBC=9,r_pulley=1) {
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
    spr_pt_AB = spr_dist_AB*vecAB;    
    
    // payload force on LengthEnd
    force_arrow(pt,[0,-1,0],payload);
    
    // Sum moments about C to determine belt force on LengthEnd
    belt_force=payload*(LengthEnd[0]/r_pulley); // ratio of distances
    force_arrow(LengthEnd_t,-vecBC,belt_force); 
    
    // Sum forces to determine force on joint C using a force polygon
    c_vec=vector_subtract(payload*[0,-1,0],belt_force*vecBC);
    c_to=vector_add(c,c_vec);  
    c_force=norm(c_vec);
    force_arrow(c,c_vec,c_force);
    force_arrow(b,-c_vec,c_force); // equal & opp on b
    
    // Determine torque on joint B.  Link BC is a cantilever beam.
    // The upper arm motor holds joint B in (fixed) rotation.
    // The torque is the joint c force x the distance to joint b.
    //
    // translate the c vectors to the origin for distance calc.
    c1=vector_subtract(c,b);
    c2=vector_subtract(c_to,b);
    cforce_to_b_arm=dist_line_origin([c1[0],c1[1]],[c2[0],c2[1]]);
    torque_at_B=cforce_to_b_arm*c_force;
    
    // To minimize the upper arm torque we add a balance weight.
    // Upper arm motor and balance weight is at joint A
    // They are connected by closed belt to B
    // This reduces the lower motor torque
    //  d = location of weight
    //d=[(lenBal*cos(angle_BC+180+adjust_ang_bal)), (lenBal*sin(angle_BC+180+adjust_ang_bal)),0];
    // balance weight torque = balance weight times the moment arm
    //bw_torque = balance_w*(lenBal*cos(angle_BC+180+adjust_ang_bal));
    // upper arm torque subtracts the balance weight torque.
    B_mtr_trq = (torque_at_B);
    //force_arrow(d,[0,-1,0],balance_w);
    
    // torque of upper arm motor at A
    color("Blue",1) torque_arrow([0,0,0],B_mtr_trq);
    
    // calculate and draw B joint pulley (belt) forces
    // there are two belt forces on the B joint pulley (_t1 and _t2)
    force_arrow(pulley_t1,vecBC,-belt_force); 
    force_arrow(pulley_t2,-vecAB,belt_force); 
    // Do vector polygon to sum forces on pulley at B
    p_vec=vector_subtract(-belt_force*vecBC,belt_force*-vecAB);
    p_force=norm(p_vec);
    force_arrow(b,p_vec,p_force); 
    
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
    spr_len = norm(vector_subtract(spr_pt_AB,spr_pt_gnd)); 
    spr_force = spr_k*(spr_len-spr_free_len);
    spr_to_origin=dist_line_origin([spr_pt_gnd[0],spr_pt_gnd[1]],
    [spr_pt_AB[0],spr_pt_AB[1]]);
    spr_torque = spr_force*spr_to_origin;
    
    // Total Lower Arm Torque
    A_mtr_trq=p_torque_at_A+uaf_torque_at_A+spr_torque;
    color("Plum",1) torque_arrow([0,0,0],A_mtr_trq);
    
    // Output to console.  Used to get data into spreadsheet
    echo ($t=$t,c=c,A_angle=A_angle,B_angle=B_angle,
    B_mtr_trq=B_mtr_trq,
    p_torque_at_A=p_torque_at_A,uaf_torque_at_A=uaf_torque_at_A,
    spr_force=spr_force,spr_to_origin=spr_to_origin,
    spr_torque=spr_torque,A_mtr_trq=A_mtr_trq); 
}


function get_pt_from_angles (A)= ([C_x_ang(A[0],A[1]),C_y_ang(A[0],A[1]),0]); 

function get_angles_from_t 
(t=0.5,min_A=-10,max_A=150,min_B=-50,max_B=90)= 
(t<0.25) ? ([interp(min_A,90,t,0,0.25),interp(-26,max_B,t,0,0.25),0]) : 
 (t<0.38) ? ([interp(90,max_A,t,0.25,0.38),max_B,0]): 
 (t<0.5) ? ([max_A,interp(max_B,0,t,0.38,0.5),0]): 
(t<0.63) ? ([interp(max_A,50,t,0.5,0.63),interp(0,min_B,t,0.5,0.63),0]): 
(t<0.75) ?([interp(50,min_A,t,0.63,0.75),min_B,0]) :
([min_A,interp(min_B,-26,t,0.75,1),0]); 

function C_x_ang (A,B) = (cos(A)*lenAB+cos(B)*lenBC);

function C_y_ang (A,B) = (sin(A)*lenAB+sin(B)*lenBC);

function interp (A,B,t,t_l,t_h) = (A+((t-t_l)/(t_h-t_l))*(B-A));

module draw_assy (A_angle=0,B_angle=0) {

    // calculate b and c positions from angles
    b=[lenAB*cos(A_angle),lenAB*sin(A_angle),0];  
    c=[(cos(A_angle)*lenAB+cos(B_angle)*lenBC),(sin(A_angle)*lenAB+sin(B_angle)*lenBC),0];
    BE_angle = B_angle+angEBC;
    ADE_angle = BE_angle-A_angle;
    d=[lenAD*cos(BE_angle),lenAD*sin(BE_angle),0];  
    e = d+b;
    
    echo(t=$t,A_angle=A_angle,B_angle=B_angle,BE_angle=BE_angle,ADE_angle=ADE_angle,b=b,c=c,e=e);
    
    // draw the upper and lower arms
    rotate(a=[0,0,A_angle]) {
        // Draw the AB link
        color("plum",1) 
            translate ([0,0,-widthAB/2])
                hollow_offset_link (length=lenAB,d_pin=hole_p25_inch,w=widthAB,t=widthAB,offset=.4,ang=35,wall=wall_t);
        // Draw the BCE link
        color("blue",1) 
            translate([lenAB,0,-widthBC/2-wall_t]) 
                rotate([0,0,B_angle-A_angle]) {
                    hollow_offset_link (length=lenBC,d_pin=hole_p25_inch,w=widthBC,t=widthBC,offset=.2,ang=15,wall=wall_t);
                    rotate([0,0,angEBC])
                        simple_link (l=lenBE,w=widthBC,t=0.2,d=hole_p25_inch);
                }
    }
    
    // Draw the AD arm
    color("navy",1)  
        rotate([0,0,BE_angle]) 
            simple_link (l=lenAD,w=widthBC,t=0.2,d=hole_p25_inch);

    // Draw the DE arm
    color("green",1)  
        translate(d)
            rotate([0,0,A_angle]) 
                simple_link (l=lenDE,w=widthBC,t=0.2,d=hole_p25_inch);

            
    // BASE    
    //color("grey") rotate([-90,0,0])final_base (r_pulley=end_pulley_d/2,spr_dist_ground=spr_dist_ground);
            
    // A Servo
    color ("red") rotate([0,180,-90])
        translate([0,0,A_servo_z]) {
            servo_body();
            rotate([0,0,A_angle+80])
                servo_horn();
        }
    // B Servo
    color ("red") rotate([0,0,90])
        translate([0,0,-B_servo_z-flange_z]) {
            servo_body();
            rotate([0,0,B_angle+7])
                servo_horn();
        }
} 

module plot_limits(n=20){
    // plot the expected limits of range of motion
    $fn=8;
    r=0.2;
    
    for (i=[0:1/n:1]){
        color("salmon") 
        translate(get_pt_from_angles(get_angles_from_t(i,min_A,max_A,min_B,max_B))) 
        circle(r);
    }
}
