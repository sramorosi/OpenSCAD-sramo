// Compliant 2D Beam Simulator
// using method of Pseudo-Rigid-Body Model (PRBM) 
//     from Handbook of Compliant Mechanisms by Larry L Howell
// Started on 6/5/2020 by SrAmo
// last modified 9/6/2020
use <force_lib.scad>

// material and PRBM constants
// Modulus of Elasticity (PSI)
E = 340000;
// Tensile Ultimate Strength (PSI)
Ftu = 7000;
// material density (lb per cubic inch)
rho = 0.05;

// MADE-UP FACTOR TO DETERMINE IF FORCE OR MOMENT
Q=1; 

// Display Node Points?
display_nodes = true;

// echo values?
output_console = false;

// Beam DNA: [[length,t,w,theta],[l,t,w,theta]...]
// where length is length of the segment
//       t and w are the thickness and width (Z, out of page) of segment
//       theta is the starting angle of the segment, relative to the previous segment!

// F_ext = Beam External Forces on each segment end [Fx,Fy] global
// M_ext = Beam External Moments on each segment end [Mz] global

// CANTELIVER BEAM WITH MOMENT, 2 SEGMENT
//dna = [[1.5,.15,.8,0],[1.5,.15,.8,0]];
//F_ext = [[0,0],[0,0]];
//M_ext = [0,18];

/*
// CANTELIVER BEAM WITH MOMENT, 4 SEGMENT
// Beam thickness
t=.1;
dna = [[.75,t,.8,0],[.75,t,.8,0],[.75,t,.8,0],[.75,t,.8,0]];
F_ext = [[0,0],[0,0],[0,0],[0,0]];
M_ext = [0,0,0,18*$t];
*/

// CANTELIVER BEAM WITH FORCE, 2 SEGMENT
//dna = [[1.5,.15,.8,0],[1.5,.15,.8,0]];
//F_ext = [[0,0],[0,10]];
//M_ext = [0,0];

/* CANTELIVER BEAM WITH FORCE, 4 SEGMENT
dna = [[.75,.15,.8,0],[.75,.15,.8,0],[.75,.15,.8,0],[.75,.15,.8,0]];
F_ext = [[0,0],[0,0],[0,0],[0,10]];
M_ext = [0,0,0,0];
*/

// CANTELIVER BEAM WITH FORCE, 6 SEGMENT
// beam thickness
t=0.1;
// beam width
w=.4;
dna = [[.5,t,w,0],[.5,t,w,0],[.5,t,w,0],[.5,t,w,0],[.5,t,w,0],[.5,t,w,0]];
F_ext = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,4*$t]];
M_ext = [0,0,0,0,0,0];
//

/* U shaped beam, 10 segment:
dna = [[1,.07,.8,0],[1,.07,.8,0],[.313,.07,.8,18],[.313,.07,.8,18],[.313,.07,.8,18],[.313,.07,.8,18],[1,.07,.8,18],[1,.07,.8,0],[1,.07,.8,0],[1,.07,.8,0]];
F_ext = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[-1,.01],[0,0],[0,0],[.2,0]];
M_ext = [0,0,0,0,0,0,0,0,0,0]; */

/*
// Compliant Claw, 33 segment 
// Compliant Claw, Lower Arc Segment 1 Lengths
s1=0.135;
// Compliant Claw, Lower Arc Segment relative Angles
a1=10.3;
// Compliant Claw, Linear Segment Lengths
s2=0.35;
// Compliant Claw, Thicknesses
t=0.075;
//Compliant Claw, Widths
w=1.5;
P=12*$t; // load pulling at servo
dna = [[s1,t,w,-90+a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s1,t,w,a1],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0],[s2,t,w,0]];
F_ext = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[-P,-.27*P],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[.3*P,0],[0,0],[0,0],[0,0],[0,0],[0,0]];
// Compliant Claw External Moments
M_ext = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
*/
// Straight diagonal Test Beam, 6 segment:
//dna = [[.5,.15,.8,45],[.5,.15,.8,0],[.5,.15,.8,0],[.5,.15,.8,0],[.5,.15,.8,0],[.5,.15,.8,0]];
//F_ext = [[0,0],[0,0],[0,0],[0,0],[0,0],[-4.2426,4.2426]];
//M_ext = [0,0,0,0,0,0];

/* Horizontal Test Column, 6 segment:
// beam thickness
t = 0.05;
dna = [[.5,t,.8,1.],[.5,t,.8,0],[.5,t,.8,0],[.5,t,.8,-2],[.5,t,.8,0],[.5,t,.8,0]];
F_ext = [[0,0],[0,0],[0,0],[0,0],[0,0],[-35*$t,0]];
M_ext = [0,0,0,0,0,0];
*/

//echo("INPUT [length,t,w,ang]",dna=dna);
//echo("INPUT FORCE GLOBAL SYS",F_ext=F_ext);

// CHECK number of beam segments
n = len(dna);
n_sub = [ for (i=[0:1:n-1]) len(dna[i]) ];
if ( sumv(n_sub,n-1) != n*4 ) { echo (n_sub=n_sub," NOT EQUAL ",n=n);};
n_F= len(F_ext);
if ( n_F != n ) { echo (n_F=n_F," NOT EQUAL ",n=n);};
n_M=len(M_ext);
if ( n_M != n ) { echo (n_M=n_M," NOT EQUAL ",n=n);};

// Draw the beam nodes (no load)
if (display_nodes) segmented_beam_undeformed(dna);

// generate moment of inertia
Iz=[ for (i=[0:1:n-1]) Iz_func(dna[i][2],dna[i][1]) ];
if (output_console) echo("INERTIA",Iz=Iz);

// generate area
Area=[ for (i=[0:1:n-1]) dna[i][1]*dna[i][2] ];
if (output_console) echo("AREA",Area=Area);

// sum angles along segments to get global angles, never changes
ang_initial = [ for (i=[0:1:n-1]) sumv(select(dna,3),i),0 ];
if (output_console) echo ("SEG ANG NO LOAD",ang_initial=ang_initial);

// generate dx,dy for each beam [[dx,dy],[dx,dy],...]
d0 = [ for (i=[0:1:n-1]) [dna[i][0]*cos(ang_initial[i]),dna[i][0]*sin(ang_initial[i])] ];
if (output_console) echo ("SEG dx,dy NO LOAD",d0=d0);

//******BEGIN ITERATION ZERO*******

// Load scaler for first iteration
scale_0=0.333;
// generate internal forces from external forces
F_int0 = [ for (i=[0:1:n-1]) sumv(F_ext,n-1,i)*scale_0 ];
if (output_console) echo ("INTERNAL FORCES",F_int0=F_int0);
    echo ("INTERNAL FORCES",F_int0=F_int0);

// generate fx and fy in local beam system
Fl_int0 = [ for (i=[0:1:n-1]) 
    [rot_x (F_int0[i][0],F_int0[i][1],-ang_initial[i]),
     rot_y (F_int0[i][0],F_int0[i][1],-ang_initial[i])] ];
if (output_console) echo ("INT FRC LOCAL BEAM",Fl_int0=Fl_int0);

// generate segment moments on the fixed end of beam
M_0 = [ for (i=[0:1:n-1]) F_int0[i][1]*d0[i][0]-F_int0[i][0]*d0[i][1] ];
if (output_console) echo ("INT MOMENTS DUE TO LOAD, EACH",M_0=M_0);

// sum  moments on the fixed end of each beam
M0 = [ for (i=[0:1:n-1]) sumv(M_0,n-1,i)+sumv(M_ext,n-1,i)*scale_0,0];
if (output_console) echo ("INT MOMENTS SUM",M0=M0);

// determine type of loading (force = 0, moment=1)
type0 = [ for (i=[0:1:n-1]) get_type(Fl_int0[i][0],Fl_int0[i][1],M0[i+1])];
if (output_console) echo ("TYPE LOAD,F=0,M=1",type0=type0);

// generate n  = (fx/fy) check for zero denominator
n0=[ for (i=[0:1:n-1]) (Fl_int0[i][1]<0.01 ? Fl_int0[i][0]/0.01 : -Fl_int0[i][0]/Fl_int0[i][1] ) ];
if (output_console) echo("n=-FX/FY",n0=n0);

// generate PRBM coefficients [[gamma,Kbsc]...
PRBM0=[ for (i=[0:1:n-1]) (type0[i]==0 ? [gamma(Fl_int0[i][0]/Fl_int0[i][1]),
    Kbsc(Fl_int0[i][0]/Fl_int0[i][1]) ] : [0.7346,1.5164]) ];
if (output_console) echo("gamma,Kbsc",PRBM0=PRBM0);

// generate k spring rate
K0=[ for (i=[0:1:n-1]) K(PRBM0[i][1],Iz[i],dna[i][0])];
if (output_console) echo("K spring",K0=K0);

// generate theta rotation from both force and moment
theta0=[ for (i=[0:1:n-1]) func_theta(PRBM0[i][0],dna[i][0],K0[i],(Fl_int0[i][1]/sqrt((1+n0[i]*n0[i]))),0,n0[i])+180/PI*M0[i]/K0[i] ];
if (output_console) echo("THETA spring",theta0=theta0);

// generate segment angle with load
aa0=[ 0, for (i=[0:1:n-1]) theta0[i]*pac(type0[i]) ];
//echo(aa0=aa0);
ang0=[ for (i=[0:1:n]) ang_initial[i]+sumv(aa0,i) ];
if (output_console) echo("SEG ANG LOADED",ang0=ang0);
    
// generate local tip position x,y
dx0=[ 0, for (i=[0:1:n-1]) xtip(dna[i][0],PRBM0[i][0],theta0[i],ang0[i]) ];
dy0=[ 0, for (i=[0:1:n-1]) ytip(dna[i][0],PRBM0[i][0],theta0[i],ang0[i]) ];
if (output_console) echo("dx LOCAL LOADED",dx0=dx0);
if (output_console) echo("dx LOCAL LOADED",dy0=dy0);

// generate global tip position x,y sumv(v,i,s=0)
XY0=[ for (i=[0:1:n]) [sumv(dx0,i),sumv(dy0,i)] ];
if (output_console) echo("X,Y 0",XY0=XY0);

// DRAW DISPLACEMENT CIRCLES FOR DEBUGGING
if (display_nodes) {
    for (i=[0:1:n]) {
        translate([XY0[i][0],XY0[i][1],0])
            color([val_red(1.1-scale_0),val_green(1.1-scale_0),0]) 
                circle(0.03,$fn=10);
    }
}

echo("#### END OF ITERATION ZERO ####",scale_0=scale_0);
//*********** END ITERATION ZERO************

//**********BEGIN ITERATION ONE************
scale_1=.6667;
// generate internal forces from external forces
F_int1 = [ for (i=[0:1:n-1]) sumv(F_ext,n-1,i)*scale_1 ];
if (output_console) echo ("INTERNAL FORCES",F_int1=F_int1);

// generate fx and fy in local beam system
Fl_int1 = [ for (i=[0:1:n-1]) 
    [rot_x (F_int1[i][0],F_int1[i][1],-ang0[i]),
     rot_y (F_int1[i][0],F_int1[i][1],-ang0[i])] ];
if (output_console) echo ("INT FRC LOCAL BEAM",Fl_int1=Fl_int1);

// generate segment moments on the fixed end of beam
M_1 = [ for (i=[0:1:n-1]) F_int1[i][1]*dx0[i+1]-F_int1[i][0]*dy0[i+1] ];
if (output_console) echo ("INT MOMENTS EACH",M_1=M_1);

// sum  moments on the fixed end of each beam
M1 = [ for (i=[0:1:n-1]) sumv(M_1,n-1,i)+sumv(M_ext,n-1,i)*scale_1, 0 ];
if (output_console) echo ("INT MOMENTS SUM",M1=M1);

// determine type of loading (force = 0, moment=1)
type1 = [ for (i=[0:1:n-1]) get_type(Fl_int1[i][0],Fl_int1[i][1],M1[i+1])];
if (output_console) echo ("TYPE LOAD,F=0,M=1",type1=type1);

// generate n  = (fx/fy) check for zero denominator
n1=[ for (i=[0:1:n-1]) (Fl_int1[i][1]<0.01 ? Fl_int1[i][0]/0.01 : -Fl_int1[i][0]/Fl_int1[i][1] ) ];
if (output_console) echo("n=-FX/FY",n1=n1);

// generate PRBM coefficients [[gamma,Kbsc]...
PRBM1=[ for (i=[0:1:n-1]) (type1[i]==0 ? [gamma(Fl_int1[i][0]/Fl_int1[i][1]),
    Kbsc(Fl_int1[i][0]/Fl_int1[i][1]) ] : [0.7346,1.5164]) ];
if (output_console) echo("gamma,Kbsc",PRBM1=PRBM1);

// generate k spring rate
K1=[ for (i=[0:1:n-1]) K(PRBM1[i][1],Iz[i],dna[i][0])];
if (output_console) echo("K spring",K1=K1);

// generate theta rotation from both force and moment (use prior iteration)
theta1=[ for (i=[0:1:n-1]) func_theta(PRBM1[i][0],dna[i][0],K1[i],(Fl_int1[i][1]/sqrt((1+n1[i]*n1[i]))),theta0[i],n1[i])+(180/PI)*M1[i]/K1[i] ];
if (output_console) echo("THETA spring",theta1=theta1);

// generate segment angle for next segment
aa1=[ 0, for (i=[0:1:n-1]) theta1[i]*pac(type1[i]) ];
ang1=[ for (i=[0:1:n]) ang_initial[i]+sumv(aa1,i) ];
if (output_console) echo("SEG ANG LOADED",ang1=ang1);
    
// generate local tip position x,y
dx1=[ 0, for (i=[0:1:n-1]) xtip(dna[i][0],PRBM1[i][0],theta1[i],ang1[i]) ];
dy1=[ 0, for (i=[0:1:n-1]) ytip(dna[i][0],PRBM1[i][0],theta1[i],ang1[i]) ];
if (output_console) echo("dx LOCAL LOADED",dx1=dx1);
if (output_console) echo("dx LOCAL LOADED",dy1=dy1);

// generate global tip position x,y sumv(v,i,s=0)
XY1=[ for (i=[0:1:n]) [sumv(dx1,i),sumv(dy1,i)] ];
if (output_console) echo("X,Y 1",XY1=XY1);

//  DRAW DISPLACEMENT CIRCLES FOR DEBUGGING

if (display_nodes) {
    for (i=[0:1:n]) {
        translate([XY1[i][0],XY1[i][1],0])
            //color([val_red(1.1-scale_1),val_green(1.1-scale_1),0]) 
                circle(0.03,$fn=10);
    }
}

echo("#### END OF ITERATION ONE ####",scale_1=scale_1);
//*********** END ITERATION ONE************///

//**********BEGIN ITERATION TWO************
scale_2=1;
// generate internal forces from external forces
F_int2 = [ for (i=[0:1:n-1]) sumv(F_ext,n-1,i)*scale_2 ];
if (output_console) echo ("INTERNAL FORCES",F_int2=F_int2);

// generate fx and fy in local beam system
Fl_int2 = [ for (i=[0:1:n-1]) 
    [rot_x (F_int2[i][0],F_int2[i][1],-ang1[i]),
     rot_y (F_int2[i][0],F_int2[i][1],-ang1[i])] ];
if (output_console) echo ("INT FRC LOCAL BEAM",Fl_int2=Fl_int2);

// generate segment moments on the fixed end of beam
M_2 = [ for (i=[0:1:n-1]) F_int2[i][1]*dx0[i+1]-F_int2[i][0]*dy0[i+1] ];
if (output_console) echo ("INT MOMENTS EACH",M_2=M_2);

// sum  moments on the fixed end of each beam
M2 = [ for (i=[0:1:n-1]) sumv(M_2,n-1,i)+sumv(M_ext,n-1,i)*scale_2, 0 ];
if (output_console) echo ("INT MOMENTS SUM",M2=M2);

// determine type of loading (force = 0, moment=1)
type2 = [ for (i=[0:1:n-1]) get_type(Fl_int2[i][0],Fl_int2[i][1],M2[i+1])];
if (output_console) echo ("TYPE LOAD,F=0,M=1",type2=type2);

// generate n  = (fx/fy) check for zero denominator
n2=[ for (i=[0:1:n-1]) (Fl_int2[i][1]<0.01 ? Fl_int2[i][0]/0.01 : -Fl_int2[i][0]/Fl_int2[i][1] ) ];
if (output_console) echo("n=-FX/FY",n2=n2);

// generate PRBM coefficients [[gamma,Kbsc]...
PRBM2=[ for (i=[0:1:n-1]) (type2[i]==0 ? [gamma(Fl_int2[i][0]/Fl_int2[i][1]),
    Kbsc(Fl_int2[i][0]/Fl_int2[i][1]) ] : [0.7346,1.5164]) ];
if (output_console) echo("gamma,Kbsc",PRBM2=PRBM2);

// generate k spring rate
K2=[ for (i=[0:1:n-1]) K(PRBM2[i][1],Iz[i],dna[i][0])];
if (output_console) echo("K spring",K2=K2);

// generate theta rotation from both force and moment (use prior iteration)
theta2=[ for (i=[0:1:n-1]) func_theta(PRBM2[i][0],dna[i][0],K2[i],(Fl_int2[i][1]/sqrt((1+n2[i]*n2[i]))),theta1[i],n2[i])+(180/PI)*M2[i]/K2[i] ];
if (output_console) echo("THETA spring",theta1=theta1);

// generate segment angle for next segment
aa2=[ 0, for (i=[0:1:n-1]) theta2[i]*pac(type2[i]) ];
ang2=[ for (i=[0:1:n]) ang_initial[i]+sumv(aa2,i) ];
if (output_console) echo("SEG ANG LOADED",ang2=ang2);
    
// generate local tip position x,y
dx2=[ 0, for (i=[0:1:n-1]) xtip(dna[i][0],PRBM2[i][0],theta2[i],ang2[i]) ];
dy2=[ 0, for (i=[0:1:n-1]) ytip(dna[i][0],PRBM2[i][0],theta2[i],ang2[i]) ];
if (output_console) echo("dx LOCAL LOADED",dx2=dx2);
if (output_console) echo("dx LOCAL LOADED",dy2=dy2);

// generate global tip position x,y sumv(v,i,s=0)
XY2=[ for (i=[0:1:n]) [sumv(dx2,i),sumv(dy2,i)] ];
if (output_console) echo("X,Y 2",XY2=XY2);

// DRAW DISPLACEMENT CIRCLES FOR DEBUGGING
if (display_nodes) {
    for (i=[0:1:n]) {
        translate([XY2[i][0],XY2[i][1],0])
            color([val_red(1.1-scale_2),val_green(1.1-scale_2),0]) 
                circle(0.03,$fn=10);
    }
}

echo("#### END OF ITERATION TWO ####",scale_2=scale_2);
//*********** END ITERATION TWO ************///

echo("MAX F/M ALGORITHEMS NEED TO BE REDONE (sqrt of sum squares)");
max_size = abs(max(select(XY2,0))+max(select(XY2,1))+min(select(XY2,0))+min(select(XY2,1)));
max_force = abs(max(select(F_int2,0)))+abs(max(select(F_int2,1)))+abs(min(select(F_int2,0)))+abs(min(select(F_int2,1)));
f_scale = (max_size/2)/(4*max_force);
max_moment = abs(max(M2))+abs(min(M2));
m_scale = (max_size/2)/(4*max_moment);
echo(max_size=max_size,max_force=max_force,f_scale=f_scale,max_moment=max_moment,m_scale=m_scale);

draw_internal_loads(dx2,dy2,F_int2,M2);

// draw external loads using same module
draw_loads(dx2,dy2,F_ext);


// calcute and echo stresses
bend_stress=[for(i=[0:1:n-1]) (M2[i]+Fl_int2[i][1]*dx2[i])*(dna[i][1]/2)/Iz[i] ];
echo("STRESS DUE TO MOMENT, MAX ",max(bend_stress)," MIN ",min(bend_stress));
axial_stress=[for(i=[0:1:n-1]) Fl_int2[i][0]/Area[i] ];
echo("STRESS DUE AXIAL LOAD, MAX ",max(axial_stress)," MIN ",min(axial_stress));
MS=[ for (i=[0:1:n-1]) 1-(abs(bend_stress[i])-axial_stress[i])/Ftu ];
echo("Margin of Safety, MIN ",min(MS));

//segmented_beam_deformed(dna,PRBM0,theta0,ang0,type0,MS);
segmented_beam_deformed(dna,PRBM2,theta2,ang2,type2,MS);


///////////////////FUNCTIONS////////////////////

// get beam type from forces **TO BE WORKED**
function get_type(fx=1,fy=1,m=.2) = 
    ((abs(fx)+abs(fy))>abs(Q*m) )? 0 : 1 ;

// convert the margin of safety into a red to green value
function val_red(i) = i < .5 ? 1 : i > 1 ? 0 : 2-i*2 ;

function val_green(i) = i < 0 ? 0 : i > .5 ? 1 : i*2 ;

// segment area moment of inertia about Z axis
function Iz_func(w=1,t=.1) = ((w*t*t*t)/12);

// PRBM characturistic radius for cantilever beam
//gamma = 0.85 for a vertically loaded beam;
function gamma(n) = (n>10? 0.817 : n>0.5 ? 
.841655-0.0067807*n+0.000438*n*n : n>-1.8316 ? 
.852144-0.0182867*n : n>-5? .912364+0.0145928*n : .839);

// PRBM bending stiffness coefficient for cantilever beam
//Kbsc = 2.258 for a vertically loaded beam;
function Kbsc(n)=(n>10 ? 2.576: n>-1 ? 
2.654855-0.0509896*n+0.0126749*n*n-.00142039*n*n*n+.000584525*n*n*n*n : n>-2.5 ? 
1.967647-2.616021*n-3.738166*n*n-2.649437*n*n*n-0.891906*n*n*n*n-0.113063*n*n*n*n*n: n>-5 ?
3.024112+0.12129*n+.003169*n*n : 2.497);

// PRBM spring constant value
function K(Kbsc,Iz,len) = Kbsc*E*Iz/len;

// pivot angle (note estimated angle term)
// Recursive Function!!
function func_theta(gamma,l,k,p,est_ang,n) = 180/PI*(p/k)*gamma*l*(cos(est_ang)+n*sin(est_ang)); 
//function theta(gamma,len,k,fy,est_ang,n) = 
//let(new= 180/PI*(fy/k)*gamma*len*(cos(est_ang)+n*sin(est_ang))) 
//abs(new-est_ang) < 0.5 ? new : theta(gamma,len,k,fy,new,n) ;
//let(new= 180/PI*(fy/k)*gamma*len*(cos(est_ang)+n*sin(est_ang))) 
// PRBM Parametric Angle Coefficient for cantilever beam
function pac(n) = (n==0 ? 1.24 : 1.5164);

// recursion - find the sum of the values in a vector (array) by calling itself
// from the start (or s'th element) to the i'th element - remember elements are zero based
// From the OpenSCAD user manual on recursive calls
function sumv(v,i,s=0) = (i==s ? v[i] : v[i] + sumv(v,i-1,s));

// select a sub vector from a 2d vector
function select(vector,index) = [ for (i = [0:1:len(vector)-1]) vector[i][index] ];
    
function cat(L1, L2) = [for (i=[0:len(L1)+len(L2)-1]) 
                i < len(L1)? L1[i] : L2[i-len(L1)]] ;
    
function rot_x (x,y,a) = x*cos(a)-y*sin(a);

function rot_y (x,y,a) = x*sin(a)+y*cos(a);

function xtip(L,g,tha,a) = rot_x(L*(1-g*(1-cos(tha))),g*L*sin(tha),a);

function ytip(L,g,tha,a) = rot_y(L*(1-g*(1-cos(tha))),g*L*sin(tha),a);

///////////////////MODULES////////////////////

// recursive module that draws the deformed beam.
module segmented_beam_deformed(dna_vector,PRBM,THETA,NEWA,TYP,ms,idx = 0) {
    if (idx < len(dna_vector)) {
        L = dna_vector[idx][0];
        t = dna_vector[idx][1];
        w = dna_vector[idx][2];
        z_ang =  NEWA[idx];
        
        gamma=PRBM[idx][0];
        theta = THETA[idx];
        //echo(idx=idx,z_ang=z_ang,gamma=gamma,theta=theta);

        // draw the beam segment
        color([val_red(ms[idx]),val_green(ms[idx]),.2]) 
            //rotate([0,0,z_ang*pac(TYP[idx])])
            rotate([0,0,z_ang])
            translate([(L*(1-gamma))/2,0,0])
                cube([L*(1-gamma),t,w],center=true);
        color([val_red(ms[idx]),val_green(ms[idx]),0]) 
            //rotate([0,0,z_ang*pac(TYP[idx])])
            rotate([0,0,z_ang])
            translate([L*(1-gamma),0,0])
            rotate([0,0,theta])
                translate([L*gamma/2,0,0])
                cube([L*gamma,t,w],center=true);
        
        // Recursive call generating the next beam
        // transform according to the prior beam segment

    translate([xtip(L,gamma,theta,z_ang),ytip(L,gamma,theta,z_ang),0]) 
        //rotate([0,0,theta*pac(TYP[idx])])  
            segmented_beam_deformed(dna_vector,PRBM,THETA,NEWA,TYP,ms,idx+1);
    }
}
// recursive module that draws the undeformed beam.
module segmented_beam_undeformed(dna_vector,idx = 0) {
    if (idx < len(dna_vector)) {
        L = dna_vector[idx][0];
        t = dna_vector[idx][1];
        w = dna_vector[idx][2];
        z_ang = dna_vector[idx][3];
        
        // draw the beam segment
        color ("yellow") 
                rotate([0,0,z_ang])
                    translate([L,0,0])
                        //cube([L,t,w],center=true);
                         circle(0.03,$fn=10);
        // Recursive call generating the next beam
        // transform according to the prior beam segment
        translate([L*cos(z_ang), L*sin(z_ang), 0]) {
            rotate([0,0,z_ang])
                segmented_beam_undeformed(dna_vector,idx+1);
        }
    }
}

// recursive module that draws forces and moments.
module draw_internal_loads(dx,dy,f,m,idx = 1) {
    if (idx <= len(dx)) {
        if (idx > 1) {
            // draw forces
            fx = f[idx-2][0];
            fy = f[idx-2][1];
            fmag = sqrt(fx*fx + fy*fy);
            if (abs(fmag*f_scale)>0.1) {
                color ("blue")
                    force_arrow([0,0,0],[fx,fy,0],mag=fmag*f_scale);
            }
        
            // draw the moments
            moment = m[idx-1];
            if (abs(moment*m_scale)>0.1) {
                color ("blue")
                    torque_arrow([0,0,0],mag=moment*m_scale);
            }  
        }  
        //echo(idx=idx,fx=fx,fy=fy,moment=moment);
        // Recursive call generating the next beam
        // transform according to the prior beam segment
        if (idx < len(dx)) 
            translate([dx[idx], dy[idx], 0]) {
            draw_internal_loads(dx,dy,f,m,idx+1);
        }
    }
}
// recursive module that draws forces.
module draw_loads(dx,dy,f,idx = 1) {
    if (idx <= len(dx)) {
        if (idx > 1) {
            // draw forces
            fx = f[idx-2][0];
            fy = f[idx-2][1];
            fmag = sqrt(fx*fx + fy*fy);
            if (abs(fmag)>0.1)
                color ("red")
                    force_arrow([0,0,0],[fx,fy,0],mag=fmag*f_scale);
            }
        
        // Recursive call generating the next beam
        // transform according to the prior beam segment
        if (idx < len(dx)) 
            translate([dx[idx], dy[idx], 0]) {
            draw_loads(dx,dy,f,idx+1);
        }
    }
}
