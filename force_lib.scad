// Force, Torque, Spring Module and Function Library
//  Started on 4/6/2020 by SrAmo
//  last modified 7/22/2021 by SrAmo
/*  >>>>> Introduction <<<<<

    The purpose of this library is to:
    1) Create 3D Force display module
    2) Create 3D Torque display module (2d works, 3d not yet)
    3) Function for distance from point to line
*/
use <Robot_Arm_Parts_lib.scad>
// Force or Torque magnitude for testing modules
mag = 1; // [-40:0.2:40]
// Display force arrows
display_force = false; 
// Display torque arrow
display_torque = false;

// test array
e=[[2,3,4],[1,2,3],[0,0,1]];
echo(norm(e[0]),norm(e[1]),norm(e[2]));
echo(cross([1,0,0],[0,0,1]));
echo(e[1],rotZ_pt(90,e[1]));

echo(dist_line_pt());

if (display_force) force_arrow([0,0,0],[1,0,0],mag=mag);
if (display_force) force_arrow([0,0,0],[0,-1,0],mag=mag);
if (display_force) force_arrow([0,0,0],[0,0,1],mag=-mag); // note neg mag

if (display_force) color ("blue") force_arrow([2,2,2],[1,1,0],mag=mag);
if (display_force) color ("green") force_arrow([2,2,2],[0,1,1],mag=mag);
if (display_force) force_arrow([2,2,2],[1,0,1],mag=mag);

if (display_torque) torque_arrow([0,0,0],mag=mag);
if (display_torque) torque_arrow([0,0,5],mag=-mag);

// linear interpolation function
// Returns the value between A and B given t between t_l and t_h
function linear_interp (A,B,t,t_l,t_h) = (A+((t-t_l)/(t_h-t_l))*(B-A));

// distance from 0,0 to line function
//  reference Wikipedia "distance from a point to a line"
//  see formula for line defined by two points
//  when the point is 0,0 the formula is simple
// line is defined by two points p1 and p2
function dist_line_origin (p1=[1,1],p2=[0,2])=
(p2[0]*p1[1]-p2[1]*p1[0])/norm([(p1[0]-p2[0]),(p1[1]-p2[1]),0]);

function dist_line_pt (p1=[-5,5,0],p2=[0,5,0],pt=[10,0,0])=
// removed abs
((p2[1]-p1[1])*pt[0]-(p2[0]-p1[0])*pt[1]+p2[0]*p1[1]-p2[1]*p1[0])/norm([(p1[0]-p2[0]),(p1[1]-p2[1]),0]);

function rotZ_pt (a=10,p=[1,1,0]) = ([(p[0]*cos(a)+p[1]*sin(a)),(p[1]*cos(a)+p[0]*sin(a)),p[2]]);

module plot_circle(rad=2,n=10,dot_r=2,center=[0,0,0]) {
    // plots a circle of radius rad, at center, in n steps
    
    $fn = $preview ? 12 : 24;     // number of fragments
    
    for (i=[0:360/n:360]){
        // draw a small circle at each point
        translate([ (center[0]+rad*cos(i)),
        (center[1]+rad*sin(i))])      
        circle(dot_r);       
    };
}

module force_arrow(from=[1,1,0],vec=[1,0,0],mag=10) {
    // draw a 3D force of length (mag), at (from), direction (vec)
    //
    $fn = $preview ? 10 : 20;     // number of fragments
    if (norm(vec)>0.001) {  // check for non zero vector
        
        dx = -vec[0];
        dy = -vec[1];
        dz = -vec[2];
        
        // "cylinder" is centered around z axis
        // These are angles needed to rotate to correct direction
        ay = 90 - atan2(dz, sqrt(dx*dx + dy*dy));
        az = atan2(dy, dx);
        angles = [0, ay, az];
        
        d = abs(mag); // used to scale arrow
        sclr=d*.05; // scaler for arrow shaft
        
        if (abs(mag)>0.1) { // don't draw if small
            color("red") 
            translate (from) 
            rotate (angles) 
            translate([0,0,-d])
            union () {
                translate([0,0,d*.1])
                    cylinder(d*.9,d1=sclr,d2=sclr,false);
                cylinder(d*.2,0,d*.1,false);
            }
        } else {
            echo("MODULE FORCE_ARROW; small mag = ",mag);
    }
    } else {
        echo("MODULE FORCE_ARROW; vec too small = ",vec);
    }
}
module torque_arrow(to=[10,4,0],mag=10) {
    // draw a torque of diameter abs(mag), at to point
    // assumes torque is on x,y plane (for now)
    // arrowhead changes direction with sign of mag
    //
    $fn = $preview ? 20 : 72;     // number of fragments
    d = abs(mag); // used to scale arrow
    sclr=d*.025; // scaler for arrow shaft
    if (abs(mag)>0.1) {
        // "cylinder" is centered around z axis
        // first rotate so that it is around x axis
        // then rotate about z to point in vector location
        color("blue") 
        translate (to) 
        rotate_extrude(angle=270,convexity = 10) 
        translate([abs(mag/2), 0, 0]) 
        circle(r = sclr);
        if (mag>0) { // positive arrowhead
            color("blue") 
            translate ([to[0]+d*.2,to[1]-mag/2,to[2]]) 
            rotate([0,-90,0]) 
            cylinder(d*.2,0,d*.1,false);
        } else { // negative arrowhead
            color("blue") 
            translate ([to[0]-mag/2,to[1]-d*.2,to[2]]) 
            rotate([-90,0,0]) 
            cylinder(d*.2,0,d*.1,false);
        }
    } else {
        echo("MODULE TORQUE_ARROW; small mag = ",mag);
    }
}
// Calculate the torque about a joint ptj caused by a pt1-pt2 spring
// of spring constant K and free length freelen
function spring_torque(pt1=[10,0,0],pt2=[10,10,0],ptj=[-10,0,0],K=1,freelen=1) = 
    let (arm = dist_line_pt(pt1,pt2,ptj)) // dist_line_pt in force_lib
    let (sprlen = norm(pt1-pt2)) 
    K * (sprlen-freelen) * arm;  // the torque calculation

sprtest = spring_torque();  // test
echo(sprtest=sprtest);

// convert number into a red to green value
function val_red(i) = i < 3 ? 0 : i < 4 ? 0.25 : i < 5 ? 0.5 : 1 ;
function val_green(i) = i < 3 ? 1 : i < 4 ? 0.75 : i < 5 ? 0.5 : 0;

// Draw a spring cylinder from pt1-pt2 
// of spring constant K and free length freelen
// and color it based on the percent elongation
module draw_spring(pt1=[100,0,50],pt2=[100,100,50],freelen=10) {
    sprlen = norm(pt1-pt2); 
    pct_elong = (sprlen-freelen)/freelen-1; 
    sprd = norm(pt1-pt2)/15; // spring diameter for display only
    echo(pct_elong=pct_elong*100,sprd=sprd);
    color ([val_red(pct_elong),val_green(pct_elong),0.1])
        pt_pt_cylinder(from=pt1, to=pt2, d=sprd);
}
draw_spring();

// vector subtract function ## Just use V1 - V2 ##
//function vector_subtract(a=[1,1,1],b=[-1,1,-1]) =[(b[0]-a[0]),(b[1]-a[1]),(b[2]-a[2])];

// vector addition function ## Just use V1 + V2 ##
//function vector_add(a=[1,1,1],b=[-1,1,-1]) = [(b[0]+a[0]),(b[1]+a[1]),(b[2]+a[2])];


