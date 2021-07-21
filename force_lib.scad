// Force and Torque Object Library
//  Started on 4/6/2020 by SrAmo
//  last modified 6/6/2020
/*  >>>>> Introduction <<<<<

    The purpose of this library is to:
    1) Create 3D Force Object
    2) Create 3D Torque Object (2d works, 3d not yet)
    3) Functions for vectors
    4) Function for distance from point to line
*/
// Force or Torque magnitude
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

// vector subtract function
function vector_subtract(a=[1,1,1],b=[-1,1,-1]) =
[(b[0]-a[0]),(b[1]-a[1]),(b[2]-a[2])];

// vector addition function
function vector_add(a=[1,1,1],b=[-1,1,-1]) = 
[(b[0]+a[0]),(b[1]+a[1]),(b[2]+a[2])];

// distance from 0,0 to line function
//  reference Wikipedia "distance from a point to a line"
//  see formula for line defined by two points
//  when the point is 0,0 the formula is simple
// line is defined by two points p1 and p2
function dist_line_origin (p1=[1,1],p2=[0,2])=
(p2[0]*p1[1]-p2[1]*p1[0])/norm([(p1[0]-p2[0]),(p1[1]-p2[1]),0]);

function dist_line_pt (p1=[-5,5,0],p2=[0,5,0],pt=[10,0,0])=
abs((p2[1]-p1[1])*pt[0]-(p2[0]-p1[0])*pt[1]+p2[0]*p1[1]-p2[1]*p1[0])/norm([(p1[0]-p2[0]),(p1[1]-p2[1]),0]);

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
