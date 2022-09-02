// Damper for vacuum cannon
// by SrAmo  July, 2022
use <Robot_Arm_Parts_lib.scad>

MM = 25.4;

module piston() {
    difference () {
        washer(d=22.4,t=16,d_pin=4.5,$fn=96);
        cylinder(h=12,d=15,center=false,$fn=48);
    }
}    
*piston();

module cap(shaft=false) {
    difference () {
        union() {
            washer(d=35,t=20,d_pin=25.8,$fn=96);
            cylinder(h=10,d=30,$fn=48);
            translate([-28,13,-10]) cube([56,8,20]);
        }
        rotate([90,0,0]) 
            Rotation_Pattern(number=2,radius=22,total_angle=360)
                cylinder(h=50,d=4,center=true,$fn=24);
        if (shaft) {
            cylinder(h=50,d=5.0,center=true,$fn=48);
        }
    }
}
*translate([0,0,50]) cap(shaft=false);
*translate([0,0,-50]) rotate([180,0,180]) cap(shaft=true);

*color("grey",0.5) washer(d=25.6,d_pin=22.6,t=100,$fn=48);

module trapizoid2d(h1=40,h2=30,w=20,r=5) {
    h3=(h1-h2)/2;
    newh1 = h1-2*r;
    translate([r,r,0]) minkowski(convexity = 10,$fn=48) {
        polygon(points = [[0,0],[0,newh1], [w-2*r,newh1-h3], [w-2*r,h3]]);
        circle(r=r);
    }
}
*trapizoid2d();

module trapizoid3d(h1=40,h2=30,w=20,r=5,z) {
    h3=(h1-h2)/2;
    newh1 = h1-2*r;
    translate([r,r,0]) minkowski(convexity = 10) {
        linear_extrude(z,convexity=10)
            polygon(points = [[0,0],[0,newh1], [w-2*r,newh1-h3], [w-2*r,h3]]);
        sphere(r=r);
    }
}
*trapizoid3d();
*trapizoid3d(h1=3.75*MM,h2=3*MM,w=2.4375*MM,r=0.375*MM,z=10);

module door_handle(hole_space=2.5*MM) {
    height = 3.75*MM;
    hole_offset = (height-hole_space)/2;
    difference() {
        trapizoid3d(h1=height,h2=3*MM,w=2.5*MM,r=0.375*MM,z=0.7*MM); // outside
        translate([0.5*MM,0.5*MM,.3*MM]) // inside
            trapizoid3d(h1=2.75*MM,h2=2.2*MM,w=1.6*MM,r=0.2*MM,z=0.4*MM);
        translate([-1,0,-1*MM]) 
            cube([3*MM,4*MM,1*MM],center=false); // bottom
        translate([-1.2*MM,0,2.375*MM]) rotate([-90,0,0]) // top
            //cube(size=[3*MM,2*MM,4*MM],center=false);
            rounded_cube(size=[3*MM,2*MM,4*MM],r=0.7*MM,center=false,$fn=130);
        translate([0,hole_space/2+hole_offset,0]) 
            hole_pair (x = .19*MM,y=hole_space,d=.15*MM,h=2*0.376*MM,csk=true);
    }
}
color("grey") door_handle(hole_space=3.0*MM,$fn=48);
