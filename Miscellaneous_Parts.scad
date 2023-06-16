// Miscellanious parts
//    Damper for vacuum cannon
//    Screen Door Handel for Juniper
//    Berry Sieve 
//    Spice Drawer Model
//    Nut Covers for human turntable (June 2023)
//
// by SrAmo  January, 2023
use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>

// mm in an inch. Don't change.
MM = 25.4;
// use 140 for printing, 40 for display
FACETS = 40; // [40,100]

module NutCover(sphereD=MM,topH=10,nutH=4) { // for windsurf training table
    difference() {
        translate([0,0,-sphereD/2 + topH]) sphere(d=sphereD); // sphere
        translate([-50,-50,-100]) cube(100); // removes eveything below z=0
        cylinder(h=100,d=4.8,center=true); // hole for no 10 screw
        translate([0,0,-.01]) 
            RoundedWasher(d=11.5,t=nutH,fillet=2.7); // space for nut
    }
}
NutCover(sphereD=1.1*MM,topH=12.5,nutH=5.5,$fn=FACETS); // Top
*NutCover(sphereD=1.1*MM,topH=8.5,nutH=6.5,$fn=FACETS); // Bottom

module piston() {
    difference () {
        washer(d=22.4,t=16,d_pin=4.5,$fn=96);
        cylinder(h=12,d=15,center=false,$fn=48);
    }
}    

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
DAMPER_LEN = 200;
module DamperPistonAssy() {
    rotate([-90,0,0]) {
    translate([0,0,DAMPER_LEN/2]) cap(shaft=false);
    translate([0,0,-DAMPER_LEN/2]) rotate([180,0,180]) cap(shaft=true);
    piston();
    color("grey",0.5) washer(d=25.6,d_pin=22.6,t=DAMPER_LEN,$fn=48);
    }
}
*DamperPistonAssy(); // for vacuum cannon (with tennis balls)
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

module door_handle(inside=true) {
    // outside door = 2.5 inch hole space
    // inside door = 3.0 inch hole space
    // inside door has rebate under screws
    hole_space = inside ? 3.0*MM :  2.5*MM;
    inside_r = inside ? 0.4*MM : 0.2*MM;
    height = 3.75*MM;
    hole_offset = (height-hole_space)/2;
    difference() {
        trapizoid3d(h1=height,h2=3*MM,w=2.5*MM,r=0.375*MM,z=0.7*MM); // outside
        translate([0.5*MM,0.5*MM,inside_r+0.1*MM]) // inside
            trapizoid3d(h1=2.75*MM,h2=2.2*MM,w=1.6*MM,r=inside_r,z=0.5*MM-inside_r);
        translate([-1,0,-1*MM]) 
            cube([3*MM,4*MM,1*MM],center=false); // bottom
        translate([-1.2*MM,0,2.375*MM]) rotate([-90,0,0]) // top
            //cube(size=[3*MM,2*MM,4*MM],center=false);
            rounded_cube(size=[3*MM,2*MM,4*MM],r=0.7*MM,center=false,$fn=130);
        translate([0,hole_space/2+hole_offset,0]) 
            hole_pair (x = .19*MM,y=hole_space,d=.15*MM,h=2*0.376*MM,csk=true);
        if (inside) {
            translate([-.1,-.1,-.1]) cube([0.5*MM,4*MM,0.25*MM],center=false);
        }
    }
}
*color("grey") door_handle(inside=true,$fn=48);

module berry_sieve() {
    berryHole = 7.5; // size for berry.  7.5 is a bit small. Go to 8 mm?
    mesht = 0.5;  // mesh thickness
    comb = berryHole+mesht;
    thk=4;  // thickness of sieve
    union() {
        difference() {
            cylinder(h=thk,d=168,center=true);
            translate([-100,-200,0]) for (i= [1:30]) {
                for (j=[1:30]) {
                    translate([i*(berryHole-mesht),j*comb+(i-1)*comb/2,-2*thk]) cylinder(h=4*thk,d=berryHole,$fn=6);
                }
            }
        }
        translate([0,0,thk/2]) washer(d=180,t=2*thk,d_pin=167,$fn=96);
    }
}
*berry_sieve();

// Spice Jar Tilt (Deg)
TILT = 35;  // [0:1:80]

SPICE_JAR_DIA = 52;
SPICE_JAR_HEIGHT = 115;
NX = 7;  // number of jars in x 
NY = 5;  // number of jars in y
echo("TOTAL JARS = ",NX*NY);
T_WOOD = 1.75*MM;  // 1.75 INCH
xy_adjust = SPICE_JAR_DIA/2;
z_adjust = (SPICE_JAR_DIA/2)*sin(TILT);
//y_space = SPICE_JAR_DIA + tan(TILT)*SPICE_JAR_HEIGHT/2;
y_space = T_WOOD+T_WOOD*tan(TILT)+22;
echo(y_space=y_space);

module spice_container() {  // Simplistic model, represents largest spice jar
    // Max = diameter 52 mm (could be square), height 115 mm.
    // Cap = diameter 45 mm, thk = 11 mm
    color("cornsilk") union() {
        cylinder(h=SPICE_JAR_HEIGHT-11,d=SPICE_JAR_DIA,$fn=FACETS);
        translate([0,0,SPICE_JAR_HEIGHT-11]) cylinder(h=11,d=45,$fn=FACETS);
    }
};
*spice_container();
module drawer() {  // Simplistic model of spice drawer
    // Inside = 384mm x 500 mm x 145 mm
    // wall thickness = 15 mm typical
    color("Goldenrod") difference() {
        translate([-15+20,-15,-15]) cube([384+0,500+30,145+15], center=false);
        cube([384+20,500,145+10], center=false);
    };
};

module rectPattern(nx=2,x=10,ny=3,y=20) { // translate obects in rectangular pattern
    echo("OBJECTS=",nx*ny);
    for (i=[0:1:nx-1]) {
        for (j=[0:1:ny-1]) {
            translate([i*x,j*y,0]) children();
        };
    };
}

module spice_holder() {  // single spice holder
    difference() {
        cube([SPICE_JAR_DIA,y_space,T_WOOD],center=false);
        //translate([xy_adjust,xy_adjust,z_adjust])
                //spice_container();
            translate([-1,0,T_WOOD*tan(TILT)]) 
                rotate([-TILT,0,0]) 
                    translate([0,-T_WOOD,0])
                    cube([SPICE_JAR_DIA+4,T_WOOD/cos(TILT)+T_WOOD,T_WOOD*2],center=false);
        
    };
};
*spice_holder();

module spice_drawer_assy() { // simple assembly of spice drawer
    color("red") drawer();
    
    translate([0,T_WOOD,0]) 
        rectPattern(nx=NX,x=SPICE_JAR_DIA,ny=NY,y=y_space) 
            rotate([-TILT,0,0]) 
                translate([SPICE_JAR_DIA/2,-SPICE_JAR_DIA/2,0])
                spice_container();
    
    rectPattern(nx=NX,x=SPICE_JAR_DIA,ny=NY,y=y_space) 
            spice_holder();
    
    //rotate([-TILT,0,0]) spice_container(); // single spice container for debug

};
*spice_drawer_assy();