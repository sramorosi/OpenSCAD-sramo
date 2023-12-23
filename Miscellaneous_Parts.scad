// Miscellanious parts
//    Rack Support for Clara's Freezer
//    Damper for vacuum cannon
//    Screen Door Handel for Juniper
//    Berry Sieve 
//    Spice Drawer Model
//    Nut Covers for human turntable (June 2023)
//
// by SrAmo  July, 2023
use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>

// mm in an inch. Don't change.
MM = 25.4;

/*
$vpr = [60, 0, -360*$t];   // view point rotation (spins the part)
$vpt = [0,0,8];    // view point translation
$vpf = 50;          // view point field of view
$vpd = 90;         // view point distance
*/
// use large value (~100) for printing, smaller (~40) for display
FACETS = $preview ? 120 : 100; // [40,100]

// Die, by SRA, 11/24/2023
module die(size = 18) {
    rad=0.08*size;  // edge radius
    flat=size-2*rad;  // account for radius
    divotD = 0.18*size; // diameter of divot
    d = divotD*1.3; // divot from edge
    SphereRad = size*0.73;
    STRANS = size/2;
    color("white") 
    difference() {
        intersection() {
            translate([rad,rad,rad]) 
                minkowski(convexity=20) {
                    cube([flat,flat,flat]);
                    sphere(r=rad);
                };
            translate([STRANS,STRANS,STRANS]) sphere(r=SphereRad);
            }
            
        translate([size/2,size/2,0]) sphere(d=divotD); // 1 
        
        translate([d,0,size-d]) sphere(d=divotD); // 2
        translate([size-d,0,d]) sphere(d=divotD); // 2 
        
        translate([0,size/2,size/2]) sphere(d=divotD); // 3 
        translate([0,d,size-d]) sphere(d=divotD); // 3
        translate([0,size-d,d]) sphere(d=divotD); // 3 

        translate([size,d,size-d]) sphere(d=divotD); // 4
        translate([size,size-d,d]) sphere(d=divotD); // 4 
        translate([size,d,d]) sphere(d=divotD); // 4
        translate([size,size-d,size-d]) sphere(d=divotD); // 4 

        translate([size/2,size,size/2]) sphere(d=divotD); // 5 
        translate([d,size,size-d]) sphere(d=divotD); // 5
        translate([size-d,size,d]) sphere(d=divotD); // 5 
        translate([d,size,d]) sphere(d=divotD); // 5
        translate([size-d,size,size-d]) sphere(d=divotD); // 5 

        translate([d,size-d,size]) sphere(d=divotD); // 6
        translate([size-d,d,size]) sphere(d=divotD); // 6 
        translate([d,d,size]) sphere(d=divotD); // 6
        translate([size-d,size-d,size]) sphere(d=divotD); // 6
        translate([size/2,d,size]) sphere(d=divotD); // 6
        translate([size/2,size-d,size]) sphere(d=divotD); // 6
    }
}
die($fn=100);

// Lantern Handle, by SRA, 11/20/2023
module LanternHandle() {
    dia1 = 12; // diameter of the cylinder
    bendRad = 10; // radius of the bends
    openingW = 87; // opening Width
    secondL = 30; // second length
    
    module bend(rot1 = 180, radOfCurve = 20, d = 5){
        rotate_extrude(angle=rot1,convexity = 20)
        translate([radOfCurve, 0, 0])
            circle(d=d); // on X,Z plane

    };
    
    module halfHandle() {
        translate([-bendRad,openingW/2,0]) 
            bend(90,bendRad,dia1);
        
        translate([-bendRad-secondL/2,openingW/2+bendRad,0]) 
            rotate([0,90,0]) 
                cylinder(secondL,d=dia1,center=true);
        
        translate([-bendRad-secondL,openingW/2,0]) 
        rotate([0,0,90]) bend(90,bendRad,dia1);
        
        translate([-2*bendRad-secondL,openingW/2,0]) 
        rotate([-90,0,0]) clip();

    };
    
    module clip() {
        translate([0,0,-7]) cylinder(h=7,d=dia1,center=false);
        translate([0,0,-11])
        difference() {
            union() {
                translate([0,0,-0.7]) cylinder(h=5,d=8);
                simpleTorus (bigR = 4, littleR = 0.7);
            };
            rotate([90,0,0]) 
                rounded_cube([4,7.8,20],r=1.8);
        }
    };
    
    // Main
    halfHandle();
    rotate([90,0,0]) cylinder(openingW,d=dia1,center=true);
    mirror([0,1,0]) halfHandle();
}
*LanternHandle($fn=80);

// X-Mas Lightbulb Hook
module C7hook() {
    thickness = 3;
    height = 10;
    rotation1 = 120;
    radius1 = 8;
    rotation2 = -90;
    radius2 = 6;
    clipW = 11;
    clipR = 2;
    baseH = 14;

    module half_hook(r1=10,r2=10) {
        curved_beam(rot1 = rotation1, radOfCurve = r1, t1 = thickness, t2 = height);
        rotate([0,0,rotation1-180]) // rotation1+180
            translate([-r1-r2-thickness,0,0])
            curved_beam(rot1 = rotation2, radOfCurve = r2, t1 = thickness, t2 = height);
    }
    
    module full_hook(r1=10,r2=10) {
        half_hook(r1,r2);
        mirror([0,1,0]) half_hook(r1,r2);
    }
 
     module full_clip(r1=10,r2=10) {
        translate([-r1,clipW/2,0]) half_hook(r1,r2);
        translate([thickness/2,0,height/2]) 
            cube([thickness,clipW,height],center=true);
        translate([-r1,-clipW/2,0]) mirror([0,1,0]) half_hook(r1,r2);
    }
    union() {  
    // C7 hook
    translate([-radius1,0,0]) full_hook(r1=radius1,r2=radius2);
    
    // backside of C7 hook
    translate([radius1,0,5])
        rounded_cube(size=[2*radius1,1.4*radius1,10],r=3,center=true);
    
    // ground clip
    translate([thickness,0,-baseH]) rotate([0,0,180]) 
        full_clip(r1=clipR,r2=clipR);
    
    // C7 base tab
    translate([-radius1/2+1,0,-baseH+2])
        rounded_cube(size=[radius1,1.4*radius1,4],r=1,center=true);
    
    // connection cube
    translate([thickness/2,0,-(baseH-height)/2]) // (baseH-height)/2
        cube([thickness,14,baseH+height],center=true);
    }
}
*C7hook($fn=80);

// spacers for juniper duck wings
*washer(d=0.8*MM,t=3,d_pin=0.65*MM,$fn=FACETS);

// Spirograph from https://openscadsnippetpad.blogspot.com/2017/06/flower-shape-path-spirograph.html
// r1 = planet orbit radius, r2 = moon orbit radius, r3 = point on moon surface, v1 & v2 = phase
function flower(r1=100,r2,r3,v1,v2 ,steps=200)=
[ for(t=[0:1/steps:1])
let(
x =r1* sin(t*360) +r2* sin(t*v1*360)+r3* sin(t*v2*360),  
y =r1*cos(t*360) +r2*cos(t*v1*360)  +r3*cos(t*v2*360)   )
[x,y] ];

// function close adds a last point equal to the first
function close(p)= concat(p,[p[0]]);
 
function rnd(a = 1, b = 0, s = []) = s == [] ? 
(rands(min(a, b), max(
  a, b), 1)[0]) : (rands(min(a, b), max(a, b), 1, s)[0]);

function un(v) = v / max(1e-15, norm(v)); // just for color

// draw a random spirograph each time preview is run, with random color
*color(un([round(rnd(3)) ,round(rnd(3))  *0.75 ,round(rnd(3))  ])) {
    paramas=[70,rnd(30),rnd(15),round(rnd(3,17))*(round(rnd(-1,1))==1?1:-1),round(rnd(3,20)*(round(rnd(-1,1))==1?1:-1))  ,300];
    echo(paramas);
    p=close(flower(paramas[0],paramas[1],paramas[2],paramas[3],paramas[4],paramas[5]));
    polygon(p);
}
// ECHO: [70, 21.827, 9.88683, 14, -11, 300]   nice one
p=close(flower(70, 21.827, 9.88683, 14, -11, 300));
*polygon(p);

// Shower Rack Bumper
module showerClip(ID = 2, OD = 10, H = 5) {
    linear_extrude(H,convexity=10)
        difference() {
            circle(d=OD);
            circle(d=ID);
            polygon([[-ID/2,0],[OD/1.8,ID/1.0],[OD/1.8,-ID/1.0]]);
        }
}
*showerClip(ID=6.4,OD=16,H=6,$fn = FACETS);

// Freezer Rack Support for Clara & Adam.  Units are in MM
RACK_WIRE_D = 7.5;  // measured 7.3.  Add 2% for shrinkage
EXT_L = 20.0;
EXT_T = 18.0;

module rackSupport() {
    color("lightblue")
    difference() {
        // Add main profile
        linear_extrude(EXT_T,center=true,convexity=10) {
            rackProfile2D();
            mirror([0,1,0]) rackProfile2D();
        }
        
        // Subtract z-direction wire
        translate([EXT_L + RACK_WIRE_D/2,0,0]) 
            cylinder(1.2*EXT_T,d=RACK_WIRE_D,center=true);
        
        // Subtract x-direction wire
        translate([EXT_L + RACK_WIRE_D/2,0,0]) rotate([0,90,0])
            cylinder(1.2*EXT_L,d=RACK_WIRE_D,center=false);
        
        // Subtract wire intersection triangle
        X1 = RACK_WIRE_D/2 + 5;
        translate([EXT_L + RACK_WIRE_D/2,0,0]) rotate([90,0,0])
            linear_extrude(height = RACK_WIRE_D, center=true, convexity=10)
                polygon(points=[[X1,0],[0,X1],[0,-X1]]);

        // Subtract opening
        w = RACK_WIRE_D*0.8;
        translate([EXT_L + RACK_WIRE_D/2,-w/2,-EXT_T*0.6])
            cube([EXT_L*2,w,EXT_T*1.2],center=false);
        
        // Subtract zip-tie torus
        translate([EXT_L*2,0,0]) rotate([0,90,0])
            simpleTorus (bigR = EXT_T/2+4, littleR = 3);
    }
    
    module rackProfile2D() { // Outside profile of rack support
        X1 = 2*EXT_L+RACK_WIRE_D;
        polygon(points = [[0,0],[X1-RACK_WIRE_D/2,0],[X1,RACK_WIRE_D/2],[X1,EXT_T/2],[EXT_L,EXT_T/2],
        [10,5],[2,5],[0,3]]);
    }
}
*rackSupport($fn = FACETS);

*difference() {
    rackSupport($fn = FACETS);
    translate([-10,0,-50]) cube([100,100,100]);
}


module NutCover(sphereD=MM,topH=10,nutH=4) { // for windsurf training table
    difference() {
        translate([0,0,-sphereD/2 + topH]) sphere(d=sphereD); // sphere
        translate([-50,-50,-100]) cube(100); // removes eveything below z=0
        cylinder(h=100,d=4.8,center=true); // hole for no 10 screw
        translate([0,0,-.01]) 
            RoundedWasher(d=11.5,t=nutH,fillet=2.7); // space for nut
    }
}
*NutCover(sphereD=1.1*MM,topH=12.5,nutH=5.5,$fn=FACETS); // Top
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
//echo("TOTAL JARS = ",NX*NY);
T_WOOD = 1.75*MM;  // 1.75 INCH
xy_adjust = SPICE_JAR_DIA/2;
z_adjust = (SPICE_JAR_DIA/2)*sin(TILT);
//y_space = SPICE_JAR_DIA + tan(TILT)*SPICE_JAR_HEIGHT/2;
y_space = T_WOOD+T_WOOD*tan(TILT)+22;
//echo(y_space=y_space);

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