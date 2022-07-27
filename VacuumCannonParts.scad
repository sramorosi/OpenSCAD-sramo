// Damper for vacuum cannon
// by SrAmo  July, 2022
use <Robot_Arm_Parts_lib.scad>

module piston() {
    difference () {
        washer(d=22.4,t=16,d_pin=4.5,$fn=96);
        cylinder(h=12,d=15,center=false,$fn=48);
    }
}    
piston();

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
translate([0,0,50]) cap(shaft=false);
translate([0,0,-50]) rotate([180,0,180]) cap(shaft=true);

color("grey",0.5) washer(d=25.6,d_pin=22.6,t=100,$fn=48);