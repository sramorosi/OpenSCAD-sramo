// FTC Auto Planning Tool
// Skystone field
// SrAmo June, 2023

module field() {
    color("grey") cube([141,141,1],center=true);
    
    color("blue") 
        translate([-141/2+4+34.5/2,-18.5/2-4,1]) 
        cube([34.5,18.5,2],center=true);
    
    color("red") 
        translate([-141/2+4+34.5/2,18.5/2+4,1]) 
        cube([34.5,18.5,2],center=true);
    
    color("yellow") {
        translate([24,-24-0,0]) cube([7.8,4,6],center=false);
        translate([32,-24-0,0]) cube([7.8,4,6],center=false);
        translate([40,-24-0,0]) cube([7.8,4,6],center=false);
        translate([48,-24-0,0]) cube([7.8,4,6],center=false);
        translate([56,-24-0,0]) cube([7.8,4,6],center=false);
        translate([62,-24-0,0]) cube([7.8,4,6],center=false);
    }
    *color("yellow") 
        translate([0,0,0]) cube([8,4,6],center=false);
    
}
module robot() {
    color("silver") // Chassis
        translate([0,0,2]) cube([17.5,17.5,4],center=true);
    color("green") { // intakes
        translate([-7.5,4,4]) cylinder(h=8,d=3,center=true,$fn=32);
        translate([-7.5,-4,4]) cylinder(h=8,d=3,center=true,$fn=32);
    }
}
field();

translate([-36,-62,0]) rotate([0,0,90]) robot(); // startPose

translate([-56,-30,0]) rotate([0,0,90]) robot(); // PLAT_POINT

translate([-40,-55,0]) rotate([0,0,180]) robot(); // PLAT_POINT_2

translate([2,-38,0]) rotate([0,0,180]) robot(); // MID_POINT

translate([23,-24,0]) rotate([0,0,225]) robot(); // BLOCK_POINT