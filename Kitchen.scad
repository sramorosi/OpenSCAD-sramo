//
// by SrAmo  July, 2023
use <Robot_Arm_Parts_lib.scad>
//use <NEW_LDB_Modules.scad>

// mm in an inch. Don't change.
MM = 25.4;

// origin is NW corner, X is east, Y is north, z is up
// units are in inches
// 32 inch from floor to bottom of windows
// 60 inch north wall to door frame

color("lightyellow") { // walls
    // North wall, to door
    cube([60,4,32]);

    // West wall to counter
    translate([-4,-79,0]) cube([4,79+4,32]);

    // South counter
    translate([0,-4-79,0]) cube([55,4,32]);
};

color("chocolate") { // table
    translate([0,-40-16,28]) {
        cube([60,32,1]); // table top
        translate([50,16,-14]) 
            cube([4,4,28],center=true); // post
    };
};

color("sandybrown") { // bench
    translate([0,-24,15]) 
        cube([60,24,1]); // seat
    translate([0,-5,16]) 
        rotate([-13,0,0]) 
            cube([60,1,16]); // back rest
    translate([0,-18,0]) 
        rotate([17,0,0]) 
            cube([60,1,16]); // kick
    
};