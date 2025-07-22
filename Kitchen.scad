// Amorosi Kitchen Nook, Table and Bench
// by SrAmo  July, 2025
// Modeled in OpenSCAD.  Colors are just for visulization.
// origin is NW corner, X is east, Y is north, z is up
// units are in inches
F_BW = 32; // inch from floor to bottom of windows
NW_DF = 59; // inch north wall to door frame

// North wall, to door frame
color("lightyellow")
    cube([NW_DF,4,32]); // wall
color("chocolate") {
    translate([0,-0.75,F_BW])
        cube([32,0.75,2.5]); // lower window trim
    translate([29.5,-0.75,F_BW])
       cube([2.5,0.75,46]); // vertical window trim
    translate([NW_DF,-0.75,0])
       cube([1.5,0.75,83]); // vertical door trim
    };

// West wall to counter
color("LemonChiffon") 
    translate([-4,-79,0]) cube([4,79+4,32]);
color("chocolate") {
    translate([0,-68,F_BW])
        cube([0.75,68,2.5]); // lower window trim
     translate([0,-68,F_BW])
       cube([0.75,2.5,46]); // vertical window trim
    };

// South cabinet counter 
color("PapayaWhip")
    translate([0,-24-79,0]) cube([55.5,24,32]);

BENCH_WIDTH = 20; // used to position table

// TABLE
color("chocolate") { 
    THK = 1.5; // slab?
    LEN = 55.5; 
    WIDTH = 36;
    translate([0,-WIDTH-BENCH_WIDTH,28]) {
        cube([LEN,WIDTH,THK]); // table top
        translate([LEN-10,WIDTH/2,-14]) 
            cube([4,4,28],center=true); // post
    };
};

// BENCH
PLY_THK = 0.5;
BENCH_LEN = 55.5;  // Sized for two people
BENCH_SEAT_HGT = 18; // Height of seat before cushion
color("sandybrown") { 
    translate([0,-BENCH_WIDTH,BENCH_SEAT_HGT]) 
        cube([BENCH_LEN,BENCH_WIDTH,PLY_THK]); // seat
    translate([0,-3.6,BENCH_SEAT_HGT+PLY_THK]) 
        rotate([-13,0,0]) // BACK IS ROTATED 13 DEG
            cube([BENCH_LEN,PLY_THK,14]); // back rest
    translate([0,-14.5,0]) 
        rotate([17,0,0])  // KICK IS ROTATED 17 DEG
            cube([BENCH_LEN,PLY_THK,19]); // kick
};
// BENCH END COVER REPRESENTATION
color("tan") { 
    translate([BENCH_LEN,0,0]) 
    rotate([90,0,-90]) 
    linear_extrude(PLY_THK) 
        // Ploygon points start in x,y system, which is on the floor,
        // and need to be rotated per above rotate()
        polygon([[0,0],[0,F_BW],[sin(13)*13.5,BENCH_SEAT_HGT+PLY_THK],
        [BENCH_WIDTH,BENCH_SEAT_HGT+PLY_THK],[14,0]]);
};
