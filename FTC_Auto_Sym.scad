// FTC Auto Planning Tool
// Skystone field
// SrAmo June, 2023

FIELD_WIDTH = 141;  // inches
ONE_TILE_WIDTH = FIELD_WIDTH/6;

module tile() {
    color("grey") cube([ONE_TILE_WIDTH,ONE_TILE_WIDTH,1],center=false);
}
module field_tiles() {
    for (i=[-3:1:2]) {
        for (j=[-3:1:2]) {
            translate([i*ONE_TILE_WIDTH,j*ONE_TILE_WIDTH,0])
                scale([.98,.98,1]) tile();
        }
    }
}
module quarry() {
    color("yellow") {
        translate([24,-24-0,0]) cube([7.8,4,6],center=false);
        translate([32,-24-0,0]) cube([7.8,4,6],center=false);
        translate([40,-24-0,0]) cube([7.8,4,6],center=false);
        translate([48,-24-0,0]) cube([7.8,4,6],center=false);
        translate([56,-24-0,0]) cube([7.8,4,6],center=false);
        translate([62,-24-0,0]) cube([7.8,4,6],center=false);
    }

}
module field() {
    field_tiles();
    
    color("blue") // foundation
        translate([-141/2+4+34.5/2,-18.5/2-4,1]) 
            cube([34.5,18.5,2],center=true);

    color("blue") // foundation moved
        translate([-141/2+13,-48,1]) 
            cube([18.5,34.5,2],center=true);
    
    color("red") // foundation
        translate([-141/2+4+34.5/2,18.5/2+4,1]) 
            cube([34.5,18.5,2],center=true);
    
    quarry();
    mirror([0,1,0]) quarry();
}
module robot() {
    color("silver") // Chassis
        difference() {
            translate([0,0,2]) cube([17.5,17.5,4],center=true);
            translate([-6,0,3]) cube([8,8,6],center=true);
        }

    color("green") { // intakes
        translate([-7.5,4,4]) cylinder(h=8,d=3,center=true,$fn=32);
        translate([-7.5,-4,4]) cylinder(h=8,d=3,center=true,$fn=32);
    }
}
// The following lines draw the objects
field();

translate([-36,-62,0]) rotate([0,0,90]) robot(); // startPose
translate([-56,-30,0]) rotate([0,0,90]) robot(); // PLAT_POINT
translate([-40,-55,0]) rotate([0,0,180]) robot(); // PLAT_POINT_2
translate([2,-38,0]) rotate([0,0,180]) robot(); // MID_POINT
translate([23,-24,0]) rotate([0,0,225]) robot(); // BLOCK_POINT