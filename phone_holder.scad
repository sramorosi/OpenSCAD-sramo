// Phone Charging Holder
// by SrAmo, 12/15/2021

height = 45;
cut = 15;
difference() {
    cube([50,50,height],center=true);
    
    translate([0,0,height/2-10+2.5]) cube([10.7,8.5,20],center=true);
    cube([8.4,5.9,height+5],center=true);
    translate([0,0,-height/2]) rotate([90,45,0]) cube([cut,cut,55],center=true);
    translate([0,0,-height/2]) rotate([0,90,0]) rotate([0,0,45]) cube([cut,cut,55],center=true);
    translate([25,0,0]) cube([50,4,50],center=true);
    }