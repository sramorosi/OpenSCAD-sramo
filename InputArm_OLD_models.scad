module single_offset_link (length=50,w=15,offset=7,d_pin=5,pot_short=false) {
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,-offset], Second joint is at [length,0]
    // link width is w (y dimension), constant along length of part
    // offset is the distance down from the x axis
    bend_ang = 45; // the angle of the bend
    long_leg = length-offset;
    short_leg = sqrt(2*offset*offset);
    echo(long_leg=long_leg,short_leg=short_leg);
    wire_hole_offset = offset+wire_hole_dia; // wire  hole offset
    difference () {
        union () {
            // Draw the main link
            dog_leg2 (d1=long_leg,ang=bend_ang,d2=short_leg,w=w,t=w);
            // pot bracket
            if(pot_short) { // short = AB arm
                translate([0,-offset,0]) 
                    rotate([0,0,bend_ang]) 
                        translate([w/3,0,w/1.2]) 
                            cube([w,w,w/1.2],center=true);
                } else { // BC arm
                    translate([length-w/3,0,w/1.2]) cube([w,w,w/1.2],center=true);
                }
            };
            // remove wire holes (same for both)
            //   diagonal hole 1
            translate([length/4,widthAB-wire_hole_offset,1]) 
                rotate ([0,30,0]) 
                    translate([0,0,length/4]) 
                        wire_path(length/4.2);
            //   diagonal hole 1
            translate([length/1.3,widthAB-wire_hole_offset,1]) 
                rotate ([0,-60,0]) 
                    translate([0,0,length/3.5]) 
                        wire_path(length/4.5);
            if(pot_short) {  // potentiometer on short end, AB arm
                // remove long wire hole
                translate([length/3.3,widthAB-wire_hole_offset,1.5]) 
                    rotate ([0,87,0]) 
                        wire_path(length/2);
                // remove either side of lug on long end
                translate([length-w*0.7,-w/1.1,pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                translate([length-w*0.7,-w/1.1,-w-pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                // remove clevis on short end
                translate([0,-offset,0]) {
                    rotate([0,0,bend_ang]) 
                        cube([1.4*w,2.2*w,pot_lug_t+clevis_gap],center=true);
                   cylinder(h=2*w,d=d_pin,center=true);
                }
                // remove the potentiometer interfaces
                translate([0,-offset,widthAB/2+1]) 
                    rotate([180,0,180-45]) 
                           P090S_pot(negative=true);
                translate([length,0,widthAB/2]) 
                    rotate([180,0,90]) 
                        P090S_pot(negative=true);
                // donut holes on long end for wires
                translate([length,0,pot_lug_t/2.5]) 
                    rotate_extrude(convexity = 10, $fn = donut_fragments) {
                        translate([widthAB/1.3, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                        translate([widthAB/1.8, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                    }
            } else { // potentiometer on long end, BC arm
                // remove long wire hole
                translate([length/3.3,widthAB-wire_hole_offset,1.5]) 
                    rotate ([0,90,0]) 
                        wire_path(length/2);
                // remove either side of lug on short end
                translate([0,-offset,0]) rotate([0,0,bend_ang]) {
                translate([-w*0.7,-w/1.1,pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                translate([-w*0.7,-w/1.1,-w-pot_lug_t/2])
                    cube([1.4*w,2.2*w,w],center=false); 
                }
                // remove clevis on short end
                translate([length,0,0]) {
                        cube([1.4*w,2.2*w,pot_lug_t+clevis_gap],center=true);
                   cylinder(h=2*w,d=d_pin,center=true);
                }
                // remove the potentiometer interfaces
                translate([0,-offset,-widthAB/2]) 
                    rotate([0,0,135]) 
                           P090S_pot(negative=true);
                translate([length,0,widthAB/2+1]) 
                    rotate([180,0,-90]) 
                        P090S_pot(negative=true);
                // donut holes on short end for wires
                translate([0,-offset,pot_lug_t/2.5]) 
                    rotate_extrude(convexity = 10, $fn = donut_fragments) {
                        translate([widthAB/1.3, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                        translate([widthAB/1.8, 0, 0]) 
                            circle(d=wire_hole_dia*1.3, $fn = donut_fragments);
                    }
            }
    }
}
single_offset_link(length=lenAB,w=widthAB,offset=offset,d_pin=pot_shaft_dia,pot_short=true,$fn=donut_fragments); 

module BC_arm_assy(length=10,offset=2){
    // Create a Link on xy plane along the X axis 
    // First joint is at [0,-offset], Second joint is at [length,0]
//    $fa=$preview ? 6 : 1; // minimum angle fragment
//    $fs=0.01; // minimum size of fragment (default is 2)
    hook_ang=atan2(offset,length); // hook angle
    echo("BC",length=length,hook_ang=hook_ang,offset=offset);
    rotate([0,0,-hook_ang]) 
        translate([0,offset,0])  
            //single_offset_link 
            single_offset_link(length=length,w=widthAB,offset=offset,d_pin=pot_shaft_dia,pot_short=false,$fn=donut_fragments); 
}

module finger_ring(length=20,height=10,inside_dia=16) {
   // Finger Ring
   finger_width =6;
   lug (r=(inside_dia+4)/2,w=height,h=length,t=finger_width,d=inside_dia);
}
finger_ring();
module final_hand(length=14){
    // Claw that attaches to End Effector
//    $fa=$preview ? 6 : 1; // minimum angle fragment
//    $fs=0.05; // minimum size of fragment (default is 2)
    difference () {
        union () {  
            // LUG
            translate ([0,pot_lug_t/2,-length]) rotate([90,0,0]) 
                lug (r=widthAB/2,w=armt,h=length,t=pot_lug_t,d=.1);
            
            // CLEVIS
             translate ([armt/2,0,-length+1]) rotate([-90,0,90]) 
                lug (r=widthAB/2,w=widthAB,h=length,t=armt,d=.1);
            // pot bracket
            translate([length,0,-length*1.6]) cube([10,widthAB,widthAB],center=true); 
            // ADD the finger loop
            translate([-armt/2,5,-length]) 
                rotate([-55,0,0])
                rotate([0,90,0]) finger_ring(30,height=10,inside_dia=20);
            };
        // clevis remove slot
        translate([0,0,-length*2]) 
           cube([pot_lug_t+clevis_gap,length*2,length+4],center=true); 
        // remove lug end Potentiometer
        translate([0,-pot_lug_t,0]) 
               rotate([90,0,180]) P090S_pot(negative=true);
        // remove clevis end Potentiometer
        translate([armt/2,0,-length*2+1]) 
            rotate([90,0,-90]) P090S_pot(negative=true);
        // pin remove 
        translate([0,0,-2*length+1])
           rotate([0,-90,0]) cylinder(h=armt,d=pot_shaft_dia,center=false); 
        // remove donut hole for wire
        rotate([90,0,0]) rotate_extrude(convexity = 10, $fn = donut_fragments) 
            translate([widthAB/1.8, pot_lug_t/2, 0]) circle(d=wire_hole_dia*1.8, $fn = donut_fragments);
        } 
}
*final_hand();
module final_finger() {
    y_offset = 10;
    union() {
        difference() {
            lug (r=widthAB/2,w=10,h=y_offset,t=pot_lug_t,d=.1);
            // remove lug end Potentiometer
            translate([0,y_offset,pot_lug_t*1.5]) 
                rotate([0,180,0]) 
                    P090S_pot(negative=true);
        }
        translate([0,0,0])rotate([0,0,180]) 
            finger_ring(length=2*y_offset,height=10,inside_dia=20);
    }
}
final_finger();
module end_effector_assy() {
    translate([0,0,armt/2]) rotate([-90,0,90]) final_hand();
    translate([39,-4,10]) 
        rotate([90,90,180]) final_finger();
    }
module wire_path(length=30){
    cylinder(h=length*2,d=wire_hole_dia,center=true);
    translate([0,wire_hole_dia/1.9,0])
        cube([wire_dia,wire_hole_dia,length*2.2],center=true);
}
