// Two Wheel Balancing Robot
// Design and Analysis model
// by SrAmo  June, 2024
use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>
//use <ME_lib.scad>
use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

// use large value (~100) for printing, smaller (~40) for display
FACETS = $preview ? 100 : 150; // [40,100]

CG_Y = 90; // Y offset of Chassis Center of Gravity, mm
WGT_CHASSIS = 3;  // weight of the chassis, kg
WGT_WHEELS = 3;  // weight of the wheels and motors, kg
FORCE_SCALER = 20;  // for visual
WHEEL_DIA = 152; // mm

PITCH_ANG = 30; // *sin($t*90);


module WheelAssy(dWheel=152, tWheel=25.4, ) {
    // Wheel Andy Mark 
    color("blue") washer(d=dWheel,t=tWheel,d_pin=dWheel-2*tWheel,$fn=FACETS);
    color("grey") washer(d=dWheel-1,t=tWheel/2,d_pin=10,$fn=FACETS);

    // Motor REV UltraPlanetary Gearbox
    translate([0,0,70]) color("silver") washer(d=38,t=110,d_pin=0,$fn=FACETS);
}
*WheelAssy(WHEEL_DIA);

module ChassisAssy(pitchAngle=0,y=0) {
    rotate([0,0,pitchAngle]) {
        // represents REV Control Hub
        color("orange") translate([0,y,0]) 
            rotate([90,0,0])
                rounded_cube(size=[103,143,30],r=5,center=true);
        // represents plastic base
        color("white") translate([0,y-15,0]) 
            cube([120,6,150],center=true);

        // represents wheel mount
        color("darkblue") translate([0,y/2,0]) 
            cube([30,y,40],center=true);
    };
}
*ChassisAssy(0,CG_Y);

module TwoWheelRobotAssy(pitchAngle=0) {
    WheelAssy(WHEEL_DIA);
    translate([0,0,100]) ChassisAssy(pitchAngle,CG_Y);
}
TwoWheelRobotAssy(PITCH_ANG);

// Motor Free Body Diagram (balanced forces at the motor)
module MOTOR_FBD(pitchAngle=0) {
    Z_VIS = 200; // z offset for visulization
    scaled_force = WGT_CHASSIS*FORCE_SCALER;
    
    translate([0,0,Z_VIS]) {
        color ("red") translate([-CG_Y*sin(PITCH_ANG),CG_Y*cos(PITCH_ANG),0]) 
        force_arrow([0,0,0],[0,-scaled_force,0],mag=scaled_force);
        
        color ("green") translate([0,-scaled_force,0])
        force_arrow([0,0,0],[0,scaled_force,0],mag=scaled_force);

        torque_mag = -CG_Y*sin(PITCH_ANG)*WGT_CHASSIS;
        color("yellow") torque_arrow([0,0,0],mag=torque_mag);
        echo(pitchAngle=pitchAngle,torque_mag=torque_mag);
    }
}
*MOTOR_FBD(PITCH_ANG);

// Robot Force Diagram (NOT balanced! Robot accelerates horizontally)
module ROBOT_FORCES(pitchAngle=0) {
    Z_VIS = 200; // z offset for visulization
    scaled_force = (WGT_CHASSIS+WGT_WHEELS)*FORCE_SCALER;
    
    COMBINED_CG = CG_Y/2;
    
    translate([0,0,Z_VIS]) {
        color ("darkred") translate([-COMBINED_CG*sin(PITCH_ANG),COMBINED_CG*cos(PITCH_ANG),0]) 
        force_arrow([0,0,0],[0,-scaled_force,0],mag=scaled_force);
        
        color ("darkgreen") translate([0,-scaled_force-WHEEL_DIA/2,0])
        force_arrow([0,0,0],[0,scaled_force,0],mag=scaled_force);

        //torque_mag = -COMBINED_CG*sin(PITCH_ANG)*WGT_CHASSIS;
        //color("yellow") torque_arrow([0,0,0],mag=torque_mag);
        //echo(pitchAngle=pitchAngle,torque_mag=torque_mag);
    }
}
*ROBOT_FORCES(PITCH_ANG);