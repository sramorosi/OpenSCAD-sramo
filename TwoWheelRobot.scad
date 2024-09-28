// Two Wheel Balancing Robot
// Design and Analysis model
// by SrAmo  , 2024
use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>
//use <ME_lib.scad>
use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

// use large value (~100) for printing, smaller (~40) for display
FACETS = $preview ? 100 : 150; // [40,100]

WHEEL_DIA = 152; // mm

PITCH_ANG = -30; // *sin($t*90);

PET_DENSITY = 1.39/1000;  // grams/mm^3
POLYCARB_DENSITY = 1.20/1000;  // grams/mm^3

CUBE_ARRAY = [[["NAME","COLOR","DENSITY"],["X_Size","Y_Size","Z_Size"],["X_cg","Y_cg","Z_cg"]]];
echo(CUBE_ARRAY=CUBE_ARRAY);

STRUT_L = 90; // LENGTH OF STRUT, mm
STRUT_DIM = [30,STRUT_L,30];
STRUT_POS = [0,STRUT_L/2,0];
STRUT_MASS = cube_mass(STRUT_DIM,POLYCARB_DENSITY/2); 
STRUT_CG = cube_centroid(STRUT_DIM,STRUT_POS);
echo(STRUT_MASS=STRUT_MASS,STRUT_CG=STRUT_CG," NO ROTATION");

PLATE_DIM = [120,6,150];
PLATE_POS = [0,STRUT_L+PLATE_DIM[1]/2,0];
PLATE_MASS = cube_mass(PLATE_DIM,POLYCARB_DENSITY); 
PLATE_CG = cube_centroid(PLATE_DIM,PLATE_POS);
echo(PLATE_MASS=PLATE_MASS,PLATE_CG=PLATE_CG," NO ROTATION");

HUB_DIM = [103,30,143];   // FROM REV SPECS
HUB_POS = [0,STRUT_L+PLATE_DIM[1]+HUB_DIM[1]/2,0];   // 
HUB_MASS = cube_mass(HUB_DIM,PET_DENSITY/3); // REV HUB Spec says 209 grams
HUB_CG = cube_centroid(HUB_DIM,HUB_POS);
echo(HUB_MASS=HUB_MASS,HUB_CG=HUB_CG," NO ROTATION");

PHONE_DIM = [10,75,150];   
PHONE_POS = [0,STRUT_L+PLATE_DIM[1]+HUB_DIM[1]+PHONE_DIM[1]/2,0]; 
PHONE_MASS = cube_mass(PHONE_DIM,PET_DENSITY/0.9); // measured 165 grams
PHONE_CG = cube_centroid(PHONE_DIM,PHONE_POS);
echo(PHONE_MASS=PHONE_MASS,PHONE_CG=PHONE_CG," NO ROTATION");


CHASSIS_MASS = STRUT_MASS+PLATE_MASS+HUB_MASS+PHONE_MASS;  // MASS of the chassis, g
echo(CHASSIS_MASS=CHASSIS_MASS," g");

// Weighted average of the centroid positions of each cube based on their mass
CHASSIS_CG = rot_z( (STRUT_CG*STRUT_MASS + PLATE_CG*PLATE_MASS + HUB_CG*HUB_MASS + PHONE_CG*PHONE_MASS)/CHASSIS_MASS,PITCH_ANG);
echo(CHASSIS_CG=CHASSIS_CG,"at ",PITCH_ANG=PITCH_ANG);

ARM = CHASSIS_CG[0]*cos(PITCH_ANG)-CHASSIS_CG[1]*sin(PITCH_ANG);
MTR_TORQUE = CHASSIS_MASS*ARM;
echo(MTR_TORQUE=MTR_TORQUE,"g-mm at ",PITCH_ANG=PITCH_ANG);

WGT_WHEELS = 3;  // weight of the wheels and motors, kg
FORCE_SCALER = 1;  // for visual

function cube_mass(size=[1,2,3],density=1) =
    size[0]*size[1]*size[2]*density;

// Function to calculate the centroid of a single cube
function cube_centroid(size, position) = position;// + size/2;

module WheelAssy(dWheel=152, tWheel=25.4, ) {
    // Wheel Andy Mark 
    color("blue") washer(d=dWheel,t=tWheel,d_pin=dWheel-2*tWheel,$fn=FACETS);
    color("grey") washer(d=dWheel-1,t=tWheel/2,d_pin=10,$fn=FACETS);

    // Motor REV UltraPlanetary Gearbox
    translate([0,0,70]) color("silver") washer(d=38,t=110,d_pin=0,$fn=FACETS);
}
*WheelAssy(WHEEL_DIA);

module ChassisAssy(pitchAngle=0) {
    rotate([0,0,pitchAngle]) {
        // represents STRUT
        color("darkblue") translate(STRUT_POS) 
            cube(STRUT_DIM,center=true);
        
        // represents plastic base
        color("white") translate(PLATE_POS) 
            cube(PLATE_DIM,center=true);

        // represents REV Control Hub
        color("orange") translate(HUB_POS) 
                cube(size=HUB_DIM,center=true);
        
        // represents phone
        color("grey") translate(PHONE_POS) 
            cube(PHONE_DIM,center=true);


        
    };
    // represents CG
    color("red") translate(CHASSIS_CG) sphere(d=40);

}
*ChassisAssy(0);

module TwoWheelRobotAssy(pitchAngle=0) {
    translate([0,0,-100]) WheelAssy(WHEEL_DIA);
    ChassisAssy(pitchAngle);
}
TwoWheelRobotAssy(PITCH_ANG);


function rot_z(pt,ang) = 
   [pt[0]*cos(ang)-pt[1]*sin(ang),pt[0]*sin(ang)+pt[1]*cos(ang),pt[2]];

// CHASSIS Force Diagram 
module CHASSIS_FORCES(pitchAngle=0) {
    Z_VIS = 0; // z offset for visulization
    scaled_force = (CHASSIS_MASS)*FORCE_SCALER;
      
    translate([0,0,Z_VIS]) {
        color ("darkred") translate(CHASSIS_CG) 
        force_arrow([0,0,0],[0,-scaled_force,0],mag=scaled_force);
        
        color ("darkgreen") 
        torque_arrow(to=[0,0,0],mag=MTR_TORQUE/100);
    }
}
*CHASSIS_FORCES(PITCH_ANG);
