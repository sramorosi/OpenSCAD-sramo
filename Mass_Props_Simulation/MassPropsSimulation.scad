// Tools for calculating Mass Properties Design and 2D Dynamic analysis
// Also, Two Wheel Balancing Robot model
// by SrAmo  , 2025
// In physics, kinematics is the study of motion without considering the forces causing it,
// focusing on aspects like displacement, velocity, and acceleration, while dynamics examines
// the forces and torques that produce or change motion.
//
//use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>
//use <ME_lib.scad>
//use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

G = 980.665;  // cm/sec^2

// Object Member:
// Vector of objects OBJV: [[cube1],[cube2],[cyl1]], etc for each composite object
// Object=["NAME", "TYPE", "COLOR", density, X_Size, Y_Size, Z_Size, X_CM,Y_CM,Z_CM];
//           0       1       2        3         4       5       6      7    8    9
ON=0;
OT=1;
OC=2;
OD=3;
O1=4;
O2=5;
O3=6;
OX=7;
OY=8;
OZ=9;

// use large value (~100) for printing, smaller (~40) for display
FACETS = $preview ? 100 : 150; // [40,100]

PET_DENSITY = 1.39;  // grams/cm^3
POLYCARB_DENSITY = 1.20;  // grams/cm^3
ALUM_DEN = 2.72;  // grams/cm^3
/*
HUB_DIM = [103,30,143];   // FROM REV SPECS
HUB_POS = [0,STRUT_L+PLATE_DIM[1]+HUB_DIM[1]/2,0];   // 
HUB_MASS = cubeMass(HUB_DIM,PET_DENSITY/3); // REV HUB Spec says 209 grams

PHONE_DIM = [10,75,150];   
PHONE_POS = [0,STRUT_L+PLATE_DIM[1]+HUB_DIM[1]+PHONE_DIM[1]/2,0]; 
PHONE_MASS = cubeMass(PHONE_DIM,PET_DENSITY/0.9); // measured 165 grams
*/

/*
 * Wobble Wheel Test Data
 * Each Grey wheel, m = 270 g, dia = 15.5 cm
 * brass weight = 50 by 25 by 42 mm,  m = 522 g, center of mass offset = 40 mm
 * Test results, with roll from dead stop:
 * Initial angle = 30 deg, time to first reverse = 1.37 sec, distance = ~400 cm
 *                         time to second reverse = 1.25 sec, less distance
 */
WHEEL_DIA = 15.5; // cm
RAD_WHEEL = WHEEL_DIA/2;
STRUT_L = 9; // LENGTH OF STRUT, cm
// Oject v=["NAME", "TYPE", "COLOR", densi,X_Size,Y_Size,Z_Size,X_CM,Y_CM,Z_CM];
STRUT_OBJ =["BRASS","CUBE","gold"      ,9.94  ,5.0     ,2.5 ,4.2     ,0,4.0  ,0];
//PLATE_OBJ =["PLATE","CUBE","white"   ,1.2 ,12    ,1.6    ,15    ,0,STRUT_L+0.8,0];
WHEEL1_OBJ =["WHEEL1","CYL" ,"grey"    ,0.56 ,WHEEL_DIA ,2.54,0    ,0,0          ,-1.5];
WHEEL2_OBJ =["WHEEL2","CYL" ,"grey"    ,0.56 ,WHEEL_DIA ,2.54,0    ,0,0          ,1.5];

// OBJV = OBJECT VECTOR, or OBJECTS
OBJECTS = [STRUT_OBJ,WHEEL1_OBJ,WHEEL2_OBJ];

OBJM = objMass(OBJECTS);
echo(str("TOTAL MASS = ",OBJM));

// Find weighted average center of mass Objects in each direction
OBJCMx = objCMx(OBJECTS)/OBJM;
OBJCMy = objCMy(OBJECTS)/OBJM;
OBJCMz = objCMz(OBJECTS)/OBJM;
OBJCM = [OBJCMx,OBJCMy,OBJCMz];
echo(str("CENTER OF MASS, NO TRANSFORMATION  = ",OBJCM));

OBJI = objIzz(OBJECTS); //  defaults to around [0,0,0] 
echo(str("TOTAL MOMENT OF INERTIA Izz = ",OBJI));

// Simplify variables, in preparation for multiple objects
M1 = OBJM;
I1 = OBJI;
CM1 = OBJCMy;

// Simple pendulum frequency = 2*PI*sqrt(L/G)
SimpleFreq = 2*PI*sqrt(CM1/G);
echo(str("Simple Frequency = ",SimpleFreq," seconds"));
// Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr))
CompoundFreq = 2*PI*sqrt(I1/(M1*G*CM1));
echo(str("Compound Frequency = ",CompoundFreq," seconds"));

// Function for calculating Mass Properties of objects,
// such as mass, moment of inertias, center of mass (CM).

// Recursive function to add mass of objects    
function objMass(OBJV,i=0) = 
    OBJV[i]==undef ? 0 : 
        let (m= OBJV[i][OT]=="CUBE" ? cubeMass(OBJV[i]) :
                OBJV[i][OT]=="CYL" ? cylMass(OBJV[i]) : 0)
        echo(str("MASS OF ",OBJV[i][ON],",",OBJV[i][OT],",dens=",OBJV[i][OD],",m=",m))
        m + objMass(OBJV,i+1) ;

// Recursive functions to find composite center of mass CM
function objCMx(OBJV,i=0) = 
    OBJV[i]==undef ? 0 : 
        let (m= OBJV[i][OT]=="CUBE" ? cubeMass(OBJV[i]) :
                OBJV[i][OT]=="CYL" ? cylMass(OBJV[i]) : 0)
        //echo(str("X CM OF ",OBJV[i][ON],",",OBJV[i][OT],",",x=",",OBJV[i][OX]))
        m*OBJV[i][OX] + objCMx(OBJV,i+1) ;

function objCMy(OBJV,i=0) = 
    OBJV[i]==undef ? 0 : 
        let (m= OBJV[i][OT]=="CUBE" ? cubeMass(OBJV[i]) :
                OBJV[i][OT]=="CYL" ? cylMass(OBJV[i]) : 0)
        //echo(str("Y CM OF ",OBJV[i][ON],",",OBJV[i][OT],",",y=",",OBJV[i][OY]))
        m*OBJV[i][OY] + objCMy(OBJV,i+1) ;

function objCMz(OBJV,i=0) = 
    OBJV[i]==undef ? 0 : 
        let (m= OBJV[i][OT]=="CUBE" ? cubeMass(OBJV[i]) :
                OBJV[i][OT]=="CYL" ? cylMass(OBJV[i]) : 0)
        //echo(str("Z CM OF ",OBJV[i][ON],",",OBJV[i][OT],",",z=",",OBJV[i][OZ]))
        m*OBJV[i][OZ] + objCMz(OBJV,i+1) ;

/* recursive function to add Izz of objects -- DOES NOT INCLUDE LOCATION
function objIzz(OBJV,i=0) = 
    OBJV[i]==undef ? 0 : 
        let (Izz= OBJV[i][OT]=="CUBE" ? cubeInertia(OBJV[i]) :
                OBJV[i][OT]=="CYL" ? cylInertia(OBJV[i]) : 0)
        Izz + objIzz(OBJV,i+1) ;
*/

// Recursive function to add Izz of objects -- INCLUDES LOCATION
function objIzz(OBJV,pos=[0,0,0],i=0) = 
    // Paralll axis function for moments of inertia
    // The inertia of the composite object is at location pos
    OBJV[i]==undef ? 0 : 
    let (Izz= OBJV[i][OT]=="CUBE" ? cubeInertia(OBJV[i]) :
            OBJV[i][OT]=="CYL" ? cylInertia(OBJV[i]) : 0)
    let (m= OBJV[i][OT]=="CUBE" ? cubeMass(OBJV[i]) :
            OBJV[i][OT]=="CYL" ? cylMass(OBJV[i]) : 0)
    let (dx=pos[0]-OBJV[i][OX]) // sign does not matter
    let (dy=pos[1]-OBJV[i][OY]) // sign does not matter
    let (Parall_Izz=m*(pow(dx,2)+pow(dy,2)))
    echo(str("INERTIA OF ",OBJV[i][ON],",",OBJV[i][OT],",dx=",dx,",dy=",dy,",Izz=",Izz,",Parall_Izz=",Parall_Izz))
    //[Inertia[0] + m*(pow(pos[1],2)+pow(pos[2],2)),  // for Ixx
    //Inertia[1] + m*(pow(pos[0],2)+pow(pos[2],2)),   // for Iyy
    Izz + Parall_Izz + objIzz(OBJV,pos,i+1);  // ];

// There is also a Paralll axis fuction for products of inertia, but not sure when to use it

function cubeVolume(size=[1,2,3]) = size[0]*size[1]*size[2];

function cubeMass(OBJ) =
    cubeVolume(size=[OBJ[O1],OBJ[O2],OBJ[O3]])*OBJ[OD];

// Calculate the centroid of a single cube
function cubeCentroid(OBJ) = [OBJ[OX],OBJ[OY],OBJ[OZ]];// + size/2; ???

function cubeInertia(OBJ) =
    // also know as a rectangular Paralllipiped
    let (m = cubeMass(OBJ))
    //let (Ixx = (1/12)*m*(pow(OBJ[O2],2)+pow(OBJ[O3],2)))
    //let (Iyy = (1/12)*m*(pow(OBJ[O1],2)+pow(OBJ[O3],2)))
    let (Izz = (1/12)*m*(pow(OBJ[O1],2)+pow(OBJ[O2],2)))
    Izz;
    //[Ixx,Iyy,Izz];
    
function cylVolume(size=[1,2]) = PI*pow(size[0]/2,2)*size[1];  // [radius,length]

function cylMass(OBJ) =
    cylVolume(size=[OBJ[O1],OBJ[O2]])*OBJ[OD];
    
function cylInertia(OBJ) =
    // cylinder
    let (m = cylMass(OBJ))
    //let (Ixx = (1/12)*m*(3*pow(OBJ[O1]/2,2)+pow(OBJ[O2],2)))
    //let (Iyy = Ixx)
    let (Izz = (1/2)*m*(pow(OBJ[O1]/2,2)))
    Izz;
    //[Ixx,Iyy,Izz];
    

module drawObjects(OBJV) {
    for (i=[0:len(OBJV)-1]) {
        color(OBJV[i][OC],0.5) 
            translate([OBJV[i][OX],OBJV[i][OY],OBJV[i][OZ]])
                if (OBJV[i][OT] == "CUBE") 
                    cube([OBJV[i][O1],OBJV[i][O2],OBJV[i][O3]],center=true);
                else if (OBJV[i][OT] == "CYL")
                    cylinder(d=OBJV[i][O1],h=OBJV[i][O2],center=true,$fn=FACETS);
                else
                    echo("OBJECT TYPE NOT FOUND");
    };
}

// TRANSFORMATION FUNCTIONS
function rot_z(pt,ang) = 
   [pt[0]*cos(ang)-pt[1]*sin(ang),pt[0]*sin(ang)+pt[1]*cos(ang),pt[2]];

function roll(rad,pt,ang) = 
// roll point pt on a wheel of radius rad, initially at [0,0], by angle ang, 
   [pt[0]*cos(ang)-pt[1]*sin(ang),pt[0]*sin(ang)+pt[1]*cos(ang),pt[2]];

// NEED A MOTOR TORQUE VECTOR, TO SPECIFY TORQUE VS TIME
//  max torque for a 40:1 is about 43 kg-cm
//TORQ = [for (t=[0.0:DT:END_TIME]) 0.1 ];
//echo(TORQ=TORQ);


// Dynamic Analysis methods:
// State (time) Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
//                     0   1 2 3    4  5  6   7  8   9
KT=0; // time
KX=1; // x position of center of mass
KY=2; // y position of center of mass
KR=3; // wheel angle (radians)
KVX=4; // x velocity
KVY=5; // y velocity
KVR=6; // rotational velocity
KAX=7; // x acceleration
KAY=8; // y acceleration
KAR=9; // rotational acceleration
//
END_TIME = 0.20;  // seconds
DT = 0.01; // delta time in seconds

INIT_ANG = 30; // DEG
IAR = INIT_ANG*PI/180;  // initial angle radians
INIT_POS = XY_from_Gamma(INIT_ANG);
FINAL_POS = [PI*WHEEL_DIA-INIT_POS[0],INIT_POS[1],0];
translate(FINAL_POS) color("blue") cube([0.5,10,1],center=true);
echo(FINIAL_POS=FINAL_POS);

// Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
KIN_0=    [0,INIT_POS[0],INIT_POS[1],IAR, 0,0,0,AX_from_Gamma(IAR,0),AY_from_Gamma(IAR,0),-INIT_POS[2]]; // initial kinematics

function XY_from_Gamma(GAMMA) = 
    // Cog Whobble Wheel function, returns X,Y,Angle Accel give angle
    // GAMMA is in Degrees (unlike the other times!)
    // Third value is Angular acceleration
    [(GAMMA*PI/180)*RAD_WHEEL + sin(GAMMA)*CM1,cos(GAMMA)*CM1, (M1*G*sin(GAMMA)*CM1)/I1];

function AX_from_Gamma(Gamma,Acentrip) = (G - Acentrip)*sin(Gamma*180/PI);

function AY_from_Gamma(Gamma,Acentrip) = (-G + Acentrip)*cos(Gamma*180/PI);

// RECURSIVE TIME STEP FUNCTION, GENERATES ARRAY OF DYNAMIC POSITIONS
function time_stepper(DELTA_T,END_T,PRIOR,time=0) = 
    // COMPUTE NEXT TIME
    let (OldGamma = PRIOR[KR])
    let (Acentrip = 0) //-pow(PRIOR[KVR],2)/CM1) // Centripital Acceleration = v^2/R
    let (AX = AX_from_Gamma(OldGamma,Acentrip))
    let (AY = AY_from_Gamma(OldGamma,Acentrip))
    let (Fx = M1*PRIOR[KAX] + M1*Acentrip*(RAD_WHEEL + CM1*cos(OldGamma)))
    let (Fy = M1*(PRIOR[KAY]+ G - Acentrip*sin(OldGamma)))
    //let (AR = (OBJM*G*sin(OldGamma*180/PI)*CM1 + OBJM*AX*cos(OldGamma*180/PI)/2)/OBJI) // SUM MOMENTS
    let (AR=(-Fy*CM1*sin(OldGamma) + Fx*(RAD_WHEEL+CM1*cos(OldGamma)) )/I1) 
    let (VR = (PRIOR[KVR] + AR*DELTA_T)) 
    let (Gamma = OldGamma + VR*DELTA_T)  // Gamma = wheel angle (radians)
    let (NEW_POS = XY_from_Gamma(Gamma*180/PI))
    let (X = NEW_POS[0]) // COG WHEEL
    let (Y = NEW_POS[1])   // COG WHEEL
    let (VX = (X-PRIOR[KX])/DELTA_T) 
    let (VY = (Y-PRIOR[KY])/DELTA_T)
    let (NEXT_STEP = [time+DELTA_T,X,Y,Gamma,VX,VY,VR,AX,AY,AR])
    echo(str(",",time,",",PRIOR[KX],",",PRIOR[KY],",",PRIOR[KR], ",", PRIOR[KVX],",",PRIOR[KVY],",",PRIOR[KVR],",", PRIOR[KAX],",", PRIOR[KAY],",",PRIOR[KAR],",",Acentrip,",",Fx,",",Fy,","))
    (time <= END_T) ? 
        concat([PRIOR],time_stepper(DELTA_T=DELTA_T,END_T=END_T, PRIOR=NEXT_STEP,time=time+DELTA_T)) :
        [PRIOR] ;

function echo_header() =
    // This function writes header line before data lines for spreadsheet
    echo(str(",time,X,Y,rot,X velo,Y velo,rot velo,X accel,Yaccel,rot accel,Acentrip,Fx,Fy,"));
dummy = echo_header(); // for spreadsheet

SIM1=time_stepper(DELTA_T=DT,END_T=END_TIME,PRIOR=KIN_0);
//echo(SIM1=SIM1);
drawSIM(SIM=SIM1,DELTA_T=DT,END_T=END_TIME);

// Recursive module to draw the center of mass of a simulation
module drawSIM(SIM,DELTA_T,END_T,PRIOR,time=0,i=0) {
    if (time <= END_T) {
        // draw center of mass indicator
        color("green") 
            translate([SIM[i][KX],SIM[i][KY],0])
                rotate([0,0,(-SIM[i][KR])*180/PI]) {
                    cylinder(h=1,r=0.5,$fn=FACETS);
                    cube([2,0.2,1],center=true);
                };
        // recursive call to draw next time step
        drawSIM(SIM=SIM1,DELTA_T=DT,END_T=END_TIME,time=time+DELTA_T,i=i+1);
    };
};

// static picture (put last to display propertly):
translate([(INIT_ANG*PI/180)*RAD_WHEEL,0,0]) rotate([0,0,-INIT_ANG]) {
    color("red") translate(OBJCM) sphere(r=0.7,$fn=FACETS);
    drawObjects(OBJECTS);
};

/*  UNUSED METHODS    
// Concatenating two vectors, Without using indices:
function cat(L1, L2) = [for(L=[L1, L2], a=L) a];


// CHASSIS Force Diagram 
module CHASSIS_FORCES(pitchAngle=0) {
    Z_VIS = 0; // z offset for visulization
    scaled_force = (CHASSIS_MASS)*FORCE_SCALER;
      
    translate([0,0,Z_VIS]) {
        color ("darkred") translate(CHASSIS_CM) 
        force_arrow([0,0,0],[0,-scaled_force,0],mag=scaled_force);
        
        color ("darkgreen") 
        torque_arrow(to=[0,0,0],mag=MTR_TORQUE/100);
    }
}
*CHASSIS_FORCES(PITCH_ANG);
*/
