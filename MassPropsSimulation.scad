// Tools for calculating Mass Properties Design and 2D Dynamic analysis
// Also, Two Wheel Balancing Robot model
// by SrAmo  , 2025

//use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>
//use <ME_lib.scad>
//use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

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

PITCH_ANG = 30; // *sin($t*90);

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

STRUT_L = 9; // LENGTH OF STRUT, cm
// Oject v=["NAME", "TYPE", "COLOR", densi,X_Size,Y_Size,Z_Size,X_CM,Y_CM,Z_CM];
STRUT_OBJ =["STRUT","CUBE","darkblue",0.6 ,3    ,STRUT_L,3     ,0,STRUT_L/2  ,0];
PLATE_OBJ =["PLATE","CUBE","white"   ,1.2 ,12   ,1.6    ,15    ,0,STRUT_L+0.8,0];
WHEEL_OBJ =["WHEEL","CYL" ,"blue"    ,0.7 ,15.3 ,2.54   ,0     ,0,0          ,0];

// OBJV = OBJECT VECTOR, or OBJECTS
OBJECTS = [STRUT_OBJ,PLATE_OBJ,WHEEL_OBJ];

OBJM = objMass(OBJECTS);
echo(str("TOTAL MASS = ",OBJM));

// Find weighted average center of mass Objects in each direction
OBJCMx = objCMx(OBJECTS);
OBJCMy = objCMy(OBJECTS);
OBJCMz = objCMz(OBJECTS);
OBJCM = [OBJCMx/OBJM,OBJCMy/OBJM,OBJCMz/OBJM];
echo(str("CENTER OF MASS, NO TRANSFORMATION  = ",OBJCM));

OBJI = objIzz(OBJECTS); //  defaults to around [0,0,0] 
echo(str("TOTAL MOMENT OF INERTIA Izz = ",OBJI));


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

// Dynamic Analysis methods:
// Kinematics Vector=[x,y,r,  vx,vy,vr,  ax,ay,ar];  position velocity acceleration
//
// NEED A MOTOR TORQUE VECTOR, TO SPECIFY TORQUE VS TIME

END_TIME = 1.0;  // seconds
DT = 0.1; // delta time in seconds

KIN_0=[OBJCM[0],OBJCM[1],0, 0,0,0, 0,0,0];

for (t=[0.0:DT:END_TIME]) {
    echo(str("t=",t));
    CM2 = rot_z(OBJCM,KIN_0[2]);
    color("green") translate(OBJCM) sphere(r=0.5,$fn=FACETS);
    // SOLVE FOR AR USING SUM MOMENTS IN Z
    // VR = AR*DT
    // POS = V*DT, INCLUDE ROLLING WHEEL
};

// static picture (put last to display propertly):
rotate([0,0,PITCH_ANG]) {
    color("red") translate(OBJCM) sphere(r=1,$fn=FACETS);
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
