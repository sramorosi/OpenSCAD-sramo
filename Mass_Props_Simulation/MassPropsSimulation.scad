// Tools for calculating Mass Properties Design and 2D Dynamic analysis
// Also, Two Wheel Balancing Robot model
// by SrAmo  , 2025
// In physics, kinematics is the study of motion without considering the forces causing it,
// focusing on aspects like displacement, velocity, and acceleration, while dynamics examines
// the forces and torques that produce or change motion.
//
//use <../Robot_Arm_Parts_lib.scad>
use <../ME_lib.scad>  // force vectors and such

// use large value (~100) for printing, smaller (~40) for display
FACETS = $preview ? 100 : 150; // [40,100]
//
ECHO_V = true;  // this is a boolean to toggle echo of values in the log window

G = 980.665;  // cm/sec^2  gravity
echo(str("Gravity = ",G,", g/sec^2"));

PET_DENSITY = 1.39;  // grams/cm^3
POLYCARB_DENSITY = 1.20;  // grams/cm^3
ALUM_DEN = 2.72;  // grams/cm^3

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


// Function for calculating Mass Properties of objects,
// such as mass, moment of inertias, center of mass (CM).
// 

// Recursive function to add mass of objects    
function objMass(OBJV,ECHO=false,i=0) = 
    OBJV[i]==undef ? 0 : 
        let (m= OBJV[i][OT]=="CUBE" ? cubeMass(OBJV[i]) :
                OBJV[i][OT]=="CYL" ? cylMass(OBJV[i]) : 0)
        let (z=ECHO?echo(str("MASS OF ",OBJV[i][ON],",",OBJV[i][OT],",dens=",OBJV[i][OD],",m=",m)):0)
        m + objMass(OBJV,ECHO,i+1) ;

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

function objCM(OBJ) =
    // Find weighted average center of mass Objects in each direction
    let (OBJM = objMass(OBJ,false))
    let (OBJCMx = objCMx(OBJ)/OBJM)
    let (OBJCMy = objCMy(OBJ)/OBJM)
    let (OBJCMz = objCMz(OBJ)/OBJM)
    let (OBJCM = [OBJCMx,OBJCMy,OBJCMz])
    echo(str("CENTER OF MASS, NO TRANSFORMATION  = ",OBJCM))
    OBJCM;

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

function Mass_Totals(OBJ) = 
    let (OBJM = objMass(OBJ,true))
    echo(str("TOTAL MASS = ",OBJM))
    let (OBJI = objIzz(OBJ)) //  defaults to around [0,0,0] 
    echo(str("TOTAL MOMENT OF INERTIA Izz = ",OBJI))
    let (CM = objCM(OBJ)) // get center of mass
    [OBJM,OBJI,CM];

module drawObjects(OBJV) {
    for (i=[0:len(OBJV)-1]) {
        color(OBJV[i][OC],0.3) 
            translate([OBJV[i][OX],OBJV[i][OY],OBJV[i][OZ]])
                if (OBJV[i][OT] == "CUBE") 
                    cube([OBJV[i][O1],OBJV[i][O2],OBJV[i][O3]],center=true);
                else if (OBJV[i][OT] == "CYL")
                    cylinder(d=OBJV[i][O1],h=OBJV[i][O2],center=true,$fn=FACETS);
                else
                    echo("OBJECT TYPE NOT FOUND");
    };
}
module draw_Object(OBJ,ANG) {
     rotate([0,0,ANG]) {
        drawObjects(OBJ);
    };
};

// Dynamic Analysis methods:
// State (time) Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
//                     0   1 2 3    4  5  6   7  8   9
KT=0; // time
KX=1; // x position of center of mass body1
KY=2; // y position of center of mass body1
KR=3; // body1 (wheel) angle (radians)
KVX=4; // x velocity body1
KVY=5; // y velocity body1
KVR=6; // rotational velocity body1
KAX=7; // x acceleration body1
KAY=8; // y acceleration body1
KAR=9; // rotational acceleration body1

KMT=10; // Motor Torque Applied (g-force cm)
KSP=11; // Angle set point for controller

KX2=12; // x position of center of mass body2
KY2=13; // y position of center of mass body1
KR2=14; // body2 angle (radians)
KVX2=15; // x velocity body2
KVY2=16; // y velocity body2
KVR2=17; // rotational velocity body2
KAX2=18; // x acceleration body2
KAY2=19; // y acceleration body2
KAR2=20; // rotational acceleration body2


function XY_from_Gamma(GAMMA,M,L,I,RAD_WHEEL) = 
    // Cog time_step_Whobble_Wheel function, returns X,Y,Angle Accel give angle
    // GAMMA is in Degrees (unlike the other times!)
    // Third value is Angular acceleration
    let (xg = 90-GAMMA)
    [(xg*PI/180)*RAD_WHEEL + cos(GAMMA)*L, sin(GAMMA)*L, (M*G*sin(GAMMA)*L)/I];

// RECURSIVE TIME STEP FUNCTION, GENERATES ARRAY OF DYNAMIC POSITIONS
// PRIOR = initial condition state vector
// Returns a vector of state vectors, which is the simulation

// 
function time_step_Whobble_Wheel(DELTA_T,END_T,M,L,I,WHEELR,PRIOR,time=0) = 
    // Motion simulator for Wobble Wheel
    // Gamma = wheel angle (radians), -PI/2 is down, 0 is out-right, PI/2 is up
    let (OldGamma = PRIOR[KR])
    let (oldXaccel = PRIOR[KAX])
    let (BA = bodyAccel(R=WHEELR,M=M,I=I,Xaccel=oldXaccel))
    //let (Acentrip = 0) //-pow(PRIOR[KVR],2)/CM1) // Centripital Acceleration = v^2/R
    let (AR=GammaAccel(M=M,L=L,I=I,Gamma=OldGamma*180/PI) + BA) // from moment balance
    let (VR = PRIOR[KVR] + AR*DELTA_T) // change velocity based on acceleration
    let (Gamma = OldGamma + VR*DELTA_T)  // change rotation based on velocity
    let (NewPos = XY_from_Gamma(GAMMA=Gamma*180/PI,M=M,L=L,I=I,RAD_WHEEL=WHEELR))
    let (X = NewPos[0]) 
    let (Y = NewPos[1]) 
    let (VX = (X-PRIOR[KX])/DELTA_T)
    let (VY = (Y-PRIOR[KY])/DELTA_T)
    let (AX = (VX-PRIOR[KVX])/DELTA_T)
    let (AY = (VY-PRIOR[KVY])/DELTA_T)
    let (NEXT_STEP = [time+DELTA_T,X,Y,Gamma,VX,VY,VR,AX,AY,AR])
    let (z = (ECHO_V) ? echo(str(",",time,",",PRIOR[KX],",",PRIOR[KY],",",PRIOR[KR], ",", PRIOR[KVX],",",PRIOR[KVY],",",PRIOR[KVR],",", PRIOR[KAX],",", PRIOR[KAY],",",PRIOR[KAR],",",BA)): 0)
    (time <= END_T) ? 
        concat([PRIOR],time_step_Whobble_Wheel(DELTA_T=DELTA_T,END_T=END_T, M=M, L=L, I=I, WHEELR=WHEELR, PRIOR=NEXT_STEP,time=time+DELTA_T)) :
        [PRIOR] ;

function PID(ref,setpt,ang_v,ang_acc,Kp,Kg,Kv,Ka) = 
    // simple PID
    let (error = ref-setpt)
    Kg*cos(ref) + Kv*ang_v + Ka*ang_acc + Kp*error;
    
function time_step_Pendulum(DELTA_T,END_T,M,L,I,PRIOR,SetPtVSTime,Kp,Kg,Kv,time=0) = 
    // Motion simulator for Pendulum
    // SetPtVSTime = set point verses time (vector of set point)
    // Gamma = pendulum angle (radians), -PI/2 is down, 0 is out-right, PI/2 is up
    let (OldGamma = PRIOR[KR])
    let (SetPoint = (SetPtVSTime == undef) ? 0 : lookup(time,SetPtVSTime)) // in degrees
    //let (Acentrip = -pow(PRIOR[KVR],2)/CM1) // Centripital Acceleration = v^2/R, not used
    //let (Fy = 0) // not used
    //let (Fx = 0) // not used
    //let (AR=(-G*cos(OldGamma*180/PI)/CM1))  // simple pendulum formula, don't use
    let (OldAngV = PRIOR[KVR]) // is radians per second
    let (OldAngAcc = PRIOR[KAR]) // radians per second^2
    let (T=PID(ref=OldGamma*180/PI,setpt=SetPoint,ang_v=OldAngV,ang_acc=OldAngAcc,Kp=Kp,Kg=Kg,Kv=Kv,Ka=0))
    let (AR=GammaAccel(M=M,L=L,I=I,Gamma=OldGamma*180/PI) + T*G/I) // from moment balance
    let (VR = PRIOR[KVR] + AR*DELTA_T) // change velocity based on acceleration
    let (Gamma = OldGamma + VR*DELTA_T)  // change rotation based on velocity
    let (X = L*cos(Gamma*180/PI)) 
    let (Y = L*sin(Gamma*180/PI)) 
    let (VX = (X-PRIOR[KX])/DELTA_T)
    let (VY = (Y-PRIOR[KY])/DELTA_T)
    let (AX = (VX-PRIOR[KVX])/DELTA_T)
    let (AY = (VY-PRIOR[KVY])/DELTA_T)
    let (NEXT_STEP = [time+DELTA_T,X,Y,Gamma,VX,VY,VR,AX,AY,AR,T])
    let (z = (ECHO_V) ? echo(str(",",time,",",PRIOR[KX],",",PRIOR[KY],",",PRIOR[KR], ",", PRIOR[KVX],",",PRIOR[KVY],",",PRIOR[KVR],",", PRIOR[KAX],",", PRIOR[KAY],",",PRIOR[KAR],",",T,",",SetPoint,",")) : 0)
    (time <= END_T) ? 
        concat([PRIOR],time_step_Pendulum(DELTA_T=DELTA_T,END_T=END_T, M=M, L=L, I=I, PRIOR=NEXT_STEP,SetPtVSTime=SetPtVSTime, Kp=Kp,Kg=Kg,Kv=Kv, time=time+DELTA_T)) :
        [PRIOR] ;

// rotational acceleration from moment balance
// where L is the distance from the point of rotation to the center of mass and
// Gamma is the rotation angle, zero is horizontally to the right
function GammaAccel(M,L,I,Gamma) = -G*M*L*cos(Gamma)/I; // Gamma is in degrees

// Body (wheel) acceleration from moment balance
// where R is the radius of the wheel and Xaccel is the body X acceleration
function bodyAccel(R,M,I,Xaccel) = (R*M*Xaccel)/(I*2*PI);

function echo_header() =
    // This function writes header line before data lines for spreadsheet
    let (z = (ECHO_V) ? echo(str(",time,X,Y,rot-rad,X velo,Y velo,rot velo,X accel,Yaccel,rot accel,AppliedTorq,SetPoint Deg,not used,")) : 0)
    0;

// Find the frequency by looking for the peak rotation
module max_gamma(v,i=0,index=-9999,val=-9999) {
    if (v[i][KR] != undef) {
        if ((v[i][KR] > val) && (i>20)) {
            if (i != 0) echo(str("max gamma -1, ",v[i-1][KR]*180/PI,", index,",i-1,", time,",v[i-1][KT]));
            echo(str("max gamma, ",v[i][KR]*180/PI,", index,",i,", time,",v[i][KT]));
            if (i != 0) echo(str("max gamma +1, ",v[i+1][KR]*180/PI,", index,",i+1,", time,",v[i+1][KT]));
            max_gamma(v=v,i=i+1,index=i,val=v[i][KR]);
        }
        else {
            max_gamma(v=v,i=i+1,index=index,val=val);
       }
    }
};


// draw a simple object representation for a single time
module drawSIM(SIM,i=0,vectors=false) {
    // draw center of mass indicator
    color("green") 
        translate([SIM[i][KX],SIM[i][KY],0])
            rotate([0,0,(SIM[i][KR])*180/PI]) {
                cylinder(h=.1,r=0.1,center=true,$fn=FACETS);
                cube([1,0.05,.1],center=true);
            };
    if (vectors) {
        // draw velocity vector
        VMag = 0.05*sqrt(pow(SIM[i][KVX],2) + pow(SIM[i][KVY],2));
        color("blue") force_arrow([SIM[i][KX],SIM[i][KY],0],[SIM[i][KVX],SIM[i][KVY],0],mag=VMag);
                
        // draw acceleration vector
        AMag = 0.001*sqrt(pow(SIM[i][KAX],2) + pow(SIM[i][KAY],2));
        color("red") force_arrow([SIM[i][KX],SIM[i][KY],0],[SIM[i][KAX],SIM[i][KAY],0],mag=AMag);
    };

}

// Recursive module to draw simple representations for a simulation
module drawSIM_Vector(SIM,i=0) {
    if (SIM[i] != undef) {
        
        drawSIM(SIM,i);
        
        // recursive call to draw next time step
        drawSIM_Vector(SIM=SIM,i=i+1);
    };
};

// module to draw xy chart of angle vs time for a simulation
module drawAngVSTimeChart(SIM,Tscale=10) {
    l = len(SIM);
    Last_Time = SIM[l-1][KT];
    TimeAngVec = [ for (j = [0:1:l-1])  [SIM[j][KT]*Tscale,SIM[j][KR]*180/PI] ];
    TimeAngVec_1 = concat(TimeAngVec,[[Last_Time*Tscale,SIM[0][KR]*180/PI-1]]);
    color("blue",.5) polygon(TimeAngVec_1);
};

module drawXVSTimeChart(VEC,Tscale=10,Yscale=0.01) {
    l = len(VEC);
    Last_Time = VEC[l-1][0];
    TimeXVec = [ for (j = [0:1:l-1])  [VEC[j][0]*Tscale,VEC[j][1]*Yscale] ];
    TimeXVec_1 = concat(TimeXVec,[[Last_Time*Tscale,VEC[0][1]*Yscale-1]]);
    color("yellow",.5) polygon(TimeXVec_1);
};


//drawAngVSTimeChart(SIM1);

//  ## DATA BEGINES HERE ##

// Pendulum
Pend = true; // Simple Pendulum
if (Pend) {
    BRASS_OBJ = ["BRASS"       ,"CUBE","gold" ,9.73      ,2.5     ,5.0 ,4.2   ,0,20.0  ,0];
    ARM_OBJ   = ["Arm"         ,"CUBE","silver",ALUM_DEN, 2.80    ,28.0,0.34   ,0,12.0  ,0]; 
    OBJECTS = [BRASS_OBJ,ARM_OBJ];

    // Get total mass properties for object
    om = Mass_Totals(OBJECTS);
    M1 = om[0];
    I1 = om[1];
    CM1 = abs(om[2][1]);

    END_TIME = 4.0;  // seconds, full cycle for simple = 0.29, compound = 0.74
    DT = 0.02; // delta time in seconds
    echo(str("End Time = ",END_TIME,", Time Step = ",DT,", Number of time steps = ",END_TIME/DT));

    INIT_ANG = -90; // DEG
    IAR = INIT_ANG*PI/180;  // initial angle radians

    // Simple pendulum frequency = 2*PI*sqrt(L/G),  for swing < 30 deg
    SimpleFreq = 2*PI*sqrt(CM1/G);
    echo(str("Simple Frequency = ",SimpleFreq," seconds"));
    // Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr)) ,  for swing < 30 deg
    CompoundFreq = 2*PI*sqrt(I1/(M1*G*CM1));
    echo(str("Compound Frequency = ",CompoundFreq," seconds"));

    // MOTOR TORQUE VECTOR, TO SPECIFY TORQUE VS TIME lookup table -- a vector of key-value pairs
    //  max torque for a REV HD Hex SPUR 40:1 is  43,000 g-cm
    //  max torque for a REV Hex Motor 18.9:1 with gearing (5:1 & 4:1 gearbox) is 19,300 g-cm
    Set_Point_Vect = [[0,INIT_ANG],[.05,INIT_ANG],[.07,0],[2.0,0],[2.02,-60],[END_TIME,-60]];

    dummy = echo_header(); // for spreadsheet
    
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,CM1*cos(INIT_ANG),CM1*sin(INIT_ANG),IAR, 0,0,0,0,0,GammaAccel(M=M1,L=CM1,I=I1,Gamma=INIT_ANG),0,-INIT_ANG];
    
    SIM1=time_step_Pendulum(DELTA_T=DT,END_T=END_TIME,M=M1,L=CM1,I=I1,PRIOR=KIN_0, SetPtVSTime=Set_Point_Vect, Kp=-100, Kg=M1*CM1, Kv=-1800);

    drawXVSTimeChart(VEC=Set_Point_Vect,Tscale=100,Yscale=1);
    drawAngVSTimeChart(SIM=SIM1,Tscale=100);
    
    color("red") rotate([0,0,INIT_ANG-90]) translate([0,CM1,0]) sphere(r=0.4,$fn=FACETS);
    draw_Object(OBJ=OBJECTS,ANG=INIT_ANG-90); // put last to display propertly

};

// Whobble Wheel
WW = false;  // Whobble Wheel
if (WW) {
/*
 * Wobble Wheel Test Data
 * Each Grey wheel, mass = 270 g, Outside dia = 15.5 cm, Inside dia = 2.8 cm, thickness = 2.54 cm
 *       backsolve density to match mass = 0.583, Izz = 8382
 * brass weight = 50 by 25 by 42 mm,  m = 522 g, center of mass offset = 40 mm
 * TOTAL MASS = 1062 grams
 * Test results, with roll from dead stop:
 * Initial angle = 30 deg, time to first reverse = 1.37 sec, distance = ~400 cm
 *                         time to second reverse = 1.25 sec, less distance
 */
    WHEEL_DIA = 15.5; // cm
    WHEEL_DENS = 0.768; // 0.663; // 0.583;  // g/cm^3
    RAD_WHEEL = WHEEL_DIA/2;
    STRUT_L = 9; // LENGTH OF STRUT, cm
    // Oject v=["NAME",  "TYPE",       "COLOR", densi   ,X_Size,Y_Size  ,Z_Size,X_CM,Y_CM,Z_CM];
    STRUT_OBJ =["BRASS","CUBE",        "gold" ,9.94     ,5.0     ,2.5   ,4.2     ,0,4.0  ,0];
    WHEEL1_OBJ =["WHEEL1",      "CYL" ,"grey",WHEEL_DENS ,WHEEL_DIA,2.54,0    ,0,0          ,-1.5];
    WHEEL1_HOLE_OBJ=["WHEEL1-H","CYL","white",-WHEEL_DENS,8.0      ,2.54,0    ,0,0          ,-1.5];
    WHEEL2_OBJ =["WHEEL2",      "CYL" ,"grey",WHEEL_DENS ,WHEEL_DIA,2.54,0    ,0,0          ,1.5];
    WHEEL2_HOLE_OBJ=["WHEEL2-H","CYL","white",-WHEEL_DENS,8.0      ,2.54,0    ,0,0          ,1.5];
    OBJECTS = [STRUT_OBJ,WHEEL1_OBJ,WHEEL1_HOLE_OBJ,WHEEL2_OBJ,WHEEL2_HOLE_OBJ] ;

    // Get total mass properties for object
    om = Mass_Totals(OBJECTS);
    M1 = om[0];
    I1 = om[1];
    CM1 = abs(om[2][1]);

    END_TIME = 1.5;  // seconds, full cycle for simple = 0.29, compound = 0.74
    DT = 0.01; // delta time in seconds
    echo(str("End Time = ",END_TIME,", Time Step = ",DT,", Number of time steps = ",END_TIME/DT));

    INIT_ANG = 30; // DEG
    IAR = INIT_ANG*PI/180;  // initial angle radians
    INIT_POS = XY_from_Gamma(INIT_ANG,M=M1,L=CM1,I=I1,RAD_WHEEL=RAD_WHEEL); // only used by whobble wheel
    FINAL_POS = [PI*WHEEL_DIA-INIT_POS[0],INIT_POS[1],0]; // only used by whobble wheel

    // Simple pendulum frequency = 2*PI*sqrt(L/G),  for swing < 30 deg
    SimpleFreq = 2*PI*sqrt(CM1/G);
    echo(str("Simple Frequency = ",SimpleFreq," seconds"));
    // Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr)) ,  for swing < 30 deg
    CompoundFreq = 2*PI*sqrt(I1/(M1*G*CM1));
    echo(str("Compound Frequency = ",CompoundFreq," seconds"));

    dummy = echo_header(); // for spreadsheet
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,INIT_POS[0],INIT_POS[1],IAR, 0,0,0,0,0,GammaAccel(M=M1,L=CM1,I=I1,Gamma=INIT_ANG)];
    
    SIM1=time_step_Whobble_Wheel(DELTA_T=DT,END_T=END_TIME,M=M1,L=CM1,I=I1, WHEELR=RAD_WHEEL, PRIOR=KIN_0);
    
    max_gamma(v=SIM1,val=IAR-.1); // echo the period time
    drawSIM_Vector(SIM=SIM1);
    
    translate([((90-INIT_ANG)*PI/180)*RAD_WHEEL,0,0]) {
        color("red") rotate([0,0,INIT_ANG-90]) translate([0,CM1,0]) sphere(r=0.4,$fn=FACETS);
        draw_Object(OBJ=OBJECTS,ANG=INIT_ANG-90); // put last to display propertly
    };
    
    translate(FINAL_POS) color("blue") cube([0.5,10,1],center=true); // indicator for how far ww moves
};

// Two Wheel Robot, with low center of mass
TWR = false;
if (TWR) {
    WHEEL_DIA = 15.5; // cm
    WHEEL_DENS = 0.768 * 1.09 ; // Includes wheel hub g/cm^3
    RAD_WHEEL = WHEEL_DIA/2;
    // Oject v=["NAME",  "TYPE",       "COLOR", densi   ,X_Size,Y_Size  ,Z_Size,X_CM,Y_CM,Z_CM];
    
    // Each Grey wheel, mass = 270 g, Outside dia = 15.5 cm, Inside dia = 2.8 cm, thickness = 2.54 cm
    //     backsolve density to match mass = 0.583, Izz = 8382
    WHEEL1_OBJ =["WHEEL1",      "CYL" ,"blue",WHEEL_DENS ,WHEEL_DIA,2.54,0    ,0,0          ,-15];
    WHEEL1_HOLE_OBJ=["WHEEL1-H","CYL","white",-WHEEL_DENS,8.0      ,2.54,0    ,0,0          ,-15];
    WHEEL2_OBJ =["WHEEL2",      "CYL" ,"blue",WHEEL_DENS ,WHEEL_DIA,2.54,0    ,0,0          ,15];
    WHEEL2_HOLE_OBJ=["WHEEL2-H","CYL","white",-WHEEL_DENS,8.0      ,2.54,0    ,0,0          ,15];
    
    //  max torque for a REV HD Hex SPUR 40:1 is  43,000 g-force - cm
    MTR_DENS = 3.48;
    MTR_D = 3.5;
    MTR_L = 10.0;
    MOTOR1 = ["MOTOR1", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,-1,10];
    MOTOR2 = ["MOTOR2", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,-1,-10];
    
    HUB_OBJ =["HUB", "CUBE", "black",     0.473     ,10.3,3.0,14.3, 0,-4,0]; // Hub mass = 209 grams
    PHONE = ["PHONE","CUBE", "blue",      1.47,        1.0, 7.5, 15.0, 0, 10.0, 0]; // Phone mass = 165 grams
    BATTERY = ["BATTERY","CUBE","green", 0.94, 13.0,7.5, 7.5, 0,1.5,0]; // Battery mass = 685 grams
    // total mass = 4588 grams
    PLATE = ["PLATE","CUBE","gold", 9.69,  9,1,26, 0,-6,0]; // adjust density to get total mass
    
    WHEELS = [WHEEL1_OBJ,WHEEL1_HOLE_OBJ,WHEEL2_OBJ,WHEEL2_HOLE_OBJ] ;
    BODY = [HUB_OBJ,PHONE,MOTOR1,MOTOR2, BATTERY, PLATE] ;

    // Get total mass properties for object
    wm = Mass_Totals(WHEELS);
    Mwheels = wm[0];
    Iwheels = wm[1];
    CMwheels = abs(wm[2][1]);
    bm = Mass_Totals(BODY);
    Mbody = bm[0];
    Ibody = bm[1];
    CMbody = abs(bm[2][1]);
    
    END_TIME = 2.0;  // seconds, full cycle for simple = 0.29, compound = 0.74
    DT = 0.02; // delta time in seconds
    echo(str("End Time = ",END_TIME,", Time Step = ",DT,", Number of time steps = ",END_TIME/DT));

    INIT_ANG = -90; // DEG
    IAR = INIT_ANG*PI/180;  // initial angle radians

    // Simple pendulum frequency = 2*PI*sqrt(L/G),  for swing < 30 deg
    SimpleFreq = 2*PI*sqrt(CMbody/G);
    echo(str("Body Simple Frequency = ",SimpleFreq," seconds"));
    // Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr)) ,  for swing < 30 deg
    CompoundFreq = 2*PI*sqrt(Ibody/(Mbody*G*CMbody));
    echo(str("Body Compound Frequency = ",CompoundFreq," seconds"));

    // Set Point vector to set desired body angle vs time
    Set_Point_Vect = [[0,INIT_ANG],[0.5,INIT_ANG],[0.52,0],[END_TIME,0]];

    dummy = echo_header(); // for spreadsheet
    
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,CMbody*cos(INIT_ANG),CMbody*sin(INIT_ANG),IAR, 0,0,0,0,0,GammaAccel(M=Mbody,L=CMbody,I=Ibody,Gamma=INIT_ANG),0,-INIT_ANG];
    
    SIM1=time_step_Pendulum(DELTA_T=DT,END_T=END_TIME,M=Mbody,L=CMbody,I=Ibody,PRIOR=KIN_0,SetPtVSTime=Set_Point_Vect, Kp=-200, Kg=Mbody*CMbody, Kv=-1950);

    drawXVSTimeChart(VEC=Set_Point_Vect,Tscale=100,Yscale=1);
    drawAngVSTimeChart(SIM=SIM1,Tscale=100);


    //color("red") translate(CMwheels) sphere(r=1,$fn=FACETS);
    color("orange") translate([0,-CMbody,0]) sphere(r=1,$fn=FACETS);
    //draw_Object(OBJ=WHEELS,ANG=0); // put last to display propertly
    draw_Object(OBJ=BODY,ANG=0); // put last to display propertly

};