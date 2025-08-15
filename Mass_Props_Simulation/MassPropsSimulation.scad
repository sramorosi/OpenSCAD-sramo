// Tools for calculating Mass Properties Design and 2D Dynamic analysis
// Also, Two Wheel Balancing Robot model
// by SrAmo  , 2025
// In physics, kinematics is the study of motion without considering the forces causing it,
// focusing on aspects like displacement, velocity, and acceleration, while dynamics examines
// the forces and torques that produce or change motion.
//
//use <../Robot_Arm_Parts_lib.scad>
use <../ME_lib.scad>  // force vectors and such

// cm in an inch. Don't change.
CM = 2.54;

// use large value (~100) for printing, smaller (~40) for display
FACETS = $preview ? 100 : 150; // [40,100]

ECHO_V = false;  // this is a boolean to toggle echo of values in the log window

G = 980.665;  // cm/sec^2  gravity
echo(str("Gravity = ",G,", cm/sec^2"));

PET_DENSITY = 1.39;  // grams/cm^3
POLYCARB_DENSITY = 1.20;  // grams/cm^3
ALUM_DEN = 2.72;  // grams/cm^3
BRASS_DENS = 8.49; // gram/cm^3

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
    //echo(str("CENTER OF MASS, NO TRANSFORMATION  = ",OBJCM))
    OBJCM;

// Recursive function to add Izz of objects -- INCLUDES LOCATION
function objIzz(OBJV,ECHO=false,pos=[0,0,0],i=0) = 
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
    let (junk = ECHO ? echo(str("INERTIA OF ",OBJV[i][ON],",",OBJV[i][OT],",dx=",dx,",dy=",dy,",Izz=",Izz,",Parall_Izz=",Parall_Izz)) : 0 )
    //[Inertia[0] + m*(pow(pos[1],2)+pow(pos[2],2)),  // for Ixx
    //Inertia[1] + m*(pow(pos[0],2)+pow(pos[2],2)),   // for Iyy
    Izz + Parall_Izz + objIzz(OBJV,ECHO,pos,i+1);  // ];

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

function Mass_Totals(OBJ,TEXT="NONE",ECHO=true,ECHO_I=false) = 
    let (OBJM = objMass(OBJ,ECHO))
    let (qq = ECHO ? echo(str(TEXT,": MASS = ",OBJM)) : 0)
    let (OBJI = objIzz(OBJ,ECHO_I)) //  defaults to around [0,0,0] 
    let (rr = ECHO ? echo(str(TEXT,": MOMENT OF INERTIA Izz = ",OBJI)) : 0)
    let (CM = objCM(OBJ)) // get center of mass
    let (ss = ECHO ? echo(str(TEXT,": CENTER OF MASS  = ",CM)) : 0)
    [OBJM,OBJI,CM];

module drawObjects(OBJ) {
    for (i=[0:len(OBJ)-1]) {
        color(OBJ[i][OC],0.3) 
            translate([OBJ[i][OX],OBJ[i][OY],OBJ[i][OZ]])
                if (OBJ[i][OT] == "CUBE") 
                    cube([OBJ[i][O1],OBJ[i][O2],OBJ[i][O3]],center=true);
                else if (OBJ[i][OT] == "CYL")
                    cylinder(d=OBJ[i][O1],h=OBJ[i][O2],center=true,$fn=FACETS);
                else
                    echo("OBJECT TYPE NOT FOUND");
    };
}

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

function XYfromWheelAng(Ang,L,RAD_WHEEL,OFFSET=true) = 
    // Cog time_step_Wobble_Wheel function, returns X,Y give angle
    // Ang is in Degrees (unlike the other times!)
    // Ang = 0 is horizontal to the right (+x) and wheel is at x = 0
    //  for Ang = 90, x wheel has moved to PI/2*RAD_WHEEL to the right 
    // L is the Wobble (mass imbalance) distange, can be zero
    let (xg = OFFSET ? -(90+Ang) : Ang)
    [(xg*PI/180)*RAD_WHEEL + cos(Ang)*L, sin(Ang)*L];

function PID(ref,setpt,ang_v,ang_acc,Kp,Kg,Kv,Ka) = 
    // simple PID
    let (error = ref-setpt)
    Kg*cos(ref) + Kv*ang_v + Ka*ang_acc + Kp*error; 

// RECURSIVE TIME STEP FUNCTIONS. THESE GENERATE ARRAY OF POSITIONS
// PRIOR = initial condition state vector
// Returns a vector of state vectors, which is the simulation
function time_step_Wobble_Wheel(DELTA_T,END_T,M,L,I,WHEELR,PRIOR,time=0) = 
    // Motion simulator for Wobble Wheel
    // Gamma = wheel angle (radians), -PI/2 is down, 0 is out-right, PI/2 is up
    let (OldGamma = PRIOR[KR])
    let (oldXaccel = PRIOR[KAX])
    let (BA = bodyAccel(R=WHEELR,M=M,I=I,Xaccel=oldXaccel))
    //let (Acentrip = 0) //-pow(PRIOR[KVR],2)/CM1) // Centripital Acceleration = v^2/R
    let (AR=GammaAccel(M=M,L=L,I=I,Gamma=OldGamma*180/PI) + BA) // from moment balance
    let (VR = PRIOR[KVR] + AR*DELTA_T) // change velocity based on acceleration
    let (Gamma = OldGamma + VR*DELTA_T)  // change rotation based on velocity
    let (NewPos = XYfromWheelAng(Ang=Gamma*180/PI,L=L,RAD_WHEEL=WHEELR))
    let (X = NewPos[0]) 
    let (Y = NewPos[1]) 
    let (VX = (X-PRIOR[KX])/DELTA_T)
    let (VY = (Y-PRIOR[KY])/DELTA_T)
    let (AX = (VX-PRIOR[KVX])/DELTA_T)
    let (AY = (VY-PRIOR[KVY])/DELTA_T)
    let (NEXT_STEP = [time+DELTA_T,X,Y,Gamma,VX,VY,VR,AX,AY,AR])
    let (z = (ECHO_V) ? echo(str(",",time,",",PRIOR[KX],",",PRIOR[KY],",",PRIOR[KR], ",", PRIOR[KVX],",",PRIOR[KVY],",",PRIOR[KVR],",", PRIOR[KAX],",", PRIOR[KAY],",",PRIOR[KAR],",",BA)): 0)
    (time <= END_T) ? 
        concat([PRIOR],time_step_Wobble_Wheel(DELTA_T=DELTA_T,END_T=END_T, M=M, L=L, I=I, WHEELR=WHEELR, PRIOR=NEXT_STEP,time=time+DELTA_T)) :
        [PRIOR] ;

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
        
// BEGIN CART CART CART CART CART CART CART CART
function time_step_Cart(DELTA_T,END_T,Mp,Lp,Ip,Mc,Lc,Ic,WHEELR,VECT,SetPtVSTime,Kp,Kg,Kv,time=0) = 
    // Motion simulator for Cart with pendulum
    // SetPtVSTime = set-point verses time (vector of set point)
    // Gamma = pendulum angle to wheel (motor angle) (radians), -PI/2 is down, 0 is out-right, PI/2 is up
    // Theta = wheel angle (radians), -PI/2 is down, 0 is out-right, PI/2 is up
    let (OldGamma = VECT[KR])
    let (OldGammaVelo = VECT[KVR]) // radians per second
    let (OldGammaAcc = VECT[KAR]) // radians per second^2
    let (OldTheta = VECT[KR2]) 
    let (OldThetaVelo = VECT[KVR2])
    let (MotorVelo =  OldThetaVelo + OldGammaVelo) // This is equivilant to Encoder Velocity
    let (oldXaccel = VECT[KAX2])  // PEND AND CART X ACCELERATIONS ARE SIMILAR
    let (SetPoint = (SetPtVSTime == undef) ? 0 : lookup(time,SetPtVSTime)) // in degrees
    let (Torq=PID(ref=MotorVelo,setpt=SetPoint,ang_v=MotorVelo,ang_acc=oldXaccel,Kp=Kp,Kg=Kg,Kv=Kv,Ka=-1) )
    //let (Torq = 0)
    //let (NewGammaAcc=GammaAccel(M=Mp,L=Lp,I=Ip,Gamma=OldGamma*180/PI) + Torq*G/Ip) // from moment balance
    let (NewGammaAcc=-G*Mp*Lp*sin(OldGamma*180/PI)/Ip + Torq*G/Ip) // from moment balance
    let (NewGammaVelo = VECT[KVR] + NewGammaAcc*DELTA_T) // change velocity based on acceleration
    let (Gamma = OldGamma + NewGammaVelo*DELTA_T)  // change rotation based on velocity
    //function GammaAccel(M,L,I,Gamma) = -G*M*L*cos(Gamma)/I; // Gamma is in degrees
    
    let (BA = bodyAccel(R=WHEELR,M=Mc+Mp,I=Ic+Ip,Xaccel=oldXaccel))
    let (NewThetaAcc= BA - Torq*G/Ic) // from moment balance
    let (NewThetaVelo = VECT[KVR2] + NewThetaAcc*DELTA_T) // change velocity based on acceleration
    let (Theta = OldTheta + NewThetaVelo*DELTA_T)  // change rotation based on velocity
    //let (NewPos = XYfromWheelAng(Ang=Theta*180/PI,L=Lc,RAD_WHEEL=WHEELR,OFFSET=false))
    let (Xc = Theta*WHEELR) // wheel axel moves x based on angle NewPos[0]) 
    let (Yc = 0 ) // wheel axel is always at y = 0 (not a wobble wheel!)  NewPos[1]) 
    let (VXc = (Xc-VECT[KX2])/DELTA_T)
    let (VYc = (Yc-VECT[KY2])/DELTA_T)
    let (AXc = (VXc-VECT[KVX2])/DELTA_T)
    let (AYc = (VYc-VECT[KVY2])/DELTA_T)

    let (Xp = Lp*sin(Gamma*180/PI) + Xc) // add in the wheel movement to pendulu rotation point
    let (Yp = Lp*cos(Gamma*180/PI)) 
    let (VXp = (Xp-VECT[KX])/DELTA_T)
    let (VYp = (Yp-VECT[KY])/DELTA_T)
    let (AXp = (VXp-VECT[KVX])/DELTA_T)
    let (AYp = (VYp-VECT[KVY])/DELTA_T)
    
    let (NEXT_STEP = [time+DELTA_T,Xp,Yp,Gamma,VXp,VYp,NewGammaVelo,AXp,AYp,NewGammaAcc,Torq,SetPoint, Xc,Yc,Theta, VXc,VYc, NewThetaVelo, AXc,AYc, NewThetaAcc,999])
    
    let (z = (ECHO_V) ? echo(NEXT_STEP) : 0)
    
    (time+DELTA_T < END_T) ? 
        concat([VECT],time_step_Cart(DELTA_T=DELTA_T,END_T=END_T, Mp=Mp, Lp=Lp, Ip=Ip, Mc=Mc, Lc=Lc, Ic=Ic,WHEELR=WHEELR, VECT=NEXT_STEP,SetPtVSTime=SetPtVSTime, Kp=Kp,Kg=Kg,Kv=Kv, time=time+DELTA_T)) :
        [VECT] ;
// END CART CART CART CART CART CART CART CART

// Pendulum rotational acceleration from moment balance
// where L is the distance from the point of rotation to the center of mass and
// Gamma is the rotation angle, zero is horizontally to the right
function GammaAccel(M,L,I,Gamma) = -G*M*L*cos(Gamma)/I; // Gamma is in degrees

// Body (wheel) acceleration from moment balance
// where R is the radius of the wheel and Xaccel is the body X acceleration
function bodyAccel(R,M,I,Xaccel) = (R*M*Xaccel)/(I*2*PI);

function echo_header() =
    // This function writes header line before data lines for spreadsheet
    let (z = (ECHO_V) ? echo(str(",time,X,Y,rot-rad,X velo,Y velo,rot velo,X accel,Yaccel,rot accel,AppliedTorq,SetPoint Deg,not used,")) : 0)  0;
    
function echo_header_cart() =
    // This function writes header line before data lines for spreadsheet
    let (z = (ECHO_V) ? echo(str(",time,X pend,Y pend,rot Pend-Gamma-rad,XV pend,YV pend,rotVelo Pend,XA pend,YA pend,rot accel Pend,Applied Torq,SetPoint Deg,X wheel,Y wheel,Rot Wheel-Theta-rad,XV wheel,YV wheel,TV wheel,XA wheel,YA wheel,TA wheel,")) : 0)   0;
    
function echo_VEC(VEC) =
    // This function writes a vector, boolean ECHO_V is true
    let (z = (ECHO_V) ? echo(VEC) : 0)   0;

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
module drawSIM(SIM,wheel=false,dispVelo=false,i=0) {
    // draw center of mass indicator
    if (wheel) {
        color("blue") 
        translate([SIM[i][KX],SIM[i][KY],0])
            rotate([0,0,(SIM[i][KR])*180/PI]) {
                cylinder(h=.1,r=0.1,center=true,$fn=FACETS);
                cube([1,0.05,.1],center=true);
            };
        }
    else {
        color("green") 
        translate([SIM[i][KX],SIM[i][KY],0])
            rotate([0,0,(SIM[i][KR])*180/PI]) {
                cylinder(h=.1,r=0.1,center=true,$fn=FACETS);
                cube([1,0.05,.1],center=true);
            };
    };
    if (dispVelo) {
        // draw velocity vector
        VMag = 0.05*sqrt(pow(SIM[i][KVX],2) + pow(SIM[i][KVY],2));
        //color("blue") force_arrow([SIM[i][KX],SIM[i][KY],0],[SIM[i][KVX],SIM[i][KVY],0],mag=VMag);
                
        // draw acceleration vector
        AMag = 0.01*sqrt(pow(SIM[i][KAX],2) + pow(SIM[i][KAY],2));
        color("red") force_arrow([SIM[i][KX],SIM[i][KY],0],[SIM[i][KAX],SIM[i][KAY],0],mag=AMag);
    };

}

// Recursive module to draw simple representations for a simulation
module drawSIM_Vector(SIM=undef,wheel=false,dispVelo=false,i=0) {
    if (SIM[i] != undef) {
        drawSIM(SIM=SIM,wheel=wheel,dispVelo=dispVelo,i=i);
        
        // recursive call to draw next time step
        drawSIM_Vector(SIM=SIM,wheel=wheel,dispVelo=dispVelo,i=i+1);
    };
};

// draw a cart object representation for a single time
module drawCartSIM(SIM,dispVelo=false,Rwheel=10,i=0) {
    // draw center of mass indicator
    color("blue") // wheel
    translate([SIM[i][KX2],SIM[i][KY2],0])
        rotate([0,0,(SIM[i][KR2])*180/PI]) 
            translate([Rwheel/2,0,0]) cube([Rwheel,0.05,.1],center=true);
    
    color("black") // vertical line
    translate([SIM[i][KX2],SIM[i][KY2],0])
        translate([0,-Rwheel/2,0]) cube([0.05,Rwheel,.1],center=true);
    
    color("green") // pendulum
    translate([SIM[i][KX],SIM[i][KY],0]) {
            cylinder(h=.1,r=0.1,center=true,$fn=FACETS);
        rotate([0,0,(-SIM[i][KR])*180/PI]) {
            cube([0.05,6,.1],center=true);
            };
        };
    if (dispVelo) {
        // draw velocity vector
        VMag = 0.05*sqrt(pow(SIM[i][KVX],2) + pow(SIM[i][KVY],2));
        //color("blue") force_arrow([SIM[i][KX],SIM[i][KY],0],[SIM[i][KVX],SIM[i][KVY],0],mag=VMag);
                
        // draw acceleration vector
        AMag = 0.01*sqrt(pow(SIM[i][KAX],2) + pow(SIM[i][KAY],2));
        color("red") force_arrow([SIM[i][KX],SIM[i][KY],0],[SIM[i][KAX],SIM[i][KAY],0],mag=AMag);
    };

}

// Recursive module to draw simple representations for a simulation
module drawCartSIM_Vector(SIM=undef,dispVelo=false,Rwheel=10,i=0) {
    if (SIM[i] != undef) {
        drawCartSIM(SIM=SIM,dispVelo=dispVelo,Rwheel=Rwheel,i=i);
        
        // recursive call to draw next time step
        drawCartSIM_Vector(SIM=SIM,dispVelo=dispVelo,Rwheel=Rwheel,i=i+1);
    };
};

// module to draw xy chart of angle vs time for a simulation
module drawAngVSTimeChart(SIM,Tscale=10,Index=1,Yscale=1.0,Color="blue") {
    l = len(SIM);
    Last_Time = SIM[l-1][KT];
    TimeAngVec = [ for (j = [0:1:l-1])  [SIM[j][KT]*Tscale,SIM[j][Index]*Yscale] ];
    TimeAngVec_1 = concat(TimeAngVec,[[Last_Time*Tscale,SIM[0][Index]*Yscale]]);
    color(Color,.5) polygon(TimeAngVec_1);
};

module drawXVSTimeChart(VEC,Tscale=10,Yscale=0.01) {
    l = len(VEC);
    Last_Time = VEC[l-1][0];
    TimeXVec = [ for (j = [0:1:l-1])  [VEC[j][0]*Tscale,VEC[j][1]*Yscale] ];
    TimeXVec_1 = concat(TimeXVec,[[Last_Time*Tscale,VEC[0][1]*Yscale-.1]]);
    color("yellow",.5) polygon(TimeXVec_1);
};
function integrateVec(VEC,Tstep=0.01,ECHO_V=false,area=0,time=0) =
    let (l = len(VEC))
    let (End_Time = VEC[l-1][0])
    let (V0 = lookup(time,VEC))
    let (V1 = lookup(time+Tstep,VEC))
    let (trapezoidalArea = ((V0 + V1)/2)*Tstep)
    let (new_area = area + trapezoidalArea)
    (time+Tstep < End_Time+1.5*Tstep) ? 
        let (z = (ECHO_V) ? echo(str(time,",",area)) : 0)
        concat([[time,new_area]],integrateVec(VEC=VEC,Tstep=Tstep,ECHO_V=ECHO_V,area=new_area,time=time+Tstep)) :
        echo(str("Final Integration: ",new_area)) 
        [] ;

function pitchVec(ACCELVEC,Tstep=0.01,ECHO_V=false,time=0) =
    let (l = len(ACCELVEC))
    let (End_Time = ACCELVEC[l-1][0])
    let (pitch = atan2(lookup(time,ACCELVEC),G)) // atan2(y,x)
    (time+Tstep < End_Time+1.5*Tstep) ? 
        let (z = (ECHO_V) ? echo(str(time,",",pitch)) : 0)
        concat([[time,pitch]],pitchVec(ACCELVEC=ACCELVEC,Tstep=Tstep,ECHO_V=ECHO_V,time=time+Tstep)) :
        [] ;

function derivativeVec(VEC,Tstep=0.01,ECHO_V=false,time=0) =
    let (l = len(VEC))
    let (End_Time = VEC[l-1][0])
    let (V0 = lookup(time,VEC))
    let (V1 = lookup(time+Tstep,VEC))
    let (slope = (time==0) ? 0 : ((V1-V0)/Tstep)) // force first value to be 0
    (time+Tstep < End_Time+1.5*Tstep) ? 
        let (z = (ECHO_V) ? echo(str(time,",",slope)) : 0)
        concat([[time,slope]],derivativeVec(VEC=VEC,Tstep=Tstep,ECHO_V=ECHO_V,time=time+Tstep)) :
        [] ;

function makeVeloProfile(TIME=1.0,DIST=4.0) = 
    // Velocity Profile for an approximate Triangle Wave Acceleration
    let (MAX_V = 2*DIST/TIME)  // check if value is reasonable on robot
    let (MAX_A = 4*MAX_V/TIME) // for echo
    echo(str("Maximum Velocity = ",MAX_V,", Maximum Acceleration = ",MAX_A))
    [[.00*TIME,.00*MAX_V],
    [.05*TIME,.02*MAX_V],
    [.10*TIME,.08*MAX_V],
    [.15*TIME,.18*MAX_V],
    [.20*TIME,.32*MAX_V],
    [.25*TIME,.50*MAX_V],
    [.30*TIME,.68*MAX_V],
    [.35*TIME,.82*MAX_V],
    [.40*TIME,.92*MAX_V],
    [.45*TIME,.98*MAX_V],
    [.50*TIME,1.00*MAX_V],
    [.55*TIME,.98*MAX_V],
    [.60*TIME,.92*MAX_V],
    [.65*TIME,.82*MAX_V],
    [.70*TIME,.68*MAX_V],
    [.75*TIME,.50*MAX_V],
    [.80*TIME,.32*MAX_V],
    [.85*TIME,.18*MAX_V],
    [.90*TIME,.08*MAX_V],
    [.95*TIME,.02*MAX_V],
    [1.00*TIME,.00*MAX_V]];

    //[[0,0],[0.125*TIME,0.122*MAX_V], [0.25*TIME,0.5*MAX_V], [0.375*TIME,0.875*MAX_V], [0.5*TIME,MAX_V], [0.625*TIME,0.875*MAX_V], [0.75*TIME,0.5*MAX_V], [0.875*TIME,0.122*MAX_V], [TIME,0]];

// NOT WORKING.  CALCULUS FOR SECOND AND THIRD SEGMENT IS WRONG
function mkTriangleAwaveVeloProfile(TIME=1.0,DIST=1.0) = 
    // Velocity Profile for an approximate Triangle Wave Acceleration
    // Using calculus derived formulas
    let (MAX_V = 2*DIST/TIME)  // check if value is reasonable on robot
    let (MAX_A = 4*MAX_V/TIME) // for echo
    let (A_SLOPE = MAX_A/(TIME/4))
    echo(str("Maximum Velocity = ",MAX_V,", Maximum Acceleration = ",MAX_A,", A SLOPE = ",A_SLOPE))
    [for (t =[0:TIME/20:TIME]) t<TIME/4 ? [t,(A_SLOPE/2)*t^2] : t<TIME*3/4 ? [t,(-(A_SLOPE/2)*t^2)] : 
        [t,2*t^2 + t] ];

module make_profiles(DIST=1,TIME=1,Tscale=1) { 
    // DIST is in cm
    // 
    // Use this to make the Position and Pitch Vectors
    //
    MAX_V = 2*DIST/TIME;  
    MAX_A = 4*MAX_V/TIME; // if Maximum Acceleration is > G, then problems?
    
    echo(str("For distance of ",DIST," (cm) in ",TIME," seconds, Max Velocity is ",MAX_V," (cm/sec) and Max Acceleration is ",MAX_A," cm/sec^2"));
    T = TIME;
    ACC_VEC = [[0,0],[0.25*T,MAX_A],[0.5*T,0],[0.75*T,-MAX_A],[T,0]];
    PITCH_VEC = pitchVec(ACCELVEC=ACC_VEC,Tstep=0.05,ECHO_V=true);
    VELO_VEC = integrateVec(ACC_VEC,Tstep=0.05,ECHO_V=false); // Use ECHO for spreadsheet
    POS_VEC = integrateVec(VELO_VEC,Tstep=0.05,ECHO_V=true);  // Use ECHO for spreadsheet
    translate([0,0,0]) {
        drawAngVSTimeChart(SIM=ACC_VEC,Tscale=Tscale,Index=1,Yscale=1,Color="blue");
        drawAngVSTimeChart(SIM=VELO_VEC,Tscale=Tscale,Index=1,Yscale=1,Color="yellow");
        drawAngVSTimeChart(SIM=POS_VEC,Tscale=Tscale,Index=1,Yscale=1,Color="orange");
        drawAngVSTimeChart(SIM=PITCH_VEC,Tscale=Tscale,Index=1,Yscale=1,Color="green");
    };
};
*make_profiles(DIST=20,TIME=2,Tscale=10);

VELO_PROFILE = false;
if (VELO_PROFILE) { // check velocity profiles
    TIME = 1;
    DIST = .5; // whatever units
    VELO_VEC = makeVeloProfile(TIME=TIME,DIST=DIST);
    //VELO_VEC2 = mkTriangleAwaveVeloProfile(TIME=TIME,DIST=DIST); // calculus version, not working
    POS_VEC = integrateVec(VELO_VEC,Tstep=0.01);
    ACC_VEC = derivativeVec(VELO_VEC,Tstep=0.01,ECHO_V=false);
    drawAngVSTimeChart(SIM=VELO_VEC,Tscale=1,Index=1,Yscale=1,Color="green");
    //drawAngVSTimeChart(SIM=VELO_VEC2,Tscale=1,Index=1,Yscale=1,Color="lawngreen");
    drawAngVSTimeChart(SIM=POS_VEC,Tscale=1,Index=1,Yscale=1,Color="red");
    drawAngVSTimeChart(SIM=ACC_VEC,Tscale=1,Index=1,Yscale=1,Color="blue");
};
//  ## DATA BEGINES HERE ##

// Pendulum
Pend = false; // Simple Pendulum
if (Pend) {
    BRASS_OBJ = ["BRASS"       ,"CUBE","gold" ,9.73      ,2.5     ,5.0 ,4.2   ,0,20.0  ,0];
    ARM_OBJ   = ["Arm"         ,"CUBE","silver",ALUM_DEN, 2.80    ,28.0,0.34   ,0,12.0  ,0]; 
    OBJECTS = [BRASS_OBJ,ARM_OBJ];

    // Get total mass properties for object
    om = Mass_Totals(OBJECTS);
    M1 = om[0];
    I1 = om[1];
    CM1 = abs(om[2][1]);

    END_TIME = 1.0;  // seconds, full cycle for simple = 0.29, compound = 0.74
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
    Set_Point_Vect = [[0,INIT_ANG],[.02,INIT_ANG],[.04,0],[1.0,0],[1.02,-60],[END_TIME,-60]];

    dummy = echo_header(); // for spreadsheet
    
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,CM1*cos(INIT_ANG),CM1*sin(INIT_ANG),IAR, 0,0,0,0,0,GammaAccel(M=M1,L=CM1,I=I1,Gamma=INIT_ANG),0,-INIT_ANG];
    
    SIM1=time_step_Pendulum(DELTA_T=DT,END_T=END_TIME,M=M1,L=CM1,I=I1,PRIOR=KIN_0, SetPtVSTime=Set_Point_Vect, Kp=-100, Kg=M1*CM1, Kv=-1800);

    drawSIM_Vector(SIM=SIM1,dispVelo=true);

    //drawXVSTimeChart(VEC=Set_Point_Vect,Tscale=100,Yscale=1);
    //drawAngVSTimeChart(SIM=SIM1,Tscale=100,Index=KR,Yscale=180/PI);

    color("red") rotate([0,0,INIT_ANG-90]) translate([0,CM1,0]) sphere(r=0.4,$fn=FACETS);
    drawObjects(OBJ=OBJECTS); // put last to display propertly

};

// Wobble Wheel
WW = false;  // Wobble Wheel
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

    END_TIME = 0.5;  // seconds, full cycle for simple = 0.29, compound = 0.74
    DT = 0.01; // delta time in seconds
    echo(str("End Time = ",END_TIME,", Time Step = ",DT,", Number of time steps = ",END_TIME/DT));

    INIT_ANG = 30; // DEG
    IAR = INIT_ANG*PI/180;  // initial angle radians
    INIT_POS = XYfromWheelAng(INIT_ANG,L=CM1,RAD_WHEEL=RAD_WHEEL); // only used by Wobble wheel
    FINAL_POS = [PI*WHEEL_DIA-INIT_POS[0],INIT_POS[1],0]; // only used by Wobble wheel

    // Simple pendulum frequency = 2*PI*sqrt(L/G),  for swing < 30 deg
    SimpleFreq = 2*PI*sqrt(CM1/G);
    echo(str("Simple Frequency = ",SimpleFreq," seconds"));
    // Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr)) ,  for swing < 30 deg
    CompoundFreq = 2*PI*sqrt(I1/(M1*G*CM1));
    echo(str("Compound Frequency = ",CompoundFreq," seconds"));

    dummy = echo_header(); // for spreadsheet
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,INIT_POS[0],INIT_POS[1],IAR, 0,0,0,0,0,GammaAccel(M=M1,L=CM1,I=I1,Gamma=INIT_ANG)];
    
    SIM1=time_step_Wobble_Wheel(DELTA_T=DT,END_T=END_TIME,M=M1,L=CM1,I=I1, WHEELR=RAD_WHEEL, PRIOR=KIN_0);
    
    max_gamma(v=SIM1,val=IAR-.1); // echo the period time
    drawSIM_Vector(SIM=SIM1,wheel=true,dispVelo=true);
    
    translate([((90-INIT_ANG)*PI/180)*RAD_WHEEL,0,0]) {
        color("red") rotate([0,0,INIT_ANG-90]) translate([0,CM1,0]) sphere(r=0.4,$fn=FACETS);
        drawObjects(OBJ=OBJECTS); // put last to display propertly
    };
    
    translate(FINAL_POS) color("blue") cube([0.5,10,1],center=true); // indicator for how far ww moves
};

// Two Wheel Robot with 6" dia wheels, expansion Hub, Phone, low center of mass
TWR6 = false;
if (TWR6) {
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
    
    END_TIME = 6.0;  // seconds, full cycle for simple = 0.29, compound = 0.74
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

    // Set Point vector to set desired MOTOR VELOCITY vs time
    NEW_VELO = 20; // rad/sec 
    Set_Point_VELO = [[0,0],[0.02,0],[0.03,NEW_VELO],[3.0,NEW_VELO],[3.1,0],[END_TIME,0]];

    dummy = echo_header_cart(); // for spreadsheet
    
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar, MT, SP, x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,CMbody*cos(INIT_ANG),CMbody*sin(INIT_ANG),IAR, 0,0,0,0,0,0,0,INIT_ANG, 0,0,IAR, 0,0,0, 0,0,0,999];
    
    dummy2 = echo_VEC(VEC=KIN_0);
    
    SIM1=    time_step_Cart(DELTA_T=DT,END_T=END_TIME,Mp=Mbody,Lp=CMbody,Ip=Ibody,Mc=Mwheels, Lc=CMwheels, Ic=Iwheels,WHEELR=RAD_WHEEL,VECT=KIN_0,SetPtVSTime=Set_Point_VELO,Kp=-12,Kg=0,Kv=100);
    // SetPtVSTime=Set_Point_ANG,Kp=-20,Kg=Mbody*CMbody,Kv=-1950);

    *drawCartSIM_Vector(SIM=SIM1,dispVelo=false,Rwheel=RAD_WHEEL);

    TS=10;
    translate([0,30,0]) {
        drawXVSTimeChart(VEC=Set_Point_VELO,Tscale=TS,Yscale=1);
        drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KVX2,Yscale=1,Color="blue");
        drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KMT,Yscale=.1,Color="red"); // motor torque
        *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KR,Yscale=20,Color="green");

        color("red") translate([0,CMwheels,0]) sphere(r=1,$fn=FACETS);
        color("orange") translate([0,-CMbody,0]) sphere(r=1,$fn=FACETS);
        drawObjects(OBJ=WHEELS); // put last to display propertly
        drawObjects(OBJ=BODY); // put last to display propertly
    };

};
// Two Wheel Robot with Blue 8" (20.2 cm) dia wheels, Control Hub, balancing
TWR8 = false;
if (TWR8) {
    // Each Blue wheel, mass = 302 g, Outside dia = 20.2 cm, Inside dia = 16.4 cm, thickness = 2.54 cm
    WHEEL_DIA = 20.2; // cm (8 inch)
    W_HOLE_DIA = 16.4; // cm 
    WHEEL_DENS = 1.1 ; // g/cm^3  (rubber).  Does not include Hub at this time
    RAD_WHEEL = WHEEL_DIA/2;
    
    // Oject v=["NAME",  "TYPE",       "COLOR", densi   ,X_Size,Y_Size  ,Z_Size,X_CM,Y_CM,Z_CM];
    WHEEL1_OBJ =["WHEEL1",      "CYL" ,"blue",WHEEL_DENS ,WHEEL_DIA ,2.54,0    ,0,0          ,-15];
    WHEEL1_HOLE_OBJ=["WHEEL1-H","CYL","white",-WHEEL_DENS,W_HOLE_DIA,2.54,0    ,0,0          ,-15];
    WHEEL2_OBJ =["WHEEL2",      "CYL" ,"blue",WHEEL_DENS ,WHEEL_DIA ,2.54,0    ,0,0          ,15];
    WHEEL2_HOLE_OBJ=["WHEEL2-H","CYL","white",-WHEEL_DENS,W_HOLE_DIA,2.54,0    ,0,0          ,15];
    
    //  max torque for a REV HD Hex SPUR 40:1 is  43,000 g-force - cm
    MTR_DENS = 3.48;
    MTR_D = 3.5;
    MTR_L = 10.0;
    MOTOR1 = ["MOTOR1", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,-1,10];
    MOTOR2 = ["MOTOR2", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,-1,-10];
    
    //HUB_OBJ =["HUB", "CUBE", "black",     0.473     ,10.3,3.0,14.3, 0,-5.5,0]; // Hub mass = 209 grams
    HUB_OBJ =["HUB", "CUBE", "black",     0.473     ,14.3,3.0,10.3, 0,  3.0  ,0]; // Hub mass = 209 grams
    BATTERY = ["BATTERY","CUBE","green", 0.94, 13.0,3, 10, 0,  6.5  ,0]; // Battery mass = 685 grams
    // total mass = 4588 grams
    PLATE = ["PLATE","CUBE","gold", 9.69,  9,0.5,26, 0,  4.7  ,0]; // adjust density to get total mass
    BRASS_CUBE1 = ["BRASS1","CUBE","goldenrod",BRASS_DENS,7,3.5,3.5, 0, 7 ,10]; //
    BRASS_CUBE2 = ["BRASS2","CUBE","goldenrod",BRASS_DENS,7,3.5,3.5, 0, 7 ,-10]; //
    
    // Freight, three different weights
    BOX_LIGHT = ["BOX-LIGHT","CUBE","yellow",0.385,5.08,5.08,5.08,0,15,0];
    BOX_MED =   ["BOX-MED",  "CUBE","gold"  ,0.71 ,5.08,5.08,5.08,10,15,0];
    BOX_HEAV =  ["BOX-HEAV", "CUBE","orange",1.03 ,5.08,5.08,5.08,-20,15,0];
    
    WHEELS = [WHEEL1_OBJ,WHEEL1_HOLE_OBJ,WHEEL2_OBJ,WHEEL2_HOLE_OBJ,MOTOR1,MOTOR2] ;
    BODY = [HUB_OBJ, BATTERY, PLATE,BRASS_CUBE1,BRASS_CUBE2] ;

    // Get total mass properties for object
    wm = Mass_Totals(WHEELS,"Wheels & Motors");
    Mwheels = wm[0];
    Iwheels = wm[1];
    CMwheels = abs(wm[2][1]);
    bm = Mass_Totals(BODY,"Hub,Bat,Plate,Brass");
    Mbody = bm[0];
    Ibody = bm[1];
    CMbody = abs(bm[2][1]);
    
    END_TIME = 6.0;  // seconds, full cycle for simple = 0.29, compound = 0.74
    DT = 0.005; // delta time in seconds
    echo(str("End Time = ",END_TIME,", Time Step = ",DT,", Number of time steps = ",END_TIME/DT));

    INIT_ANG = -90; // DEG
    IAR = INIT_ANG*PI/180;  // initial angle radians

    // Simple pendulum frequency = 2*PI*sqrt(L/G),  for swing < 30 deg
    SimpleFreq = 2*PI*sqrt(CMbody/G);
    echo(str("Body Simple Frequency = ",SimpleFreq," seconds"));
    // Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr)) ,  for swing < 30 deg
    CompoundFreq = 2*PI*sqrt(Ibody/(Mbody*G*CMbody));
    echo(str("Body Compound Frequency = ",CompoundFreq," seconds"));

    // Set Point vector to set desired MOTOR VELOCITY vs time
    NEW_VELO = 60; // rad/sec 
    //Set_Point_VELO = [[0,0],[0.02,0],[0.03,NEW_VELO],[3.0,NEW_VELO],[3.1,0],[END_TIME,0]];
    Set_Point_VELO = makeVeloProfile(TIME=END_TIME-4,DIST=50);

    dummy = echo_header_cart(); // for spreadsheet
    
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar, MT, SP, x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,CMbody*cos(INIT_ANG),CMbody*sin(INIT_ANG),IAR, 0,0,0,0,0,0,0,INIT_ANG, 0,0,IAR, 0,0,0, 0,0,0,999];
    
    dummy2 = echo_VEC(VEC=KIN_0);
    
    //SIM1=    time_step_Cart(DELTA_T=DT,END_T=END_TIME,Mp=Mbody,Lp=CMbody,Ip=Ibody,Mc=Mwheels, Lc=CMwheels, Ic=Iwheels,WHEELR=RAD_WHEEL,VECT=KIN_0,SetPtVSTime=Set_Point_VELO,Kp=-14,Kg=0,Kv=150);
    // SetPtVSTime=Set_Point_ANG,Kp=-20,Kg=Mbody*CMbody,Kv=-1950);

    *drawCartSIM_Vector(SIM=SIM1,dispVelo=false,Rwheel=RAD_WHEEL);

    TS=10;
    *drawXVSTimeChart(VEC=Set_Point_VELO,Tscale=TS,Yscale=1); // velocity setpoint
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KVX2,Yscale=1,Color="blue"); // horizontal velocity
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KMT,Yscale=.1,Color="red"); // motor torque
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KX,Yscale=1,Color="green"); // horizontal position
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KR,Yscale=10,Color="brown"); // pendulum rotation gamma

    color("red") translate(wm[2]) sphere(r=1,$fn=FACETS);
    
    AA = 0; 
    color("orange") rotate([0,0,AA]) translate(bm[2]) sphere(r=1,$fn=FACETS);
    *drawObjects(OBJ=WHEELS); // put last to display propertly
    drawObjects(OBJ=BODY); // put last to display propertly

};

// Two Wheel Robot with Black 12 cm dia wheels, Control Hub
TWR12cm = false;
if (TWR12cm) {
    echo("TWO WHEEL ROBOT WITH 12 CM BLACK WHEELS, MASS ABOVE AXIS");
    // Each Black wheel, mass = 54 g, Outside dia = 12 cm, Inside dia = 0 cm, thickness = .57 cm
    WHEEL_DIA = 12; // cm 
    WHEEL_DENS = 0.84 ; // g/cm^3.  Does not include Hub
    RAD_WHEEL = WHEEL_DIA/2;
    
    // Oject v=["NAME",  "TYPE",       "COLOR", densi   ,X_Size,Y_Size  ,Z_Size,X_CM,Y_CM,Z_CM];
    WHEEL1_OBJ =["WHEEL1",      "CYL" ,"black",WHEEL_DENS ,WHEEL_DIA ,.57,0    ,0,0          ,-15];
    WHEEL2_OBJ =["WHEEL2",      "CYL" ,"black",WHEEL_DENS ,WHEEL_DIA ,.57,0    ,0,0          ,15];
    
    //  max torque for a goBilda Yellow Jacket 19.2:1 is  24,300 g-force - cm
    MTR_DENS = 4.7; // back solved, total mass with hub = 456 g
    MTR_D = 3.5;
    MTR_L = 10.0;
    MOTOR1 = ["MOTOR1", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,0,10];
    MOTOR2 = ["MOTOR2", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,0,-10];
    
    //HUB_OBJ =["HUB", "CUBE", "black",     0.473     ,10.3,3.0,14.3, 0,-5.5,0]; // Hub mass = 209 grams
    HUB_OBJ =["HUB", "CUBE", "black",     0.473     ,14.3,3.0,10.3, 0,6.0,0]; // Hub mass = 209 grams
    BATTERY = ["BATTERY","CUBE","green", 1.6, 13.0,3, 10, 0,9.1,0]; // Battery mass = 620 grams
    PLATE = ["PLATE","CUBE","white", 2.05,15.5,0.5,29.5, 0,4.2,0]; // adjust density to get total mass
    
    WHEELS = [WHEEL1_OBJ,WHEEL2_OBJ,MOTOR1,MOTOR2] ;
    BODY = [HUB_OBJ, BATTERY, PLATE] ;

    // Get total mass properties for object
    wm = Mass_Totals(WHEELS);
    Mwheels = wm[0];
    Iwheels = wm[1];
    CMwheels = abs(wm[2][1]);
    bm = Mass_Totals(BODY);
    Mbody = bm[0];
    Ibody = bm[1];
    CMbody = abs(bm[2][1]);
    
    END_TIME = .1;  // seconds, full cycle for simple = 0.29, compound = 0.74
    DT = 0.005; // delta time in seconds
    echo(str("End Time = ",END_TIME,", Time Step = ",DT,", Number of time steps = ",END_TIME/DT));

    INIT_ANG = 0; // DEG
    IAR = INIT_ANG*PI/180;  // initial angle radians

    // Simple pendulum frequency = 2*PI*sqrt(L/G),  for swing < 30 deg
    SimpleFreq = 2*PI*sqrt(CMbody/G);
    echo(str("Body Simple Frequency = ",SimpleFreq," seconds"));
    // Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr)) ,  for swing < 30 deg
    CompoundFreq = 2*PI*sqrt(Ibody/(Mbody*G*CMbody));
    echo(str("Body Compound Frequency = ",CompoundFreq," seconds"));

    // Set Point vector to set desired MOTOR VELOCITY vs time
    NEW_VELO = .001; // rad/sec 
    Set_Point_VELO = [[0,0],[0.01,0],[0.04,NEW_VELO],[0.05,NEW_VELO],[0.08,0],[END_TIME,0]];
    //Set_Point_VELO = [[0,0],[END_TIME,0]];
    //Set_Point_VELO = makeVeloProfile(TIME=END_TIME-4,DIST=50);

    dummy = echo_header_cart(); // for spreadsheet
    
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar, MT, SP, x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,bm[2][0],bm[2][1],IAR, 0,0,0,0,0,0,0,0, 0,0,0, 0,0,0, 0,0,0,999];
    
    dummy2 = echo_VEC(VEC=KIN_0);
    
    SIM1=    time_step_Cart(DELTA_T=DT,END_T=END_TIME,Mp=Mbody,Lp=CMbody,Ip=Ibody,Mc=Mwheels, Lc=CMwheels, Ic=Iwheels,WHEELR=RAD_WHEEL,VECT=KIN_0,SetPtVSTime=Set_Point_VELO,Kp=14,Kg=0,Kv=150);
    // SetPtVSTime=Set_Point_ANG,Kp=-20,Kg=Mbody*CMbody,Kv=-1950);

    drawCartSIM_Vector(SIM=SIM1,dispVelo=false,Rwheel=RAD_WHEEL);

    TS=10;
    *drawXVSTimeChart(VEC=Set_Point_VELO,Tscale=TS,Yscale=1); // velocity setpoint
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KVX2,Yscale=1,Color="blue"); // horizontal velocity
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KMT,Yscale=1,Color="red"); // motor torque
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KX,Yscale=1,Color="green"); // horizontal position
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KR,Yscale=1,Color="brown"); // pendulum rotation gamma

    color("red") translate([0,CMwheels,0]) sphere(r=1,$fn=FACETS);
    
    AA = 0; 
    color("orange") rotate([0,0,AA]) translate(bm[2]) sphere(r=1,$fn=FACETS);
    drawObjects(OBJ=WHEELS); // put last to display propertly
    drawObjects(OBJ=BODY); // put last to display propertly

};

// Freight Frenzy Two Wheel Robot with Blue 8" (20.2 cm) dia wheels, Control Hub
FRGTFRZY = true;
if (FRGTFRZY) {
    // total "new" robot mass = 3392 grams (8/9/2025, al side plates, 16 cm arm tube)
    // total robot mass = 4105 grams (8/2/2025, no freight, no Omni)
    // total robot mass = 4300 grams (6/27/2025, no freight)
    
    LOW = false; // LOW CG robot for true, HIGH CG robot for false
    YS = LOW ? -1 : 1; // Y direction sine
    
    NEW = true; // new (side plates) or old (freight frenzy printed holders)

    // Oject v=["NAME",  "TYPE","COLOR", densi,X_Size,Y_Size  ,Z_Size,X_CM,Y_CM,Z_CM];
        
    // Freight, same size, three different weights/density
    BOX_LIGHT = ["BOX-LIGHT","CUBE","yellow",0.385,5.08,5.08,5.08, 0,0,0]; // 52 g
    BOX_MED =   ["BOX-MED",  "CUBE","gold"  ,0.71 ,5.08,5.08,5.08, 0,0,0];
    BOX_HEAV =  ["BOX-HEAV", "CUBE","orange",1.03 ,5.08,5.08,5.08, 0,0,0];
    translate([20,-7.5,0]) drawObjects([BOX_LIGHT]);

    // Each Blue wheel, mass = 302 g, Outside dia = 20.2 cm, Inside dia = 16.4 cm, thickness = 2.54 cm
    WHEEL_DIA = 20.2; // cm (8 inch)
    W_HOLE_DIA = 16.4; // cm 
    WHEEL_DENS = 1.1 ; // g/cm^3  (rubber).  Does not include Hub at this time
    RAD_WHEEL = WHEEL_DIA/2;
    
    WHEEL1_OBJ =["WHEEL1",      "CYL" ,"blue",WHEEL_DENS ,WHEEL_DIA ,2.54,0 ,0,0 ,-18];
    WHEEL1_HOLE_OBJ=["WHEEL1-H","CYL","white",-WHEEL_DENS,W_HOLE_DIA,2.54,0 ,0,0 ,-18];
    WHEEL2_OBJ =["WHEEL2",      "CYL" ,"blue",WHEEL_DENS ,WHEEL_DIA ,2.54,0 ,0,0 ,18];
    WHEEL2_HOLE_OBJ=["WHEEL2-H","CYL","white",-WHEEL_DENS,W_HOLE_DIA,2.54,0 ,0,0  ,18];
    
    //  max torque for a REV HD Hex SPUR 40:1 is  43,000 g-force - cm
    MTR_DENS = 3.48;  // to equal 335 grams
    MTR_D = 3.5;
    MTR_L = 10.0;
    MOTOR1 = ["MOTOR1", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,YS*1,11];
    MOTOR2 = ["MOTOR2", "CYL","silver", MTR_DENS,MTR_D,MTR_L,0,0,YS*1,-11];
    
    HUB1 = ["HUB1", "CYL","black", .5,6,4,0,0,0,18];
    HUB2 = ["HUB2", "CYL","black", .5,6,4,0,0,0,-18];

    WHEELS = [WHEEL1_OBJ,WHEEL1_HOLE_OBJ,WHEEL2_OBJ,WHEEL2_HOLE_OBJ,HUB1,HUB2,MOTOR1,MOTOR2] ;
        
    // BODY OBJECTS
    ARM_AXLE_DIST = NEW ? 15.0 : 4.0; // cm, distance from shaft to wheel axis, due to Spur motor
    // Hub mass = 209 grams
    HUB_OBJ = NEW ?
    ["REV HUB", "CUBE", "black", 0.55 ,14.3,3.0,10.3, 0,  YS*4.0  ,0] :
    ["REV HUB", "CUBE", "black", 0.48 ,14.3,3.0,10.3, 0,  -1*YS*3.0  ,10];
    // Battery mass (REV slim) = 610 grams 
    BATTERY = NEW ?
    ["BATTERY","CUBE","green", 1.56, 13.0,10, 3, 0,  YS*8.5  ,-10.5] : 
    ["BATTERY","CUBE","green", 1.56, 13.0,3, 10, 0,  YS*6.5  ,-10.5]; 
    // adjust shaft mass to up total mass
    SHAFT = ["SHAFT","CUBE","silver", ALUM_DEN*1.35,1.6,1.6,31,0,YS*ARM_AXLE_DIST,0]; 
    PLATE1  = ["PLATE1","CUBE","silver", ALUM_DEN*1.5,3*CM,7.5*CM,0.125*CM,0,3*CM,5*CM];
    PLATE2  = ["PLATE2","CUBE","silver", ALUM_DEN*1.5,3*CM,7.5*CM,0.125*CM,0,3*CM,-5*CM];
    SQ15BEAM1 = ["15MM_BEAM1","CUBE","navy",ALUM_DEN*0.4,1.5,1.5,10*CM,3,2,0];
    SQ15BEAM2 = ["15MM_BEAM2","CUBE","navy",ALUM_DEN*0.4,1.5,1.5,10*CM,-3,2,0];
    BRASS_CUBE1 = ["BRASS1","CUBE","goldenrod",BRASS_DENS,  3.5,3.5,10.5, 0, YS*7 ,10]; 
    ARM_SERVO = ["ARM_SVO", "CYL" ,"blue", 1 ,6 ,5,0    ,0,     YS*ARM_AXLE_DIST ,0];
    
    BODY = NEW ? 
    [HUB_OBJ, BATTERY, SHAFT,PLATE1,PLATE2,SQ15BEAM1,SQ15BEAM2] :
    [HUB_OBJ, BATTERY, SHAFT,BRASS_CUBE1,ARM_SERVO,MOTOR1,MOTOR2];

    // ARM OBJECTS
    // aluminum tube 1" sqr, 0.0625" (1/16") walls, 14.6 cm long = 58 grams, density = 0.62 grams/cm^3
    
    TUBE_LEN = 16; // cm 
    ARM_TUBE = ["ARM_TUBE","CUBE","silver", .62,  2.54,TUBE_LEN, 2.54, 0,YS*ARM_AXLE_DIST+TUBE_LEN/2,0]; // see al tube above
    CLAW_LEN = 14; // cm
    CLAW = ["CLAW","CUBE","purple",   0.25, 5,CLAW_LEN,9, 0,YS*ARM_AXLE_DIST+TUBE_LEN+CLAW_LEN/2,0]; // claw assy = 160 grams
        
    // Omni no longer used
    OMNIS = ["OMNI WHLS", "CYL" ,"black", 1 ,8 ,3,0    ,0,YS*ARM_AXLE_DIST+TUBE_LEN-5,3]; // omni assy = 158 grams
    
    ARM = [ARM_SERVO,ARM_TUBE,CLAW];
    
    // Combo mass object if for calculating body pitch given arm angles
    COMBO = NEW ? 
    [HUB_OBJ, BATTERY, SHAFT,PLATE1,PLATE2,SQ15BEAM1,SQ15BEAM2,ARM_SERVO,ARM_TUBE,CLAW] :
    [HUB_OBJ, BATTERY, SHAFT,BRASS_CUBE1,ARM_SERVO,MOTOR1,MOTOR2,ARM_TUBE,CLAW];

    // Get total mass properties for objects
    wm = Mass_Totals(WHEELS,"WHEELS");
    Mwheels = wm[0];
    Iwheels = wm[1];
    CMwheels = abs(wm[2][1]);
    
    bm = Mass_Totals(BODY,"BODY");
    Mbody = bm[0];
    Ibody = bm[1];
    CMbody = abs(bm[2][1]);
    
    if (false) {
        // trying to get an idea of the control terms
        L=CMbody;
        M=Mwheels;
        m=Mbody;
        G=9806.6; 
        echo(str("m/M=",m/M," (M+m)/M*L=",(M+m)/(M*L)));
    };
 
    am = Mass_Totals(ARM,"Arm,Claw");
    
    cm = Mass_Totals(COMBO,"Combo Arm & Body",false);
    
    TOTALMASS = wm[0]+bm[0]+am[0];
    echo("TOTAL MASS =",TOTALMASS);
    
    color("red") translate(wm[2]) sphere(r=0.5,$fn=FACETS);
   
    AA = 0; // Desired arm angle
    
    NEW_ARM_CG0 = rot_pt_z (pt=[am[2][0],am[2][1]-ARM_AXLE_DIST,0],zang=AA);
    NEW_ARM_CG = [NEW_ARM_CG0[0],NEW_ARM_CG0[1]+ARM_AXLE_DIST,0];  
    // EQUIVALENT MASS CUBE FOR ROTATE ARM TO GET NEW CG  
    ROT_ARM = ["ROT_ARM","CUBE","red", am[0] ,1,1,1,NEW_ARM_CG[0],NEW_ARM_CG[1],0];
    EQ_ARM = [ROT_ARM];
    eq_arm = Mass_Totals(EQ_ARM,"Equivalent Arm",false);
    COMBO2 = [HUB_OBJ, BATTERY, SHAFT,PLATE1,PLATE2,SQ15BEAM1,SQ15BEAM2,ROT_ARM];
    eq_combo = Mass_Totals(COMBO2,"COMBO ROTATED ARM AND BODY",true,false);
    
    // solve for Body pitch Angle, using atan2 on new combo cg
    BA = atan2(eq_combo[2][0],eq_combo[2][1]); // new BP, to keep robot from rolling
    NEW_AA = AA+BA;  // This is the Set arm angle, to get desired arm angle
    echo(str("For Servo Arm Angle,",AA,",Set Pitch,",BA,",New Global Arm Angle,",NEW_AA));
    
    translate([0,0,-35]) drawObjects(OBJ=WHEELS); // moved -Z for Top view
    
    ARP = [0,-ARM_AXLE_DIST,0];  // arm rotition point
    rotate([0,0,BA]) {
        color("orange")  translate(bm[2]) sphere(r=1,$fn=FACETS);
        color("green")  translate(eq_combo[2]) sphere(r=1,$fn=FACETS);

        translate(-YS*ARP) rotate([0,0,AA]) translate(YS*ARP) {
            color("blue") translate(am[2]) sphere(r=.5,$fn=FACETS);
            drawObjects(OBJ=ARM); // put last to display propertly
            
            //color("black") translate(cm[2]) sphere(r=1,$fn=FACETS);
        }
        
        drawObjects(OBJ=BODY); // put last to display propertly

    }
    
    // write out the pitch vector, vs daa desired arm angle
    for (daa = [-140:20:140]) {
        // ba = body pitch, to keep center of gravity over wheels
        NEW_ARM_CG0 = rot_pt_z (pt=[am[2][0],am[2][1]-ARM_AXLE_DIST,0],zang=daa);
        NEW_ARM_CG = [NEW_ARM_CG0[0],NEW_ARM_CG0[1]+ARM_AXLE_DIST,0];  
        // EQUIVALENT MASS CUBE FOR ROTATE ARM TO GET NEW CG  
        ROT_ARM = ["ROT_ARM","CUBE","red", am[0] ,1,1,1,NEW_ARM_CG[0],NEW_ARM_CG[1],0];
        EQ_ARM = [ROT_ARM];
        eq_arm = Mass_Totals(EQ_ARM,"Equivalent Arm",false);
        COMBO2 = [HUB_OBJ, BATTERY, SHAFT,PLATE1,PLATE2,SQ15BEAM1,SQ15BEAM2,ROT_ARM];
        eq_combo = Mass_Totals(COMBO2,"COMBO2",false);
        
        // solve for Body pitch Angle, using atan2 on new combo cg
        ba = atan2(eq_combo[2][0],eq_combo[2][1]); // new BP, to keep robot from rolling
        // Java format for piecewise function
        echo(str("pitchTermVec.addElement(",daa,",",eq_combo[2][1]/cm[2][1],");"));
        *echo(str("pitchAngVec.addElement(",daa,",",ba,"); // new global arm angle is ",daa+ba));
    };
/*            
    END_TIME = 6.0;  // seconds, full cycle for simple = 0.29, compound = 0.74
    DT = 0.005; // delta time in seconds
    echo(str("End Time = ",END_TIME,", Time Step = ",DT,", Number of time steps = ",END_TIME/DT));

    INIT_ANG = -90; // DEG
    IAR = INIT_ANG*PI/180;  // initial angle radians

    // Simple pendulum frequency = 2*PI*sqrt(L/G),  for swing < 30 deg
    //SimpleFreq = 2*PI*sqrt(CMbody/G);
    //echo(str("Body Simple Frequency = ",SimpleFreq," seconds"));
    // Compound Pendulum frequency = 2*PI*sqrt(I/(m*G*CMr)) ,  for swing < 30 deg
    //CompoundFreq = 2*PI*sqrt(Ibody/(Mbody*G*CMbody));
    //echo(str("Body Compound Frequency = ",CompoundFreq," seconds")); 

    // Set Point vector to set desired MOTOR VELOCITY vs time
    NEW_VELO = 60; // rad/sec 
    //Set_Point_VELO = [[0,0],[0.02,0],[0.03,NEW_VELO],[3.0,NEW_VELO],[3.1,0],[END_TIME,0]];
    Set_Point_VELO = makeVeloProfile(TIME=END_TIME-4,DIST=50);

    dummy = echo_header_cart(); // for spreadsheet
    
    // Initial state Vector=[time,x,y,r,  vx,vy,vr,  ax,ay,ar, MT, SP, x,y,r,  vx,vy,vr,  ax,ay,ar]; 
    KIN_0=[0,CMbody*cos(INIT_ANG),CMbody*sin(INIT_ANG),IAR, 0,0,0,0,0,0,0,INIT_ANG, 0,0,IAR, 0,0,0, 0,0,0,999];
    
    dummy2 = echo_VEC(VEC=KIN_0);
    
    //SIM1=    time_step_Cart(DELTA_T=DT,END_T=END_TIME,Mp=Mbody,Lp=CMbody,Ip=Ibody,Mc=Mwheels, Lc=CMwheels, Ic=Iwheels,WHEELR=RAD_WHEEL,VECT=KIN_0,SetPtVSTime=Set_Point_VELO,Kp=-14,Kg=0,Kv=150);
    // SetPtVSTime=Set_Point_ANG,Kp=-20,Kg=Mbody*CMbody,Kv=-1950);

    *drawCartSIM_Vector(SIM=SIM1,dispVelo=false,Rwheel=RAD_WHEEL);

    TS=10;
    *drawXVSTimeChart(VEC=Set_Point_VELO,Tscale=TS,Yscale=1); // velocity setpoint
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KVX2,Yscale=1,Color="blue"); // horizontal velocity
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KMT,Yscale=.1,Color="red"); // motor torque
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KX,Yscale=1,Color="green"); // horizontal position
    *drawAngVSTimeChart(SIM=SIM1,Tscale=TS,Index=KR,Yscale=10,Color="brown"); // pendulum rotation gamma
*/

};