// module to draw a curve from a two dimensional array VEC
module drawXvsTimeChart(VEC,Tscale=10,Yscale=1,YMEMB=1,COLOR="yellow") {
    l = len(VEC);
    Last_Time = VEC[l-1][0];
    TimeXVec = [ for (j = [0:1:l-1])  [VEC[j][0]*Tscale,VEC[j][YMEMB]*Yscale] ];
    TimeXVec_1 = concat(TimeXVec,[[Last_Time*Tscale,VEC[0][YMEMB]*Yscale-.01]]);
    color(COLOR,.5) polygon(TimeXVec_1);
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
        
function sineVec(AMP=1.0,Tstep=0.01,ECHO_V=false,ANGLE=0,TIME=0) =
    // ALWAYS GENERATES 18 PIECES, OR 19 POINTS
    let (value = sin(ANGLE)*AMP)
    (ANGLE <= 180) ? 
        let (z = (ECHO_V) ? echo(str(TIME,",",value)) : 0)
        concat([[TIME,value]],sineVec(AMP=AMP,Tstep=Tstep,
        ECHO_V=ECHO_V,ANGLE=ANGLE+10, TIME=TIME+Tstep)) :
        [] ;

function addVec(VEC,XADD=0,YADD=0) =
      [for (i=[0:len(VEC)-1]) [VEC[i][0]+XADD,VEC[i][1]+YADD] ] ;

function scaleVec(VEC,XADD=1,YADD=1) =
      [for (i=[0:len(VEC)-1]) [VEC[i][0]*XADD,VEC[i][1]*YADD] ] ;
          
// CONCATINATES TWO VECTORS (LISTS)
function cat(L1, L2) = [for (i=[0:len(L1)+len(L2)-1]) 
                        i < len(L1)? L1[i] : L2[i-len(L1)]] ;
// SAME but without indicies
function cat2(L1, L2) = [for(L=[L1, L2], a=L) a];
    
SINEVEC = sineVec(AMP=20,Tstep=.1,ECHO_V=false);
VEC2 = addVec(SINEVEC,2.8); // deceleration curve,  includes cruise
VEC3 = scaleVec(VEC2,1,-1); // swap sign for deceleration
NEWVEC=cat(SINEVEC,VEC3);
VELO_VEC = integrateVec(NEWVEC,Tstep=0.1,ECHO_V=false); // Use ECHO for spreadsheet
POS_VEC = integrateVec(VELO_VEC,Tstep=0.1,ECHO_V=false); // Use ECHO for spreadsheet

//echo(NEWVEC);
drawXvsTimeChart(VEC=NEWVEC,Tscale=10,YMEMB=1,Yscale=1,COLOR="blue");
drawXvsTimeChart(VEC=VELO_VEC,Tscale=10,YMEMB=1,Yscale=1,COLOR="yellow");
drawXvsTimeChart(VEC=POS_VEC,Tscale=10,YMEMB=1,Yscale=1,COLOR="green");

module trapezoidalMotionProfile(DIST=1,maxVel=1,maxAccel=1,Tscale=1) { 
    // Given Distance to move, Maximum Velocity, Maximum Acceleration
    // Solve for Time.
    //   DIST is in cm
    //
    // Converted from JAVA AI
    accelDist = (maxVel*maxVel)/maxAccel;
    vMax = (accelDist > DIST) ? sqrt (DIST * maxAccel) : maxVel;
    if (accelDist > DIST) echo("***NO CRUISE TIME***");
        
    tAccel = vMax / maxAccel; // acceleration time
    dAccel = 0.5* maxAccel * tAccel * tAccel; // acceleration distance
    dCruise = DIST - 2*dAccel;  // cruise distance
    tCruise = dCruise/vMax; // time at cruise
    TIME = 2*tAccel + tCruise;
    
    echo(str("Max Velo is ",vMax," (cm/sec) and Max Accel is ",maxAccel," cm/sec^2"));
    echo(str("Cruise Dist= ",dCruise,",  Cruise Time= ",tCruise," seconds"));
    echo(str("For distance of ",DIST," (cm) it will take ",TIME," seconds"));
    T = TIME;
    ACC_VEC = [[0,0],[tAccel/2,maxAccel],[tAccel,0],[tAccel+tCruise,0],
    [tAccel+tCruise + tAccel/2,-maxAccel],[T,0]];
    VELO_VEC = integrateVec(ACC_VEC,Tstep=0.05,ECHO_V=false); // Use ECHO for spreadsheet
    POS_VEC = integrateVec(VELO_VEC,Tstep=0.05,ECHO_V=false); // Use ECHO for spreadsheet
    translate([0,0,0]) {
        //drawAngVSTimeChart(SIM=ACC_VEC,Tscale=Tscale,Index=1,Yscale=1,Color="blue");
        drawXvsTimeChart(VEC=ACC_VEC,Tscale=Tscale,YMEMB=1,Yscale=1,COLOR="blue");
        drawXvsTimeChart(VEC=VELO_VEC,Tscale=Tscale,YMEMB=1,Yscale=1,COLOR="yellow");
        drawXvsTimeChart(VEC=POS_VEC,Tscale=Tscale,YMEMB=1,Yscale=1,COLOR="green");
    };
};
*trapezoidalMotionProfile(DIST=15,maxVel=20,maxAccel=35,Tscale=10);

