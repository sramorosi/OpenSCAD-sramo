// Large Displacement Beam, Modules and Functions
include <LDB_Indexes.scad>
//use <ME_lib.scad>
use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

// Display intermediate load steps?
Display_steps = true;
// Scale of Force & Moment Display
force_scale = 0.3; // [0.001:.025:10.0]
// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI), PLA=340000,PETG=300000,Polycar=320000
E_PSI = 340000;  // GLOBAL VARIABLE USED IN MANY FUNCTIONS AND MODULES
// ~Stress level at which the part will fail (PSI)
FAILURE_STRESS = 10000;
// This could be tensile failure, compression failure, bending, etc.
// material density (lb per inch^3)
DENSITY = 0.043;
Load_Steps = 3;  // A GOOD NUMBER OF STEPS IS DEPENDENT ON THE PROBLEM

//    // beam from points
    t=.06;  
    w=0.5;
    HGT = 6;  // height of overall beam
    ORIGIN = [1,0,0];  // NOTE, origin is applied seperatly from shape

    pts=[[0,0],[0,HGT]];  // shape
    new_pts = addPoints(pts,.5);  // breaks points into sub lines
    echo(new_pts=new_pts);
    //draw_points(new_pts,dia=0.1);
    
    BEAM1 = beamFromNodes(new_pts,t,w);
    
    NumberBeams = len(BEAM1);
    //echo(BEAM1=BEAM1," n=", len(BEAM1));
    
    Fx = .8;
    Mz = Fx*5/2;
echo(Mz=Mz);
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,0,Mz]]);
    //LOADS1 = concat ( concat(concat([for (i=[1:(NumberBeams/2)-1]) [0,0,0]],[[Fx,0,Fx*HGT/2]]),[for (j=[1:(NumberBeams/2)]) [0,0,0]]),[[0,0,0]]); // << last load is back solved to hold node in location  [-.4,.1,2.4]
       
    echo(LOADS1=LOADS1," n=", len(LOADS1));

    translate(ORIGIN) draw_beam_undeformed(BEAM1); 
    
    Do_Analysis(BEAM1,LOADS1,force_scale,Display_steps,FAILURE_STRESS,E_PSI,DENSITY,ORIGIN,steps=6);
    
    StartingNodes = getNodesFromBeams(BEAM1,ORIGIN[0],ORIGIN[1]);
    echo(StartingNodes=StartingNodes);
    

    // THREE FUNCTION CALLS TO GET FINAL NODES:
    initial_loads = spread_ext_loads(LOADS1); // Spread Loads
    beam_angles = global_angles(BEAM1); // Generate GLOBAL Beam ANGLES, undeformed
    FinalNodes = getFinalNodes(BEAM1,FAILURE_STRESS,E_PSI,DENSITY,initial_loads,beam_angles,beam_angles, ORIGIN, STEPS=6,index=6);
    
    echo(FinalNodes=FinalNodes);
    
    TranslateChildren(StartingNodes,FinalNodes,12) THING();
//


/*    // SINGLE BEAM ANALYSIS, TO CHECK THE FUNCTIONS
    LEN = 3;
    t=0.1;  
    w=0.5;

    // loads on the END are given
    FX2 = 0;
    FY2 = -2.5;  // 2
    M2 = 6;  // 4  , -7

    // loads on the START are solved
    FX1 = -FX2;
    FY1 = -FY2;
    M1 = FY2*LEN+M2;

    I=((w*t*t*t)/12);
    //echo(I=I);

    NP=4;
    RoarkPts = [for (i=[1:NP+2]) 
        let (x = ((i-1)/NP)*LEN)
        [x,Y_MidRoark(FY2,M2,LEN,E_PSI,I,x)]]; 

    color("red") draw_points(RoarkPts,dia=0.03);   
    RoarkAngles = getAnglesFromNodes(RoarkPts,0,0);

    function NL_Beam(NSeg, Len, Fy,M,E,I,x_start=0,y_start=0,i=1) = 
        i <= NSeg ? 
        let(x= (i/NSeg)*Len)
        let(y=Y_MidRoark(Fy,M,Len,E,I,x))
        let(ang=asin((y-y_start)/(Len/NSeg)))
        let(x_end = Len/NSeg*cos(ang) + x_start)
        let(y_end = Len/NSeg*sin(ang) + y_start)
        //echo(i=i,x=x,y=y,ang=ang,x_end=x_end,y_end=y_end)
        concat( [[x_end,y_end]],
            NL_Beam(NSeg,Len,Fy,M,E,I,x_start=x_end,y_start=y_end,i=i+1))
        : [] ;
        
    NL_Beam_Pts = NL_Beam(NP, LEN, FY2,M2,E_PSI,I);
    color("blue") draw_points(NL_Beam_Pts,dia=0.035);
    echo(NL_Beam_Pts=NL_Beam_Pts);
        
    pts=[[0,0],[LEN,0]];  // shape
    BEAM1 = beamFromNodes(pts,t,w);

    // compute weight
    Weight = computeWeight(BEAM1,DENSITY,START=0,END=len(BEAM1));
    echo(Weight=Weight);

    draw_beam_undeformed(BEAM1); 

    LOADS1 = [[FX1,FY1,M1],[FX2,FY2,M2]];
    echo(LOADS1=LOADS1);

    RESULTS = computeBeamChain(BEAM1,LOADS1,FAILURE_STRESS,E_PSI);
    echo("[theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,weight,newTHK,-Fx,-Fy,-m_total]");
    echo(RESULTS=RESULTS);
    NODES = concat([[0,0,0]],getNodeFromResults(RESULTS,[0]));
    echo(NODES=NODES);
    draw_beam_deformed(BEAM1,RESULTS,displayHinge=true);      
    draw_loads(nodes=NODES, loads=LOADS1, torques=LOADS1,scale=force_scale,z_rot=0);

    // Draw tangent line at the end point
    translate([RESULTS[0][Za],RESULTS[0][Zb],0]) 
        rotate([0,0,RESULTS[0][Zthetaend]]) 
            translate([LEN/4,0,0]) cube([LEN/2,t/2,w*1.2],center=true);

    echo("Theta End Roark=",ThetaEndRoark(FY2,M2,LEN,E_PSI,I)," EndLDB = ",RESULTS[0][Zthetaend]);
    echo(RoarkAngles=RoarkAngles);
    echo("Y End Roark=",Y_EndRoark(FY2,M2,LEN,E_PSI,I)," Y LDB = ",RESULTS[0][Zb]);
*/


// ROARK 6TH EDITION BEAM FORMULA SOLUTIONS (TRANSVERSE SHEAR AND MOMENT AT FREE END)

function ThetaEndRoark(FY=1,M=0,LEN=1,E=1000,I=0.1) = 
// Superposition of two Roark Beam formulas for end angle
// Table 3, p100, case 1a and 3a
    (180/PI)*((FY*LEN^2)/(2*E*I)) + (180/PI)*(M*LEN/(E*I));
    
function Y_EndRoark(FY=1,M=0,LEN=1,E=1000,I=0.1) = 
// Superposition of two Roark Beam formulas for end displacement
// Table 3, p100, case 1a and 3a
    (FY*LEN^3)/(3*E*I) + M*LEN^2/(2*E*I);

function Y_MidRoark(FY=1,M=0,LEN=1,E=1000,I=0.1,X=0.5) = 
// Superposition of two Roark Beam formulas for any mid displacement
// Table 3, p100, case 1a and 3a
    let (a = LEN-X)
    let (y_FY = (FY/(6*E*I)*(2*LEN^3 - 3*LEN^2*a + a^3))) // case 1a
    let (y_M = M*X^2/(2*E*I))  // case 3a
    (y_FY + y_M); // return the sum
    
module Do_Analysis(LDB_DEF,NODE_LOADS,fscale,Display_steps,Failure_Stress,E,density,Origin=[0,0,0],steps) {
    echo("******* LARGE DISPLACEMENT 2D BEAM ANALYSIS BASED ON COMPLIANT MECHANISM PRBM *******");
    echo(E=E,Failure_Stress=Failure_Stress,density=density,fscale=fscale);
    echo(LDB_DEF=LDB_DEF);
    echo(NODE_LOADS=NODE_LOADS);
    // perform data checks
    num_beams = count_beams(LDB_DEF);
    echo("NUMBER OF BEAMS IS ",num_beams," NUMBER OF NODES IS ",len(NODE_LOADS));
    ttl_load = sum_loads(NODE_LOADS);
    echo(" TOTAL LOAD IS ",ttl_load);
    if (num_beams > 0 && num_beams == len(NODE_LOADS)-1 && abs(ttl_load) > 0) {

        // compute weight
        Weight = computeWeight(LDB_DEF,density,START=0,END=len(LDB_DEF));
        echo(Weight=Weight);
    
        // Spread Loads. Moments don't include force-moments at this time
        initial_loads = spread_ext_loads(NODE_LOADS);
        //echo(initial_loads=initial_loads);

        // Generate GLOBAL Beam ANGLES, undeformed
        beam_angles = global_angles(LDB_DEF);
        //echo("INITIAL ",beam_angles=beam_angles);

        // MAIN ANALYSIS (MODULE)
        computeStepsModule(f_scale=fscale,LDB_DEF=LDB_DEF,Failure_Stress=Failure_Stress,E=E,density=density,loads=initial_loads, beam_angles=beam_angles,original_angles=beam_angles,Origin=Origin,STEPS=steps,index=steps,displaySteps=Display_steps); 
                
    } else echo("**NUMBER OF BEAMS OR LOADS IS ZERO, TERMINATING**"); 
}

module output_STRESS_MS_Energy(n,loadScale,Results) { 
    echo(n=n," load scale = ",loadScale,
    " σ MAX ",max_tree(Results,Zstressmax),
             " σ MIN ",min_tree(Results,Zstressmin),
             " MIN MS ",min_tree(Results,Zms),
             " MAX MS ",max_tree(Results,Zms),
             " ENERGY ",sum_fwd(Results,0,Zenergy));
}

// Recursive Module to perform analysis in steps (n)
module computeStepsModule(f_scale=1,LDB_DEF,Failure_Stress,E,density,loads,beam_angles,original_angles, Origin, STEPS=1,index=99,displaySteps=true) {
    
    loadScale = (((STEPS)-index)/STEPS);
    
    loads_scaled = scale_int_loads(loads,loadScale); // scale internal loads
    loads_local = rotate_int_loads(loads_scaled,beam_angles); // Convert internal global forces to beam-local forces 
    force_mom_temp = momentsDueToForce(loads_local, LDB_DEF, beam_angles); // Calculate moments due to forces
    beam_moments = sum_moments(force_mom_temp); // Sum moments due to forces
    NEW_loads_local = add_moments_to_loads(loads_local,beam_moments);  // Add moments due-to-forces
    newResults = computeBeamChain(LDB_DEF,NEW_loads_local,Failure_Stress,E);
    Nodes = concat([Origin],getNodeFromResults(newResults,original_angles, x_start=Origin[0], y_start=Origin[1]));
    newAngleVec = getAnglesFromNodes(Nodes,Origin[0],Origin[1]); // update the beam_angles array
       
    output_STRESS_MS_Energy(index,loadScale,newResults);
    
    if ( index>0 ) { // recursion.  Counts down.

        computeStepsModule(f_scale,LDB_DEF,Failure_Stress,E,density,loads,newAngleVec,original_angles, Origin, STEPS, index-1 , displaySteps);
        
        if (displaySteps) { color("red",loadScale+.1) draw_points(Nodes,dia=0.05); }
    }
    if (index==0) {  // last iteration
        
        if (displaySteps) { 
            color("red",loadScale) draw_points(Nodes,dia=0.05); 
            //echo("FINAL_NODES,",Nodes);
            
            draw_loads(nodes=Nodes, loads=loads_scaled, torques=NEW_loads_local,scale=f_scale,z_rot=0);

        }

        translate(Origin) union () draw_beam_deformed(LDB_DEF,newResults,displayHinge=true);

        echo("X DISPLACEMENT MAX ",max_tree(Nodes,Nx),
             "  X DISPLACEMENT MIN ",min_tree(Nodes,Nx));
        echo("Y DISPLACEMENT MAX ",max_tree(Nodes,Ny),
             "  Y DISPLACEMENT MIN ",min_tree(Nodes,Ny));
    }
}

// Recursive function to perform analysis return the node locations
function getFinalNodes(LDB_DEF,Failure_Stress,E,density,loads,beam_angles,original_angles, Origin, STEPS=1,index=99) =
    let (loadScale = (((STEPS)-index)/STEPS))
    let (loads_scaled = scale_int_loads(loads,loadScale))
    let (loads_local = rotate_int_loads(loads_scaled,beam_angles))
    let (force_mom_temp = momentsDueToForce(loads_local, LDB_DEF, beam_angles))
    let (beam_moments = sum_moments(force_mom_temp))
    let (NEW_loads_local = add_moments_to_loads(loads_local,beam_moments))
    let (newResults = computeBeamChain(LDB_DEF,NEW_loads_local,Failure_Stress,E))
    let (Nodes = concat([Origin],getNodeFromResults(newResults,original_angles, x_start=Origin[0], y_start=Origin[1])))
    let (newAngleVec = getAnglesFromNodes(Nodes,Origin[0],Origin[1]))

    index == 0 ? // last step, return nodes
        Nodes
    : // else, reduce index and do recursion
        getFinalNodes(LDB_DEF,Failure_Stress,E,density,loads,newAngleVec,original_angles, Origin, STEPS, index-1)
    ;

module TranslateChildren(StartNodes,FinalNodes,N) {
    T_X = FinalNodes[N][Nx] - StartNodes[N][Nx];
    T_Y = FinalNodes[N][Ny] - StartNodes[N][Ny];
    R_Z = FinalNodes[N-1][Nang];
    translate([T_X,T_Y,0]) 
        //translate([StartNodes[N][Nx],StartNodes[N][Ny],0])
        //rotate([0,0,R_Z]) 
            //translate([-StartNodes[N][Nx],-StartNodes[N][Ny],0])
                children();
}

function computeWeight(LDB_DEF,density,START=0,END=1) = 
    // Loop thru all beam elements and sum weight off each beam
    let (LEN = LDB_DEF[START][Zlen])
    let (THK=LDB_DEF[START][Zthk])
    let (W=LDB_DEF[START][Zw])
    let (AREA = THK*W)
    let (WEIGHT = AREA*LEN*density)
//echo("BEAM",START=START,density=density,LEN=LEN,AREA=AREA,WEIGHT=WEIGHT)
    (START>=END-1 ? WEIGHT : WEIGHT + computeWeight(LDB_DEF,density,START+1,END));
    
function computeBeamChain(LDB_DEF,Internal_Loads,Failure_Stress,E) =    
    // NEW METHOD
    // Loop thru all beam elements and compute beam type, then angular deflections
    // NOTE: length of loads = length of LDB + 1
    let (n = len(LDB_DEF))
    [ for (i=[0:1:n-1]) 
        let (m2 = Internal_Loads[i+1][Zm])
        let (Fx = Internal_Loads[i+1][Zfx])
        let (Fy = Internal_Loads[i+1][Zfy])
        let (LEN = LDB_DEF[i][Zlen])
//echo("BEAM",i=i,Fx=Fx,Fy=Fy,m2=m2)
    NewBeamAngleFromLoads(LEN=LEN ,THK=LDB_DEF[i][Zthk] ,W=LDB_DEF[i][Zw], Fx=Fx, Fy=Fy, M=m2,Failure_Stress=Failure_Stress, E=E,BEAM_NO = i) ];

// Given a Beam Element and Loads, Calculate the deflection
function NewBeamAngleFromLoads(LEN,THK,W,Fx,Fy,M,Failure_Stress,E, BEAM_NO) = 
    let (Iz = ((W*THK*THK*THK)/12))
    let (AREA = THK*W)
    let (cr = 0.7)  // constant characturistic radius, for now
    let (Kaxial = E*AREA/LEN)   // Axial spring rate
    let (NEW_LEN = (Fx / Kaxial) + LEN)  // Axial displacement
    let (y_end = Y_EndRoark(Fy,M,NEW_LEN,E,Iz))  // new method
    let (theta = (abs(y_end) < cr*NEW_LEN*0.9) ? asin(y_end/(cr*LEN)) : 0 ) // degrees 
    let (theta_end = ThetaEndRoark(Fy,M,NEW_LEN,E,Iz)) // new method
    let (a = a_position(NEW_LEN,cr,theta))  // non linear displacement
    let (b = b_position(NEW_LEN,cr,theta))  // non linear displacement
    let (c = THK / 2)              // half thickness
    let (m_total = M - Fx * b + Fy * a)
    let (stressmax = m_total*c/Iz + Fx/a)
    let (stressmin = -m_total*c/Iz + Fx/a)
    let (ms = getSafetyMargin(stressmax,stressmin,Failure_Stress))
    let (newTHK = checkMS(ms,THK,BEAM_NO))  // GIVES MESSAGE TO GAUGE UP
    let (t_rad = theta * PI / 180)      // radians
    let (Krotate = E*Iz/LEN) //force per radian, Only used for energy calc
    let (EnergyRotate = 0.5* Krotate * (t_rad*t_rad))  // PE = 1/2 * K * x ^2
    let (EnergyAxial = 0.5 * Kaxial * (Fx / Kaxial)*(Fx / Kaxial))
    let (energy = EnergyRotate + EnergyAxial)
//echo(BEAM_NO=BEAM_NO,cr=cr,y_end=y_end,theta=theta,theta_end=theta_end,a=a,b=b,ms=ms)
    // QUALITY CHECKING HERE:
    ((abs(y_end) > cr*LEN*0.9) ? 
        echo("***** IMPOSSIBLE BEAM SOLUTION *****",BEAM_NO=BEAM_NO) 
    [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,newTHK,-Fx,-Fy,-m_total] 
       : // no quality problems
    [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,newTHK,-Fx,-Fy,-m_total]);


function getSafetyMargin(stressMax,stressMin,FailureStress) = 
    (abs(stressMax) > abs(stressMin)) ? 
        FailureStress/abs(stressMax)-1 
        : FailureStress/abs(stressMin)-1 ;

function checkMS(ms,thk,BEAMNO) = 
    ms < 0.0 ?
    let (newTHK = thk*(1-ms))
    echo(BEAMNO=BEAMNO,"##### NEGATIVE MARGIN OF SAFETY, GAUGE UP!! ###### NEW THK=",newTHK) newTHK 
    : thk ;  // return current thickness is MS is positive
    
function a_position(L,cr,theta) = L*(1-cr*(1-cos(theta)));
function b_position(L,cr,theta) = cr*L*sin(theta);

// Draws the undeformed beam in the default color
module draw_beam_undeformed(LDBdef,idx = 0) {
    // Parameter idx is hidden for module
    elem_type = LDBdef[idx][Ztype];
    if (elem_type == Qbeam) {  // Note: undefined causes the recursion to stop
        L = LDBdef[idx][Zlen];
        t = LDBdef[idx][Zthk];
        w = LDBdef[idx][Zw];
        LDBdef_ang = LDBdef[idx][Zang];  // Z rotation of beam
//echo("UNDEFORMED BEAM",idx=idx,LDBdef_ang=LDBdef_ang);
        // draw the beam segment
        linear_extrude(height=w,center=true) 
            hull() { 
                circle(d=t,$fn=16);  // START
                rotate([0,0,LDBdef_ang]) translate([L,0,0]) circle(d=t,$fn=16); // END
        }
        // Recursive call generating the next beam
        rotate([0,0,LDBdef_ang]) translate([L,0,0]) draw_beam_undeformed(LDBdef,idx + 1); 
    } 
}

// recursive module that draws the deformed beam.
module draw_beam_deformed(LDBdef,results,displayHinge=false,idx = 0,prior_ang=0) {
    elem_type = LDBdef[idx][Ztype];
    if (elem_type == Qbeam) {  // Note: undefined causes the recursion to stop
        L = LDBdef[idx][Zlen];
        t = LDBdef[idx][Zthk];
        w = LDBdef[idx][Zw];
        LDBdef_ang = LDBdef[idx][Zang];  // Unloaded Z rotation of beam relative to prior beam
        cr = results[idx][Zrad];
        babyL = L*(1-cr);
        end_ang = results[idx][Zthetaend];
        a = results[idx][Za];
        b = results[idx][Zb];
        ms = results[idx][Zms]; 
//echo(idx=idx,elem_type=elem_type,LDBdef_ang=LDBdef_ang,end_ang=end_ang,cr=cr,a=a,b=b);
        // draw the two beam segments 
        color ([val_red(ms),val_green(ms),0.2]) linear_extrude(height=w,center=true) 
            hull() { 
                rotate([0,0,LDBdef_ang]) translate([babyL,0,0]) circle(d=t,$fn=16);
                circle(d=t,$fn=16);  // ZERO,ZERO
        }
        color ([val_red(ms),val_green(ms),0]) linear_extrude(height=w,center=true) 
            hull() { 
                rotate([0,0,LDBdef_ang]) translate([babyL,0,0]) circle(d=t,$fn=16);
                rotate([0,0,LDBdef_ang]) translate([a,b,0]) circle(d=t,$fn=16);
        }
        
        if(displayHinge) color("black") rotate([0,0,LDBdef_ang]) translate([babyL,0,0]) cylinder(h=w*1.2,d=t/2,$fn=16);
            
        // Recursive call generating the next beam
        rotate([0,0,LDBdef_ang]) translate([a,b,0]) rotate([0,0,end_ang]) 
                draw_beam_deformed(LDBdef,results,displayHinge,idx + 1,LDBdef_ang+prior_ang);
    } 
}

// module that draws the loads 
module draw_loads(nodes,loads,torques,scale=1,z_rot=0) {
    //echo("DRAW ",nodes=nodes,loads=loads);
    N=len(loads)-1;
    rotate ([0,0,z_rot]) {
        for (i=[0:N]) {
            fx = loads[i][Zfx];
            fy = loads[i][Zfy];
            moment = torques[i][Zm];
            fmag = sqrt(fx*fx + fy*fy);
//echo("DRAW_LOADS,",i=i,fx=fx,fy=fy,moment=moment,z_rot=z_rot);
            // draw forces and torques
            if (abs(fmag)>0.1) color ("red") 
                translate([nodes[i][0],nodes[i][1],0]) 
                    force_arrow([0,0,0],[fx,fy,0],mag=fmag*scale);
            if (abs(moment)>0.1) color ("blue")
                translate([nodes[i][0],nodes[i][1],0]) 
                    torque_arrow([0,0,0],mag=moment*scale);
        }
    }
}

// sum angles along segments to get global angles.
function global_angles(LDBdef,prior_ang=0) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) LDBdef[i][Zang] ] ;
//    let (new_sum = sum_fwd(LDBdef,i,Zang)) 
//echo("ANGLES",i=i,prior_ang=prior_ang, new_sum=new_sum)
//    new_sum+prior_ang ] ;

function sum_range(ARRAY,start=0,end=1) = 
    let (val = ARRAY[start])
    (start>=end ? val : val + sum_range(ARRAY,start+1,end));
    
//AA = [0,1,2,3,4,5];
//BB = sum_range(AA,0,1);
//echo(BB=BB);

// recursive forward summation function to sum "thing"
// from the start (or start element) to the i'th element 
function sum_fwd(ARRAY,i,thing,start=0) = 
    let (val = ARRAY[i][thing])
    (i==start ? val : val + sum_fwd(ARRAY,i-1,thing,start));

//TEST = [[0,0,0],[0,0,0],[0,1,0],[0,55,0]];
//SUM = sum_fwd(TEST,len(TEST)-1,1);
//echo("FWD",SUM=SUM);

// recursive function to find maximum "thing"  (NOTE: i parameters is hidden-don't supply)
function max_tree(array,thing,i=0) = 
    let (val = array[i][thing])
    (val==undef ? -99999999 : max(val,max_tree(array,thing,i+1)) );

// recursive function to find minimum "thing"  (NOTE: i parameters is hidden-don't supply)
function min_tree(array,thing,i=0) = 
    let (val = array[i][thing])
    (val==undef ? 99999999 : min(val,min_tree(array,thing,i+1)) );

// recursive function to spread the external forces and moments from tail of tree to root
function spread_ext_loads(ext_loads) =
    let (n = len(ext_loads))
    //echo("SPREAD EXT LOADS",n=n)
    [ for (i=[0:1:n-1]) 
    [sum_fwd(ext_loads,n-1,Zfx,i), sum_fwd(ext_loads,n-1,Zfy,i), sum_fwd(ext_loads,n-1,Zm,i)]];

// recursive function to rotate the internal forces from a global system to a beam-local system.  Moments are copied
function rotate_int_loads(int_loads,beam_angles) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) (
        let (fx = int_loads[i][Zfx])
        let (fy = int_loads[i][Zfy])
        let (ang = (i==0? beam_angles[i] :-beam_angles[i-1])) // special Ground case
//echo(i=i,fx=fx,f=fy,ang=ang)
        [rot_x(fx,fy,ang) , rot_y(fx,fy,ang) ,int_loads[i][Zm] ])];

// recursive function to scale the internal forces and moments
function scale_int_loads(int_loads,scale=1) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) 
        let (fx = int_loads[i][Zfx] * scale)
        let (fy = int_loads[i][Zfy] * scale)
        let (moment = int_loads[i][Zm] * scale)
        [fx,fy,moment ]  ];

// calculate moment due to force on current beam 
function momentsDueToForce(loads, LDBdef, angles) = 
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) (
                let (L = LDBdef[i][Zlen])
                let (fy = loads[i+1][Zfy])
                let (m = (fy*L == undef ? 0 : fy*L))
                m 
            )  ];

// recursive function to sum moments from tail of tree to root
function sum_moments(moments) =
    let (n = len(moments))
    [ for (i=[0:1:n-1]) (sum_range(moments,i,n-1)) ];
    
// recursive function to add beam_moments to node_loads
function add_moments_to_loads(node_loads,beam_moments) =
    let (n = len(node_loads))
    [ for (i=[0:1:n-1]) 
        let (moment = node_loads[i][Zm])
        let (moment2 = (i==n-1? 0 : beam_moments[i]))
        [node_loads[i][Zfx],node_loads[i][Zfy],moment+moment2] ];
    
function rot_x (x,y,a) = x*cos(a)-y*sin(a);

function rot_y (x,y,a) = x*sin(a)+y*cos(a);
    
// convert the margin of safety into a red to green value
function val_red(i) = i < .5 ? 1 : i > 1 ? 0 : 2-i*2 ;

function val_green(i) = i < 0 ? 0 : i > .5 ? 1 : i*2 ;

    
// recursive function to count the number of beams and check data
function count_beams(LDBdef,i=0,count=0) =
    (i < len(LDBdef) ? 
    let (length=(LDBdef[i][Zlen] == 0 ? echo("** FOUND ZERO LENGTH BEAM **",i=i) : 0))
    let (thick=(LDBdef[i][Zthk] == 0 ? echo("** FOUND ZERO THICKNESS BEAM **",i=i):0))
    let (width=(LDBdef[i][Zw] == 0 ? echo("** FOUND ZERO WIDTH BEAM **",i=i):0))
    //echo("BEAM",i=i,count=count)
        count_beams(LDBdef,i+1,count+1) 
    : count );

// recursive function to sum the loads, for check data
function sum_loads(LOADS,i=0,running_sum=0) =
    (i < len(LOADS) ?
    let (Fx=LOADS[i][Zfx])
    let (Fy=LOADS[i][Zfy])
    let (m=LOADS[i][Zm])
    let (sum=(Fx+Fy+m)+running_sum)
    sum_loads(LOADS,i+1,sum)
    : running_sum ); 
    
module draw_points(pts,dia=0.1) {
    numPts = len(pts);
    for (i=[0:numPts-1]) translate([pts[i][0],pts[i][1],1]) color("black") circle(dia,$fn=8);
}
// Function to get Nodes from Results recursively (NOTE: last 4 parameters are hidden-don't supply)
// NEED TO APPEND INITIAL ANGLES
function getNodeFromResults(resultsArray,initAngles,x_start=0,y_start=0,ang_start=0, index=0) = 
    index < len(resultsArray) ?
        let (x = resultsArray[index][Za])
        let (y = resultsArray[index][Zb])
        let (ang = (index==0 ? 0 : resultsArray[index-1][Zthetaend]))
        let (initAng = initAngles[index])
        let (sum_ang = initAng + ang + ang_start) // add up to get new angle
        let (x_end = rot_x(x,y,sum_ang) + x_start) // add up to get new x
        let (y_end = rot_y(x,y,sum_ang) + y_start) // add up to get new y 
//echo(index=index,x=x,y=y,y_start=y_start,initAng=initAng,sum_ang=sum_ang)
        concat([ [x_end , y_end ,resultsArray[index][Zthetaend]] ],
        // Recursive call to process the next point
        getNodeFromResults(resultsArray, initAngles, x_end , y_end , sum_ang, index + 1) ) 
    :  [] ;  // Return nothing when all points are processed

// NEW FUNCTION TO PULL NODES FROM A BEAM DEFINITION  ***********************
function getNodesFromBeams(BEAMS,x_start=0,y_start=0,ang_start=0,index=0) =
    index < len(BEAMS) ? 
        index == 0 ? // first node
            concat([ [x_start,y_start,BEAMS[index][Zang] ] ],
            getNodesFromBeams(BEAMS,x_start,y_start, BEAMS[index][Zang],index+1) )
        :  // middle nodes
            let (LEN = BEAMS[index][Zlen])
            let (END_ANG = BEAMS[index][Zang])
            let (ANG = END_ANG + ang_start)
            let (x_end = x_start + LEN*cos(ANG)) 
            let (y_end = y_start + LEN*sin(ANG))
            concat([ [ x_end, y_end, ANG] ] ,
            getNodesFromBeams(BEAMS,x_end,y_end,ANG,index+1) )
        :  // last node
            let (LEN = BEAMS[index-1][Zlen])
            let (END_ANG = BEAMS[index-1][Zang])
            let (ANG = END_ANG + ang_start)
            let (x_end = x_start + LEN*cos(ANG)) 
            let (y_end = y_start + LEN*sin(ANG))
        [ [x_end, y_end, ANG] ]  ; 

function getAnglesFromNodes(NodesArray,x_start=0,y_start=0, index=1) = 
    index < len(NodesArray) ?
        let (x = NodesArray[index][Nx])
        let (y = NodesArray[index][Ny])
        let (beamAng = atan2(y-y_start,x-x_start))
 //echo(index=index,x=x,x_start=x_start,y=y,y_start=y_start,beamAng=beamAng)
        concat([ beamAng ],
           getAnglesFromNodes(NodesArray, x , y , index + 1) ) 
    :  [] ;  // Return nothing when all points are processed

function beamFromNodes(nodes,t,w,MIN_LEN = 1,index=0,prior_ang=0) =
    let (n = len(nodes)-1)
    index < n ? 
    let (length = sqrt((nodes[index][0]-nodes[index+1][0])^2 + (nodes[index][1]-nodes[index+1][1])^2))
    let (ang = atan2((nodes[index+1][1]-nodes[index][1]),(nodes[index+1][0]-nodes[index][0])))
    concat([[Qbeam,length,t,w,ang-prior_ang]],beamFromNodes(nodes,t,w,MIN_LEN,index+1,ang))  : [] ;
    
function addPoints(points, minDistance) = flatten([
    [points[0]], // copy initial point
    for (i = [1:len(points)-1]) concat(
        recursivelyAddPoints(points[i-1], points[i], minDistance),
        [points[i]]
    )  ]  );

function flatten(l) = [ for (a = l) for (b = a) b ] ;  // need to use with addPoints 

function recursivelyAddPoints(pointA, pointB, minDistance) = 
    let(
        distance = norm(pointB - pointA),
        numPoints = ceil(distance / minDistance),
        step = 1 / numPoints
    ) [
        for (i = [1:numPoints-1])
            pointA + (pointB - pointA) * i * step
    ];

/* Example usage
points = [
    [0, 0, 0],
    [0, 10, 0],
    [5, 10, 0]
];

minDistance = 2;
result = addPoints(points, minDistance);
//result = flatten(addPoints(points, minDistance));
draw_points(result,dia=0.3);
echo(points=points," n=",len(points));
echo(result=result," n=",len(result));
        */

module THING() {
    translate([1,6,0]) cube([2,.2,1],center=false);
}
THING();
/*  // NO LONGER USED     
function compute_iteration(LDB_DEF,Internal_Loads,Failure_Stress,E) =    
// Loop thru all beam elements and compute beam type, then angular deflections
// NOTE: length of loads = length of LDB + 1
    let (n = len(LDB_DEF))
    [ for (i=[0:1:n-1]) 
        let (m1 = Internal_Loads[i][Zm])
        let (m2 = Internal_Loads[i+1][Zm])
        let (reverseBeam = (abs(m2) > abs(m1) ? true : false))
        let (Fx = Internal_Loads[i+1][Zfx])
        let (Fy = Internal_Loads[i+1][Zfy])
        let (LEN = LDB_DEF[i][Zlen])
        let (BT = beam_type(Fy,m2,LEN,reverseBeam))
//echo("BEAM",i=i,Fx=Fx,Fy=Fy,m2=m2,reverseBeam=reverseBeam)
    getBeamAngleFromLoads(LEN=LEN ,THK=LDB_DEF[i][Zthk] ,W=LDB_DEF[i][Zw], Fx=Fx, Fy=Fy, M=m2,BT=BT,Failure_Stress=Failure_Stress, E=E,BEAM_NO = i) ];
   
// Given a Beam Element and Loads, Calculate the deflection
function getBeamAngleFromLoads(LEN=2,THK=0.15,W=0.8,Fx=0,Fy=10,M=0,BT=Zvertforce,Failure_Stress,E, BEAM_NO) = 
    let (Iz = ((W*THK*THK*THK)/12))
    let (AREA = THK*W)
    let (Krotate = spring_rate(BT,Iz,LEN,E))//force per radian, Only used for energy calc
    let (cr = characteristic_radius(BT))
    let (theta = spring_angle(BT,Fy,M,LEN,Krotate,cr)) // degrees. 
    let (t_rad = theta * PI / 180)      // radians
    //let (theta_end = end_angle(BT,theta)) // degrees
    let (theta_end = ThetaEndRoark(Fy,M,LEN,E,Iz)) // new method
    let (Kaxial = E*AREA/LEN)   // Axial spring rate
    let (NEW_LEN = (Fx / Kaxial) + LEN)  // Axial displacement
    let (a = a_position(NEW_LEN,cr,theta))  // non linear displacement
    let (b = b_position(NEW_LEN,cr,theta))  // non linear displacement
    let (bLinear = Fy*NEW_LEN*NEW_LEN*NEW_LEN/(3*E*Iz))  // linear displacement
    let (bRR = (b>0? abs(1-(bLinear/b)): 0 ))  //error between linear and non linear
    let (c = THK / 2)              // half thickness
    let (m_total = M - Fx * b + Fy * a)
    let (stressmax = m_total*c/Iz + Fx/a)
    let (stressmin = -m_total*c/Iz + Fx/a)
    let (ms = getSafetyMargin(stressmax,stressmin,Failure_Stress))
    let (newTHK = checkMS(ms,THK,BEAM_NO))  // GIVES MESSAGE TO GAUGE UP
    let (EnergyRotate = 0.5* Krotate * (t_rad*t_rad))  // PE = 1/2 * K * x ^2
    let (EnergyAxial = 0.5 * Kaxial * (Fx / Kaxial)*(Fx / Kaxial))
    let (energy = EnergyRotate + EnergyAxial)
//echo(BT=BT,cr=cr,theta=theta,theta_end=theta_end,a=a,b=b,bLinear=bLinear, bRR=bRR,ms=ms)
//echo(Krotate=Krotate,Kaxial=Kaxial,LEN=LEN,NEW_LEN=NEW_LEN)
//echo(EnergyRotate=EnergyRotate,EnergyAxial=EnergyAxial,energy=energy)
    // QUALITY CHECKING HERE:
    // if beam type is vertical force and has diverged from linear
    (bRR > 0.15 && BT==Zvertforce ? 
        echo("***** NON LINEAR BEAM - DECREASE ELEMENT LENGTH *****") 
       [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,newTHK,-Fx,-Fy,-m_total] 
       : // no quality problems
       [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,newTHK,-Fx,-Fy,-m_total]);
       
       function spring_angle(bt,fy,moment,L,K,CR) =

// RETURE THE ANGLE GIVEN THE LOAD (DEGREES)
// TWO CASES DEPENDING ON LDB TYPE. 
// FOR DOMINANT MOMENT, ANGLE = M/K
// ELSE ANGLE = FORCE/K*CR*L*COS(ANGLE)
// ANGLE is a function of  COSINE ANGLE.  iterate to decrease the error.
    let (rad_deg = 180/PI)
    let (fm = moment/L)
    let (force = fm + fy)
    let (angle0 = rad_deg*(force/(K)))
    let (angle1 = rad_deg*(force/K)*CR*L*cos(angle0))
    let (angle2 = rad_deg*(force/K)*CR*L*cos(angle1))
    let (angle3 = rad_deg*(force/K)*CR*L*cos(angle2))
    let (check_ang = (abs(angle3) > 30) ? echo("###BEAM ANGLE LIMIT EXCEEDS 30 DEG ###",angle3=angle3) : angle3)
//echo(force=force,fm=fm,fy=fy,bt=bt,angle0=angle0,angle1=angle1,angle2=angle2,angle3=angle3)
    (bt == Zendmoment) ? rad_deg*moment/K : angle3;

// get beam type from forces 
// PE = 1/2 * K * x ^2
// check energy of both?
function beam_type(fy=1,m=.2,L=1,reverseBeam=false) = 
    ((abs(fy*L)>(abs(m)/2) ) ? 
    (reverseBeam ? ZvertfReverse: Zvertforce) : // use A.1.2 vertical force constants
    (reverseBeam ? ZendmReverse: Zendmoment ) ) ; // use A.1.5 moment
    
function characteristic_radius(bt) = 
    (bt == Zvertforce ? 0.85 : 
     (bt == Zendmoment ? 0.7346 : 
       (bt == ZvertfReverse? 1-0.85 : 1-0.7346) ) );
    
function spring_rate(bt,i,len,E) = 
    (bt == Zvertforce ? 2.258*E*i/len : 1.5164*E*i/len);  // NEED TO ACCOUNT FOR REVERSE
    
function end_angle(bt,angle) = 
    (bt == Zvertforce ? angle*1.24 : angle*1.5164);  // NEED TO ACCOUNT FOR REVERSE


*/