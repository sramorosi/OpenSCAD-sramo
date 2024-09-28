// Large Displacement Beam, Modules and Functions
include <NEW_LDB_Indexes.scad>
//use <ME_lib.scad>
use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

// Display intermediate load steps?
Display_steps = true;
// Scale of Force & Moment Display
force_scale = 0.3; // [0.001:.025:10.0]
// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI)
E_PLA_PSI = 320000;  // USED IN MANY FUNCTIONS AND MODULES
//E_PLA_NSMM = 2344;  // Modulus of Elasticity (NEWTONS PER mm^2), PLA
E_PETG_NSMM = 2068;  // Modulus of Elasticity (NEWTONS PER mm^2), PETG

// ~Stress level at which the material will fail
FAILURE_STRESS = 6600;  // PLA, IN PSI
//FAILURE_STRESS_PLA_METRIC = 45;  // ~Stress level at which PLA will fail (NEWTONS per mm^2)
FAILURE_STRESS_PETG_METRIC = 60;  // ~Stress level at which PETG will fail (NEWTONS per mm^2)
// This could be tensile failure, compression failure, bending, etc.

DENSITY_PLA_IMPERIAL = 0.045;  // material density (lb per inch^3)
//DENSITY_PLA_METRIC = 0.0012318;  // material density (gram per mm^3)
DENSITY_PETG_METRIC = 0.0012733;  // material density (gram per mm^3)

//Load_Steps = 6;  // THE NUMBER OF STEPS IS DEPENDENT ON THE PROBLEM

//    // SINGLE BEAM ANALYSIS, TO CHECK THE FUNCTIONS
    N_BEAMS = 6;
    LEN = 3;
    t=0.1;  
    w=0.5;

    // loads on the END are given
    FX2 = 0;
    FY2 = 2;  // 2
    M2 = 0;  // 6, 4  , -7

    // loads on the START are solved
    //FX1 = -FX2;
    //FY1 = -FY2;
    //M1 = FY2*LEN+M2;

            I=((w*pow(t,3))/12);
            //echo(I=I);

            NP=N_BEAMS;
            RoarkPts = [for (i=[1:NP+2]) 
                let (x = ((i-1)/NP)*LEN)
                [x,Y_MidRoark(FY2,M2,LEN,E_PLA_PSI,I,x)]]; 

            color("red") draw_points(RoarkPts,dia=0.03);   
            RoarkAngles = getAnglesFromNodes(RoarkPts,0,0);
                
            NL_Beam_Pts = NL_Beam(NP, LEN, FY2,M2,E_PLA_PSI,I);
            color("blue") draw_points(NL_Beam_Pts,dia=0.035);
            echo(NL_Beam_Pts=NL_Beam_Pts);
        
    pts=[[0,0],[LEN,0]];  // shape
    pts2 = addPoints(pts,LEN/N_BEAMS);  // SUBDIVIDE points
    VIS_BEAM = beamFromNodes(pts2,t,w,false);
    START_BEAM = [[Qbeam,LEN/N_BEAMS,t*4,w,0]];
    END_BEAM = [[Qbeam,LEN/N_BEAMS,t*1,w,0]];
    BEAM = concat(START_BEAM ,concat(VIS_BEAM,END_BEAM));
    //echo(BEAM=BEAM);

    MAKE_BEAM_UNDEFORMED(BEAM,w);
    
    LOADS1 = concat([for (i=[1:N_BEAMS]) [0,0,0]],[[FX2,FY2,M2]]);
    //echo(LOADS1=LOADS1," n=", len(LOADS1));

    DO_ANALYSIS(LDB=BEAM,NODE_LOADS=LOADS1,fscale=force_scale,Display_steps=Display_steps, Failure_Stress=FAILURE_STRESS, E=E_PLA_PSI, density=DENSITY_PLA_IMPERIAL,Origin=[0,0,0],steps=10);
    
function Y_MidRoark(FY=1,M=0,LEN=1,E=1000,I=0.1,X=0.5) = 
// Superposition of two Roark Beam formulas for any mid displacement
// Table 3, p100, case 1a and 3a
    let (a = LEN-X)
    let (y_FY = (FY/(6*E*I)*(2*LEN^3 - 3*LEN^2*a + a^3))) // case 1a
    let (y_M = M*X^2/(2*E*I))  // case 3a
    (y_FY + y_M); // return the sum

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

module DO_ANALYSIS(LDB,NODE_LOADS,fscale,Display_steps,Failure_Stress,E,density,Origin=[0,0,0],steps) {
    echo("******* LARGE DISPLACEMENT 2D BEAM ANALYSIS *******");
    echo(E=E,Failure_Stress=Failure_Stress,density=density,fscale=fscale);
    echo(LDB=LDB);
    echo(NODE_LOADS=NODE_LOADS);
    // perform data checks
    num_beams = count_beams(LDB);
    echo(str("NUMBER OF BEAMS =",num_beams,", NUMBER OF NODES =",len(NODE_LOADS)));
    ttl_load = sum_loads(NODE_LOADS);
    echo(str("TOTAL APPLIED LOADS AND MOMENTS =",ttl_load));
    if (num_beams > 0 && num_beams == len(NODE_LOADS)+1 && abs(ttl_load) > 0) {

        // compute weight
        Weight = computeWeight(LDB,density,START=1,END=len(LDB)-1);
        echo(Weight=Weight);
    
        // Spread Loads. Moments don't include force-moments at this time
        initial_loads = spread_ext_loads(NODE_LOADS);
        //echo(initial_loads=initial_loads);

        // Generate GLOBAL Beam ANGLES, undeformed
        beam_angles = global_angles(LDB);
        //echo("INITIAL ",beam_angles=beam_angles);

        // MAIN ANALYSIS (MODULE)
        COMPUTEStepsModule(f_scale=fscale,LDB=LDB,Failure_Stress=Failure_Stress,E=E,density=density,loads=initial_loads, beam_angles=beam_angles,original_angles=beam_angles,Origin=Origin,,displaySteps=Display_steps,STEPS=steps); 
                
    } else echo("**NUMBER OF BEAMS OR LOADS IS ZERO, TERMINATING**"); 
}

// Recursive Module to perform analysis in (STEPS) steps 
module COMPUTEStepsModule(f_scale=1,LDB,Failure_Stress,E,density,loads,beam_angles,original_angles, Origin, displaySteps=true, STEPS=1,index=0) {
    
    loadScale = (STEPS<=1) ? 1 : (index/(STEPS-1));
    
    loads_scaled = scale_int_loads(loads,loadScale); // scale internal loads
    loads_local = rotate_int_loads(loads_scaled,beam_angles); // Convert internal global forces to beam-local forces 
    force_mom_temp = momentsDueToForce(loads_local, LDB, beam_angles); // Calculate moments due to forces
    beam_moments = sum_moments(force_mom_temp); // Sum moments due to forces
    NEW_loads_local = add_moments_to_loads(loads_local,beam_moments);  // Add moments due-to-forces
    newResults = COMPUTEBeamChain(LDB,NEW_loads_local,Failure_Stress,E);
    Nodes = concat([Origin],GETNodeFromResults(newResults,original_angles, x_start=Origin[0], y_start=Origin[1]));
    newAngleVec = getAnglesFromNodes(Nodes,Origin[0],Origin[1]); // update the beam_angles array
       
    OUTPUT_STRESS(index,loadScale,newResults);
    MIN_MS = min_tree(newResults,Zms);
    
    // Determine Node Display Diameter from the overall model node size
    NODE_DISPLAY_DIA = max(abs(max_tree(Nodes,Nx)),abs(max_tree(Nodes,Ny))) * 0.01;

    if ( index<STEPS-1 ) { // recursion.  index Counts up

        COMPUTEStepsModule(f_scale,LDB,Failure_Stress,E,density,loads,newAngleVec,original_angles, Origin, displaySteps, STEPS, index+1 );
        
        if (displaySteps) { 
            color("yellow",loadScale+.1) 
                draw_points(Nodes,dia=NODE_DISPLAY_DIA); 
            } 
    } if (index==STEPS-1) {  // last iteration
        
        if (displaySteps) { 
            color("red",loadScale) draw_points(Nodes,dia=NODE_DISPLAY_DIA); 
            echo(NODE_DISPLAY_DIA=NODE_DISPLAY_DIA,"FINAL_NODES,",Nodes);
            
            draw_loads(nodes=Nodes, loads=loads_scaled, torques=NEW_loads_local,scale=f_scale);
        }
        translate(Origin) //union () 
            DRAW_DEFORMED_BEAM(LDB,newResults,SUBMS=MIN_MS);
//echo(newResults=newResults);
        echo(str("NODES: X MAX=",max_tree(Nodes,Nx),", X MIN=",min_tree(Nodes,Nx)));
        echo(str("NODES: Y MAX=",max_tree(Nodes,Ny),", Y MIN=",min_tree(Nodes,Ny)));
    }
}
    
function COMPUTEBeamChain(LDB,Internal_Loads,Failure_Stress,E) =    
    // NEW METHOD assumes rigid beams connected by rotational spring (2d)
    // Loop thru all beam elements (first and last are invisible)
    // find average thickness and lengths at nodes (spring) 
    // and get loads, send to function to compute results
    let (n = len(LDB))
    [ for (i=[0:1:n-2]) 
        let (m2 = Internal_Loads[i][Im])
        let (Fx = Internal_Loads[i][Ifx])
        let (Fy = Internal_Loads[i][Ify])
        let (LEN = (LDB[i][Blen]+LDB[i+1][Blen])/2)
        let (THK = (LDB[i][Bthk]+LDB[i+1][Bthk])/2)
        let (W = (LDB[i][Bw]+LDB[i+1][Bw])/2)
//echo("NODE",i=i,LEN=LEN,Fx=Fx,Fy=Fy,m2=m2)
    getSingleSpringResults(LEN=LEN ,THK=THK ,W=W, Fx=Fx, Fy=Fy, M=m2,Failure_Stress=Failure_Stress, E=E,BEAM_NO = i) ];

// Given a Beam Element and Loads, Calculate deflection, stress, ms, energy
function getSingleSpringResults(LEN,THK,W,Fx,Fy,M,Failure_Stress,E, BEAM_NO) = 
    let (c = THK / 2)     // c is half thickness
    let (Iz = ((W*THK^3)/12))
    let (AREA = THK*W)
    let (Krotate = E*Iz/LEN) //moment per radian
    let (m_total = M + Fy * LEN)
    let (stressmax = m_total*c/Iz + Fx/LEN)
    let (stressmin = -m_total*c/Iz + Fx/LEN)
    let (ms = getSafetyMargin(stressmax,stressmin,Failure_Stress))
    let (Kaxial = E*AREA/LEN)   // Axial spring rate
    let (NEW_LEN = (Fx / Kaxial) + LEN)  // Axial displacement
    let (theta = (m_total/Krotate)*180/PI) // degrees
    let (a = NEW_LEN*cos(theta))  // local x displacement
    let (b = NEW_LEN*sin(theta))  // local y displacement
    let (newTHK = checkMS(ms,THK,BEAM_NO))  // GIVES MESSAGE TO GAUGE UP
    let (t_rad = theta * PI / 180)      // radians
    let (EnergyRotate = 0.5* Krotate * (t_rad*t_rad))  // PE = 1/2 * K * x ^2
    let (EnergyAxial = 0.5 * Kaxial * (Fx / Kaxial)*(Fx / Kaxial))
    let (energy = EnergyRotate + EnergyAxial)
//echo(BEAM_NO=BEAM_NO,theta=theta,a=a,b=b,ms=ms)
    // QUALITY CHECKING HERE:
    ((abs(theta) > 60 ) ? 
        echo("***** LARGE SPRING ROTATION *****",BEAM_NO=BEAM_NO) 
    [theta,a,b,ms,stressmin,stressmax,energy,newTHK,-Fx,-Fy,-m_total] 
       :  //no quality problem
    [theta,a,b,ms,stressmin,stressmax,energy,newTHK,-Fx,-Fy,-m_total]);

// Function to get Nodes from Results recursively (NOTE: last 4 parameters are hidden-don't supply)
function GETNodeFromResults(resultsArray,initAngles,x_start=0,y_start=0,ang_start=0, index=0) = 
    index < len(resultsArray) ?
        let (x = resultsArray[index][Za])
        let (y = resultsArray[index][Zb])
        let (ang = resultsArray[index][Ztheta])
        let (initAng = initAngles[index])
        let (sum_ang = initAng + ang + ang_start) // add up to get new angle
        let (x_end = rot_x(x,y,sum_ang) + x_start) // add up to get new x
        let (y_end = rot_y(x,y,sum_ang) + y_start) // add up to get new y 
//echo(index=index,x=x,y=y,y_start=y_start,initAng=initAng,sum_ang=sum_ang)
        concat([ [x_end , y_end ,resultsArray[index][Ztheta]] ],
        // Recursive call to process the next point
        GETNodeFromResults(resultsArray, initAngles, x_end , y_end , sum_ang, index + 1) ) 
    :  [] ;  // Return nothing when all points are processed

module OUTPUT_STRESS(n,loadScale,Results) { 
    echo(str("STEP=",n,",load scale=",loadScale,
    ",σ MAX=",max_tree(Results,Zstressmax),
             ",σ MIN=",min_tree(Results,Zstressmin),
             ",MIN MS=",min_tree(Results,Zms),
             ",MAX MS=",max_tree(Results,Zms),
             ",ENERGY=",sum_fwd(Results,0,Zenergy)));
}
// NEW BEAM MODELER
module MAKE_BEAM_UNDEFORMED(BEAM,THK,idx=0) {
    
    OUTLINE_U = outline_beam_undeformed(BEAM,UP=true);
    OUTLINE_D = outline_beam_undeformed(BEAM,UP=false);
    
    OUTLINE = concat(OUTLINE_U,reverse_array(OUTLINE_D));
    
    linear_extrude(THK,convexity=10,center=true) 
        polygon(OUTLINE);
}

// Create an array of points that outline the beam
// This is called twice, for each side of the beam (UP boolean)
// Assumes the first node is at [x_start,y_start]  of beam angle ang_start
function outline_beam_undeformed(BEAM,UP=true,x_start=0,y_start=0,ang_start=0,index=0) =  
    let (ROT = (UP) ? 90 : -90) // rotation from the beam vector direction
    index < len(BEAM)-1 ? // -1 FOR NEW METHOD ONLY
        index == 0 ? //  first point, first beam
           let (T = (BEAM[index][Bthk] + BEAM[index+1][Bthk])/4)
           concat([ [x_start + T*cos(ang_start+ROT),y_start+ T*sin(ang_start+ROT) ] ],
            outline_beam_undeformed(BEAM,UP,x_start,y_start, BEAM[index][Bang],index+1) )
        :  // middle beams
            let (T = (BEAM[index][Bthk] + BEAM[index+1][Bthk])/4)
            let (LEN = BEAM[index][Blen])
            let (END_ANG = BEAM[index][Bang])
            let (ANG = ang_start)
            let (x_end = x_start + LEN*cos(ANG)) 
            let (y_end = y_start + LEN*sin(ANG))
            concat([ [ x_end + T*cos(ANG+ROT) , y_end + T*sin(ANG+ROT)] ] ,
            outline_beam_undeformed(BEAM,UP,x_end,y_end,END_ANG + ANG,index+1) ): [] ;

// convert the margin of safety into a red to green value
function val_red(i) = i < .5 ? 1 : i > 1 ? 0 : 2-i*2 ;

function val_green(i) = i < 0 ? 0 : i > .5 ? 1 : i*2 ;

// recursive module that draws the deformed beam.
module DRAW_DEFORMED_BEAM(LDB,results,SUBMS=0.0,idx = 1,prior_ang=0) {
    w = LDB[1][Bw];  // W IS CONSTANT
    if(idx < len(LDB)-1) {
        L = LDB[idx][Blen];
        t = LDB[idx][Bthk];
        LDB_ang = LDB[idx][Bang];  // Unloaded ang of beam relative to prior beam
        end_ang = results[idx][Ztheta];  // computed ang at end of beam
        a = results[idx-1][Za];
        b = results[idx-1][Zb];
        ms = results[idx][Zms] - SUBMS; 
//echo(idx=idx,ms=ms,a=a,b=b);
        
        color (c=[val_red(ms),val_green(ms),0],alpha=1) 
            linear_extrude(height=w,center=true) 
            hull() { 
                rotate([0,0,LDB_ang]) translate([a,b,0]) circle(d=t,$fn=16);
                circle(d=t,$fn=16);  // ZERO,ZERO
            }
            
        // Recursive call generating the next beam
        rotate([0,0,LDB_ang]) 
                translate([a,b,0]) 
                    rotate([0,0,end_ang]) 
                        DRAW_DEFORMED_BEAM(LDB,results,SUBMS,idx + 1,LDB_ang+prior_ang);

        }
                  
}

function getAnglesFromNodes(NodesArray,x_start=0,y_start=0, index=1) = 
    index < len(NodesArray) ?
        let (x = NodesArray[index][Nx])
        let (y = NodesArray[index][Ny])
        let (beamAng = atan2(y-y_start,x-x_start))
 //echo(index=index,x=x,x_start=x_start,y=y,y_start=y_start,beamAng=beamAng)
        concat([ beamAng ],
           getAnglesFromNodes(NodesArray, x , y , index + 1) ) 
    :  [] ;  // Return nothing when all points are processed

function beamFromNodes(nodes,t,w,THICKEN_ENDS=false,T_MID=false,TUP = 1.03,S=9,index=0,prior_ang=0) =
    // beam stresses at the fix endS can be larger than reported, due to stress concentrations
    // THICKEN_ENDS option will gradually increase thickenss of the ends
    // TUP is scaler for thickening up the ends
    // S is the number of nodes from each end to thicken
    // More nodes, increase S, decrease TUP
    let (n = len(nodes)-1)
    index < n ? 
    let (T_NEW_1 = index < S ? t*TUP^(S-index) : t) // Thicken Start End
    let (T_NEW_2 = index > n-S ? t*TUP^(index-(n-S)) : T_NEW_1) // Thicken End End
    let (MID_N = floor(n/2))
    let (T_NEW_3 = (index == MID_N && T_MID) ? t*1.4 : T_NEW_2) // Thicken up the middle
    let (T_NEW_4 = (index == (MID_N-1) && T_MID) ? t*1.2 : T_NEW_3) // Thicken up the middle
    let (T_NEW_5 = (index == (MID_N+1) && T_MID) ? t*1.2 : T_NEW_4) // Thicken up the middle
    let (T_NEW = THICKEN_ENDS ? T_NEW_5 : t)
    let (length = sqrt((nodes[index][0]-nodes[index+1][0])^2 + (nodes[index][1]-nodes[index+1][1])^2))
    let (dx=nodes[index+1][0]-nodes[index][0])
    let (dy=nodes[index+1][1]-nodes[index][1])
    let (ang = atan2((nodes[index+1][1]-nodes[index][1]),(nodes[index+1][0]-nodes[index][0])))
//echo(index=index,n=n,T_NEW) //ang=ang,prior_ang=prior_ang,dx=dx,dy=dy)
    concat([[Qbeam,length,T_NEW,w,ang-prior_ang]],beamFromNodes(nodes,t,w,THICKEN_ENDS,T_MID,TUP,S,index+1,ang))  : [] ;

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

// Function to reverse an array of points
function reverse_array(arr) =
    let(len_arr = len(arr)) 
        [for (i = [0 : len_arr - 1]) arr[len_arr - i - 1]] ;
// recursive function to count the number of beams and check data
function count_beams(LDB,i=0,count=0) =
    (i < len(LDB) ? 
    let (length=(LDB[i][Blen] == 0 ? echo("** FOUND ZERO LENGTH BEAM **",i=i) : 0))
    let (thick=(LDB[i][Bthk] == 0 ? echo("** FOUND ZERO THICKNESS BEAM **",i=i):0))
    let (width=(LDB[i][Bw] == 0 ? echo("** FOUND ZERO WIDTH BEAM **",i=i):0))
    //echo("BEAM",i=i,count=count)
        count_beams(LDB,i+1,count+1) 
    : count );

// recursive function to sum the loads, for check data
function sum_loads(LOADS,i=0,running_sum=0) =
    (i < len(LOADS) ?
    let (Fx=LOADS[i][Ifx])
    let (Fy=LOADS[i][Ify])
    let (m=LOADS[i][Im])
    let (sum=(Fx+Fy+m)+running_sum)
    sum_loads(LOADS,i+1,sum)
    : running_sum ); 
        
module draw_points(pts,dia=0.1) {
    numPts = len(pts);
    for (i=[0:numPts-1]) translate([pts[i][0],pts[i][1],1]) color("black") circle(dia,$fn=8);
}

function computeWeight(LDB,density,START=0,END=1) = 
    // Loop thru all beam elements and sum weight off each beam
    let (LEN = LDB[START][Blen])
    let (THK=LDB[START][Bthk])
    let (W=LDB[START][Bw])
    let (AREA = THK*W)
    let (WEIGHT = AREA*LEN*density)
//echo("BEAM",START=START,density=density,LEN=LEN,AREA=AREA,WEIGHT=WEIGHT)
    (START>=END-1 ? WEIGHT : WEIGHT + computeWeight(LDB,density,START+1,END)); 

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
    [sum_fwd(ext_loads,n-1,Ifx,i), sum_fwd(ext_loads,n-1,Ify,i), sum_fwd(ext_loads,n-1,Im,i)]];

// recursive function to rotate the internal forces from a global system to a beam-local system.  Moments are copied
function rotate_int_loads(int_loads,beam_angles) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) (
        let (fx = int_loads[i][Ifx])
        let (fy = int_loads[i][Ify])
        let (ang = (i==0? beam_angles[i] :-beam_angles[i-1])) // special Ground case
//echo(i=i,fx=fx,f=fy,ang=ang)
        [rot_x(fx,fy,ang) , rot_y(fx,fy,ang) ,int_loads[i][Im] ])];

// recursive function to scale the internal forces and moments
function scale_int_loads(int_loads,scale=1) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) 
        let (fx = int_loads[i][Ifx] * scale)
        let (fy = int_loads[i][Ify] * scale)
        let (moment = int_loads[i][Im] * scale)
        [fx,fy,moment ]  ];

// calculate moment due to force on current beam 
function momentsDueToForce(loads, LDBdef, angles) = 
    let (n = len(LDBdef))
    [ for (i=[1:1:n-2]) (  // OLD METHOD WAS 0:1:n-1
                let (L = LDBdef[i][Blen])
                let (fy = loads[i][Ify])  // OLD METHOD WAS loads[i+1]
//echo("MO DUE TO FORCE",i,L=L,fy=fy)
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
        let (moment = node_loads[i][Im])
        let (moment2 = (i==n-1? 0 : beam_moments[i]))
        [node_loads[i][Ifx],node_loads[i][Ify],moment+moment2] ];
    
function rot_x (x,y,a) = x*cos(a)-y*sin(a);

function rot_y (x,y,a) = x*sin(a)+y*cos(a);
    
// recursive forward summation function to sum "thing"
// from the start (or start element) to the i'th element 
function sum_fwd(ARRAY,i,thing,start=0) = 
    let (val = ARRAY[i][thing])
    (i==start ? val : val + sum_fwd(ARRAY,i-1,thing,start));

// sum angles along segments to get global angles.
function global_angles(LDBdef,prior_ang=0) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) LDBdef[i][Bang] ] ;

function sum_range(ARRAY,start=0,end=1) = 
    let (val = ARRAY[start])
    (start>=end ? val : val + sum_range(ARRAY,start+1,end));
    
function getSafetyMargin(stressMax,stressMin,FailureStress) = 
    (abs(stressMax) > abs(stressMin)) ? 
        FailureStress/abs(stressMax)-1 
        : FailureStress/abs(stressMin)-1 ;

function checkMS(ms,thk,BEAMNO) = 
    ms < 0.0 ?
    let (newTHK = thk*(1-ms))
    echo(BEAMNO=BEAMNO,"##### NEGATIVE MARGIN OF SAFETY, GAUGE UP!! ###### NEW THK=",newTHK) newTHK 
    : thk ;  // return current thickness is MS is positive

// module that draws the loads 
module draw_loads(nodes,loads,torques,scale=1,z_offset=20) {
    //  z_offset is used to move the force/moment objects above the beam for vis
    N=len(loads)-1;
    for (i=[0:N]) {
        fx = loads[i][Ifx];
        fy = loads[i][Ify];
        moment = torques[i][Im];
        fmag = sqrt(fx*fx + fy*fy);
//echo("DRAW_LOADS,",i=i,fx=fx,fy=fy,moment=moment);
        // draw forces and torques
        if (abs(fmag)>0.1) color ("red") 
            translate([nodes[i][0],nodes[i][1],z_offset]) 
                force_arrow([0,0,0],[fx,fy,0],mag=fmag*scale);
        if (abs(moment)>0.1) color ("blue")
            translate([nodes[i][0],nodes[i][1],z_offset]) 
                torque_arrow([0,0,0],mag=moment*scale);
    }
}
