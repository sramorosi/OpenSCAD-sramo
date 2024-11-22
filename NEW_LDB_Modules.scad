// Large Displacement Beam, Modules and Functions
include <NEW_LDB_Indexes.scad>
//use <ME_lib.scad>
use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

// TO DO LIST
// Go through test cases
// FIX LOADS DISPLAY PROBLEM (SHOWING LOCAL LOADS)
// HAVE BUG WITH STARTING ANGLE, SEE CASE 4
// ADD SUM MOMENT CHECK AFTER COMPUTATION
// Reduce number of arrays in a step by:
//     COMPUTEBeamChain  could also calculate and store node absolute x,y, angle
// 
// getAnglesFromNodes is not being used. Seems like an update step is missing. 
// Better module and function names  (FIX_RIGHT??)  Naming convention?

// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI)
E_PLA_PSI = 320000;  // USED IN MANY FUNCTIONS AND MODULES
E_PLA_NSMM = 2344;  // Modulus of Elasticity (NEWTONS PER mm^2), PLA
E_PETG_NSMM = 2068;  // Modulus of Elasticity (NEWTONS PER mm^2), PETG

// ~Stress level at which the material will fail
FAILURE_STRESS_PLA_PSI = 6600;  // PLA, IN PSI
FAILURE_STRESS_PLA_METRIC = 45;  //~Stress level when PLA will fail (NEWTONS per mm^2)
FAILURE_STRESS_PETG_METRIC = 60; //~Stress level when PETG will fail (NEWTONS per mm^2)
// This could be tensile failure, compression failure, bending, etc.

DENSITY_PLA_IMPERIAL = 0.045;  // material density (lb per inch^3)
DENSITY_PLA_METRIC = 0.0012318;  // material density (gram per mm^3)
DENSITY_PETG_METRIC = 0.0012733;  // material density (gram per mm^3)

/*
// Catilever beam, left fixed, right free, end load
// Roark 6th ed, Table 3, case 1a and 3a, page 100
    N_BEAMS = 6;
    LEN = 3;
    t=0.1;  
    w=0.5;

    // End Load can have Fx, Fy and M
    FX2 = 0;
    FY2 = 2;  // 2
    M2 = 0;  // 6, 4  , -7

        // Solution from Roark, case 1a and 3a
        I=((w*pow(t,3))/12);
        //echo(I=I);

        NP=N_BEAMS;
        RoarkPts = [for (i=[1:NP+2]) 
            let (x = ((i-1)/NP)*LEN)
            [x,Y_MidRoark(FY2,M2,LEN,E_PLA_PSI,I,x)]]; 

        // Linear solution from Roark (superposition of case 1a and 3a)
        color("red") draw_points(RoarkPts,dia=0.03);   
        RoarkAngles = getAnglesFromNodes(RoarkPts,0,0);

        // Non-linear solution where beam length is preserved
        NL_Beam_Pts = NL_Beam(NP, LEN, FY2,M2,E_PLA_PSI,I);
        color("blue") draw_points(NL_Beam_Pts,dia=0.035);
        //echo(NL_Beam_Pts=NL_Beam_Pts);
        
    pts=[[0,0],[LEN,0]];  // shape
    pts2 = addPoints(pts,LEN/N_BEAMS);  // SUBDIVIDE points
    VIS_BEAM = beamFromNodes(pts2,t,w,false);
    START_BEAM = [[Qstart,LEN/N_BEAMS,t*4,w,0]];
    END_BEAM = [[Qend,LEN/N_BEAMS,t*1,w,0]];
    BEAM = concat(START_BEAM ,concat(VIS_BEAM,END_BEAM));
    //echo(BEAM=BEAM);

    MAKE_BEAM_UNDEFORMED(BEAM,w);
    
    LOADS1 = concat([for (i=[1:N_BEAMS]) [0,0,0]],[[FX2,FY2,M2]]);
    
    DO_ANALYSIS(LDB=BEAM,EXT_LOADS=LOADS1,fscale=0.2,Display_steps=true, echoLDB=false, displayLoads=false, Failure_Stress=FAILURE_STRESS_PLA_PSI, E=E_PLA_PSI, density=DENSITY_PLA_IMPERIAL,Origin=[0,0,0],steps=3);
*/
//
// Catilever beam, left fixed, right simple support, mid load
// Roark 6th ed, Table 3, case 1c, page 100
    N_BEAMS = 8;
    LEN = 4;
    t=0.1;  
    w=0.5;

    // loads
    W = 2;  // 2

    I=((w*pow(t,3))/12);
    //echo(I=I);

    // Solution from Roark
    a = LEN/2;
    FYr = -(W/(2*LEN^3))*((LEN-a)^2)*(2*LEN+a); // case 1c, fixed left, simple right
    //FYr = -(W/LEN)*(LEN-a);  // case 1e, simple support on both ends
    FYla = -W-FYr;  // from sum of forces in Y
    FYlroark = -((W*a)/(2*LEN^3))*(3*LEN^2 - a^2); // case 1c, fixed left, simple right
    Ml = -(W*a)/(2*LEN^2)*(LEN^2-a^2);// case 1c, fixed left, simple right
    echo(FYla=FYla,FYlroark=FYlroark,Ml=Ml,FYr=FYr);
    
    pts = addPoints([[0,0,0],[LEN,0,0]],LEN/N_BEAMS);  // SUBDIVIDE points
    VIS_BEAM = beamFromNodes(pts,t,w,false);
    START_BEAM = [[Qstart,LEN/N_BEAMS,t*1,w,0]]; // LEFT FIXITY
    END_BEAM = [[Qend,LEN/N_BEAMS,t*1,w,0]];
    BEAM = concat(START_BEAM ,concat(VIS_BEAM,END_BEAM));

    //MAKE_BEAM_UNDEFORMED(BEAM,w);
    
    MID=N_BEAMS/2;
    LOADS1 = concat([for (i=[1:MID]) [0,0,0]],
        concat([[0,W,0]],
            concat([for (i=[1:MID-1]) [0,0,0]],[[0,0,0]])));//[[0,FYr,0]])));

    DO_ANALYSIS(LDB=BEAM,EXT_LOADS=LOADS1,fscale=0.3,Display_steps=true, echoLDB=false, displayLoads=true, Failure_Stress=FAILURE_STRESS_PLA_PSI, E=E_PLA_PSI, density=DENSITY_PLA_IMPERIAL,pinned=true,FixFy=true,steps=6);
//
module DO_ANALYSIS(LDB,EXT_LOADS,fscale,Display_steps=false, echoLDB=false, displayLoads=false, Failure_Stress,E,density,pinned=false,FixFy=true,steps) {
    echo("******* LARGE DISPLACEMENT 2D BEAM ANALYSIS *******");
    echo(E=E,Failure_Stress=Failure_Stress,density=density,fscale=fscale);
    if(echoLDB) echo(LDB=LDB);
    if(echoLDB) echo(EXT_LOADS=EXT_LOADS);
    // perform data checks
    num_beams = count_beams(LDB);
    echo(str("NUMBER OF BEAMS =",num_beams,", NUMBER OF NODES =",len(EXT_LOADS)));
    if (num_beams > 0 && num_beams == len(EXT_LOADS)-1 ) {

        // compute weight
        Weight = computeWeight(LDB,density,START=1,END=len(LDB)-1);
        echo(Weight=Weight);
    
        // Generate GLOBAL Beam ANGLES, undeformed
        NoLoadAngles = global_angles(LDB);
        //echo("INITIAL ",NoLoadAngles=NoLoadAngles);
        
        // DRAW THE UNDEFORMED APPLIED LOADS
        Nodes = getNodesFromLDB(LDB, num_beams);
        draw_loads(nodes=Nodes, loads=EXT_LOADS, torques=EXT_LOADS,scale=fscale);

        // MAIN ANALYSIS (MODULE)
        ComputeStepsModule(f_scale=fscale,LDB=LDB,Failure_Stress=Failure_Stress,E=E,density=density,loads=EXT_LOADS, original_angles=NoLoadAngles,new_angles=NoLoadAngles,displaySteps=Display_steps,displayLoads=displayLoads, pinned = pinned, FixFy=FixFy, NB = num_beams, STEPS=steps); 
                
    } else echo("**NUMBER OF BEAMS OR LOADS ERROR, TERMINATING**"); 
}

// Recursive Module to perform analysis in steps 
module ComputeStepsModule(f_scale=1,LDB, Failure_Stress,E,density,loads,original_angles, new_angles, displaySteps=true, displayLoads=false, pinned = false, FixFy=true, NB=0, STEPS=1,index=1) {
    
    loadScale = (STEPS<=1) ? 1 : (index/(STEPS-1));

    newResults = (!pinned) ? 
        // OLD WAY, RIGHT END HAS NO SUPPORT
        COMPUTEBeamChain(LDB,
        makeInternalLoads(EXT_LOADS=loads,LSCALE=loadScale,BEAM_ANGLES=new_angles,LDB=LDB), Failure_Stress,E) 
    :     // NEW WAY, RIGHT END SIMPLY SUPPORTED
     FIX_RIGHT(EXT_LOADS=loads,LSCALE=loadScale,BEAM_ANGLES=new_angles,INIT_ANGLES=original_angles,LDB=LDB,E=E,SFAIL=Failure_Stress,FixFy=FixFy);
    
    NEW_EXT_LOADS = (!pinned) ?
        // OLD WAY, EXTERNAL LOADS DO NOT CHANGE
        loads
        : // NEW WAY, RIGHT END LOADS CHANGE EACH STEP
        SWAP_LAST(loads,newResults[NB+1]);
    
    //Nodes = GETNodeFromResults(resultsArray=newResults,initAngles=original_angles, x_start=0, y_start=0,ang_start=0, NumNodes = NB+1);
    Nodes = concat([[0,0,0]],GETNodeFromResults(resultsArray=newResults,initAngles=original_angles, x_start=0, y_start=0,ang_start=0, NumNodes = NB+1));

    // write information to the Error Log window
    LOG_STRESS(n=index,loadScale=loadScale,Results=newResults, NODES=Nodes,LOADS=NEW_EXT_LOADS);
    
    // Determine Node Display Diameter from the overall model node size
    NODE_DISPLAY_DIA = max(abs(max_tree(Nodes,Nx)),abs(max_tree(Nodes,Ny))) * 0.01;

    if ( index<STEPS-1 ) { // Intermediate step.  index Counts up
        if (displaySteps) {  
            color("yellow",loadScale+.1) 
                draw_points(Nodes,dia=NODE_DISPLAY_DIA); 
        } 
        // GET NEW BEAM ANGLES BEFORE NEXT STEP
        new_angles = getAnglesFromNodes(Nodes,x_start=0,y_start=0);
        //echo(new_angles=new_angles);

        // RECURSION IS HERE, NEXT STEP
        ComputeStepsModule(f_scale=f_scale,LDB=LDB,Failure_Stress=Failure_Stress, E=E, density=density, loads=NEW_EXT_LOADS,original_angles=original_angles, new_angles=new_angles, displaySteps = displaySteps, displayLoads=displayLoads, pinned=pinned, FixFy=FixFy, NB=NB, STEPS=STEPS, index=index+1 );
        
    } if (index==STEPS-1) {  // last step, display extra stuff
        //echo(NEW_EXT_LOADS=NEW_EXT_LOADS);        
        if (displaySteps) { 
            color("red",loadScale) 
                draw_points(Nodes,dia=NODE_DISPLAY_DIA); 
        }
        if (displayLoads) {
            // NEW WAY TO GET FINAL LOADS
            Final_Int_Loads = 
            makeInternalLoads(NEW_EXT_LOADS,loadScale,new_angles,LDB);
            fixedLoads = 
            [[-Final_Int_Loads[0][Ifx],-Final_Int_Loads[0][Ify],-Final_Int_Loads[0][Im]]];
            draw_loads(nodes=Nodes, loads=fixedLoads, torques=fixedLoads,scale=f_scale);
            echo(str("Final Starting Fixed Loads [Fx,Fy,M]=",fixedLoads));

            draw_loads(nodes=Nodes, loads=NEW_EXT_LOADS, torques=NEW_EXT_LOADS,scale=f_scale);
        //
        } 
        // Find the minimum Margin so the colors look good
        //MIN_MS = min_tree(newResults,Zms); 
        MIN_MS = 0.0;
        //DRAW_DEFORMED_BEAM(LDB=LDB,results=newResults,prior_ang = newResults[0][Ztheta],SUBMS=MIN_MS );
        DRAW_DEFORMED_BEAM(LDB=LDB,results=newResults,nodes=Nodes, SUBMS=MIN_MS );
        
        // write min max node location to Error Log window
        nodesM1 = RMV_LAST(Nodes);
        echo(str("NODES: X MAX=",max_tree(nodesM1,Nx),", X MIN=",min_tree(nodesM1,Nx)),
        str("NODES: Y MAX=",max_tree(nodesM1,Ny),", Y MIN=",min_tree(nodesM1,Ny)));
    }
}

// FUNCTION TO MAKE SCALED INTERNAL LOADS FROM EXTERNAL LOADS
function makeInternalLoads(EXT_LOADS,LSCALE,BEAM_ANGLES,LDB) = 
    // scale external loads
    let (loads_scaled = Scale_Loads(EXT_LOADS,LSCALE)) 
    // Spread Loads. Moments don't include force-moments at this time
    let (spread_loads = spread_ext_loads(loads_scaled))
    //echo(spread_loads=spread_loads)
    // Convert internal global forces to beam-local forces 
    let (rotated_forces = rotate_int_loads(spread_loads,BEAM_ANGLES)) 
    //echo(rotated_forces=rotated_forces)
    // Calculate moments due to forces
    let (force_mom_temp = momentsDueToForce(rotated_forces, LDB, BEAM_ANGLES)) 
    // Sum moments due to forces
    let (beam_moments = sum_moments(force_mom_temp)) 
    // Add moments due-to-forces and return
    add_moments_to_loads(rotated_forces,beam_moments) ;

// Function to keep right end pinned in Y direction
// Returns Results with Last member = pinned end Fy 
//      APPLIED LOAD at FULL SCALE
function FIX_RIGHT(EXT_LOADS,LSCALE,BEAM_ANGLES,INIT_ANGLES,LDB,E,SFAIL, FixFy=true) =  // If FixFY=false then its Fix Moment
    // LOADS are scaled by LSCALE

    let (n = len(LDB)-2) // last position in Nodes array

    let (Fy = EXT_LOADS[n][Ify]) // this is the initial guess
    let (M = EXT_LOADS[n][Im]) 

    // LOAD 1
    let (Fy_1 = Fy==0 ? 0.1 : Fy*1.2) // small adjustment greater
    let (M_1 = M==0 ? 0.1 : M*1.2)
    let (new_last_1 = FixFy ? [EXT_LOADS[n][Ifx],Fy_1,EXT_LOADS[n][Im]] 
                        : [EXT_LOADS[n][Ifx],EXT_LOADS[n][Ify],M_1])
    //let (new_last_1 = [EXT_LOADS[n][Ifx],Fy_1,M_1])
    //let (new_last_1 = [EXT_LOADS[n][Ifx],EXT_LOADS[n][Ify],M_1])
    let (Ext_Loads_1 = SWAP_LAST(ARRAY=EXT_LOADS,NEW_LAST = new_last_1))
    let (Int_Loads_1 = makeInternalLoads(Ext_Loads_1,LSCALE,BEAM_ANGLES,LDB))
    let (newResults_1 = COMPUTEBeamChain(LDB=LDB,Internal_Loads=Int_Loads_1,Failure_Stress=SFAIL,E=E,checks=false))
    let (Nodes_1 = GETNodeFromResults(newResults_1,INIT_ANGLES,0,0,0,n+1))
    let (dy_1 = Nodes_1[n-1][Ny])  // assuming target is Y=0 for now
    let (da_1 = atan2(Nodes_1[n][Ny]-Nodes_1[n-1][Ny], Nodes_1[n][Nx]-Nodes_1[n-1][Nx]))

    // LOAD 2
    let (Fy_2 = Fy==0 ? -0.1 : Fy_1*0.8) // small adjustment less
    let (M_2 = M==0 ? -0.1 : M*0.8)
    let (new_last_2 = FixFy ? [EXT_LOADS[n][Ifx],Fy_2,EXT_LOADS[n][Im]]
               : [EXT_LOADS[n][Ifx],EXT_LOADS[n][Ify],M_2])
    //let (new_last_2 = [EXT_LOADS[n][Ifx],EXT_LOADS[n][Ify],M_2])
    //let (new_last_2 = [EXT_LOADS[n][Ifx],Fy_2,M_2])
    let (Ext_Loads_2 = SWAP_LAST(ARRAY=EXT_LOADS,NEW_LAST = new_last_2))
    //echo(Ext_Loads_2=Ext_Loads_2)
    let (Int_Loads_2 = makeInternalLoads(Ext_Loads_2,LSCALE,BEAM_ANGLES,LDB))
    let (newResults_2 = COMPUTEBeamChain(LDB=LDB, Internal_Loads=Int_Loads_2,Failure_Stress=SFAIL,E=E,checks=false))
    let (Nodes_2 = GETNodeFromResults(newResults_2,INIT_ANGLES,0,0,0,n+1))
    let (dy_2 = Nodes_2[n-1][Ny])  // assuming target is Y=0 for now
    let (da_2 = atan2(Nodes_2[n][Ny]-Nodes_2[n-1][Ny], Nodes_2[n][Nx]-Nodes_2[n-1][Nx]))

    // calculate new Fy where line between points intecepts Y
    let (my=(Fy_2-Fy_1)/(dy_2-dy_1)) // m = slope
    let (by=Fy_1-my*dy_1)  // b is the Y intersept
    let (NEW_FY = by)

    // calculate new M where line between points intecepts Y
    let (mM=(M_2-M_1)/(da_2-da_1)) // m = slope
    let (bM=M_1-mM*da_1)  // b is the Y intersept
    let (NEW_M = bM)

    // last computation with NEW_FY
    let (final_last = FixFy ? [EXT_LOADS[n][Ifx],NEW_FY,EXT_LOADS[n][Im]] 
                : [EXT_LOADS[n][Ifx],EXT_LOADS[n][Ify],NEW_M])
    //let (final_last = [EXT_LOADS[n][Ifx],NEW_FY,NEW_M])
    //let (final_last = [EXT_LOADS[n][Ifx],EXT_LOADS[n][Ify],NEW_M])
    //echo(final_last=final_last)
    let (Final_Ext_Loads = SWAP_LAST(ARRAY=EXT_LOADS,NEW_LAST = final_last))
    let (Final_Int_Loads = makeInternalLoads(Final_Ext_Loads,LSCALE,BEAM_ANGLES,LDB))
    let (FinalResults = COMPUTEBeamChain(LDB=LDB, Internal_Loads=Final_Int_Loads, Failure_Stress= SFAIL,E=E,checks=true))
    let (finalNodes = GETNodeFromResults(FinalResults,INIT_ANGLES,0,0,0,n+1))
    let (DY_final = finalNodes[n-1][Ny])  // assuming target is Y=0 for now
    let (da_final = atan2(finalNodes[n][Ny]-finalNodes[n-1][Ny], finalNodes[n][Nx]-finalNodes[n-1][Nx]))
    echo("^^^FIX FY:",FixFy,Fy_1=Fy_1,dy_1=dy_1,Fy_2=Fy_2,dy_2=dy_2,NEW_FY=NEW_FY,DY_final=DY_final)
    echo("^^^FIX M:",!FixFy,M_1=M_1,da_1=da_1,M_2=M_2,da_2=da_2,NEW_M=NEW_M,da_final=da_final)
    // Return the results, and append on the new right loads.
           concat(FinalResults,[final_last]) ;
    
function COMPUTEBeamChain(LDB,Internal_Loads,Failure_Stress,E,checks) =    
    // NEW METHOD assumes rigid beams connected by rotational spring (2d)
    // Loop thru all beam elements (first and last are invisible)
    // find average thickness and lengths at nodes (spring) 
    // and get loads, send to function to compute results
    let (n = len(LDB))
    [ for (i=[0:1:n-2]) 
        let (m2 = Internal_Loads[i][Im])
        let (Fx = Internal_Loads[i][Ifx])
        let (Fy = Internal_Loads[i][Ify])
        let (LEN = (LDB[i][Blen]+LDB[i+1][Blen])/2) // spring calculation
        let (THK = (LDB[i][Bthk]+LDB[i+1][Bthk])/2)
        let (W = (LDB[i][Bw]+LDB[i+1][Bw])/2)
    //echo("NODE",i=i,LEN=LEN,Fx=Fx,Fy=Fy,m2=m2)
    getSingleSpringResults(LEN=LDB[i+1][Blen],K_LEN=LEN ,THK=THK ,W=W, Fx=Fx, Fy=Fy, M=m2,Failure_Stress=Failure_Stress, E=E,BEAM_NO = i,checks=checks) ];

// Given a Beam Element and Loads, Calculate deflection, stress, ms, energy
function getSingleSpringResults(LEN,K_LEN,THK,W,Fx,Fy,M,Failure_Stress,E, BEAM_NO,checks) = 
    let (c = THK / 2)     // c is half thickness
    let (Iz = ((W*THK^3)/12))
    let (AREA = THK*W)
    let (Krotate = E*Iz/K_LEN) //moment per radian
    let (Kaxial = E*AREA/LEN)   // Axial spring rate
    let (NEW_LEN = (Fx / Kaxial) + LEN)  // Axial displacement
    let (m_1 = M + Fy * NEW_LEN)  // moment without Fx component
    let (theta_1 = (m_1/Krotate)*180/PI) // rotation, degrees, without Fx
    let (a_1 = NEW_LEN*cos(theta_1))  // local x displacement, minus Fx component
    let (b_1 = NEW_LEN*sin(theta_1))  // local y displacement, minus Fx component
    let (m_total = M - Fx * b_1 + Fy * a_1) // update moment to include Fx
    let (theta = (m_total/Krotate)*180/PI) // update rotation with new moment
    let (a = NEW_LEN*cos(theta))  // local x displacement
    let (b = NEW_LEN*sin(theta))  // local y displacement
    let (stressmax = m_total*c/Iz + Fx/LEN)
    let (stressmin = -m_total*c/Iz + Fx/LEN)
    let (ms = getSafetyMargin(stressmax,stressmin,Failure_Stress))
    let (newTHK = checks ? checkMS(ms,THK,BEAM_NO) : THK)  // GIVES MESSAGE TO GAUGE UP
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
function GETNodeFromResults(resultsArray,initAngles,x_start=0,y_start=0,ang_start=0, NumNodes = 1, index=0) = 
    index < NumNodes ?
        let (x = resultsArray[index][Za])
        let (y = resultsArray[index][Zb])
        //let (ang = index==0 ? 0 : resultsArray[index-1][Ztheta])
        let (ang = resultsArray[index][Ztheta])
        let (initAng = initAngles[index]) // undeformed angles
        let (sum_ang = initAng + ang + ang_start) // add up to get new angle
        let (x_end = index==0 ? rot_x(x,y,initAng) : rot_x(x,y,sum_ang) + x_start) // add up to get new x
        let (y_end = index==0 ? rot_y(x,y,initAng) : rot_y(x,y,sum_ang) + y_start) // add up to get new y 
//echo(index=index,x=x,y=y,ang=ang,sum_ang=sum_ang,ang_start=ang_start)
        concat([ [x_end , y_end ,resultsArray[index][Ztheta]] ],
        // Recursive call to process the next point
        GETNodeFromResults(resultsArray=resultsArray, initAngles=initAngles, x_start=x_end , y_start=y_end , ang_start=sum_ang, NumNodes=NumNodes, index=index + 1) ) 
    :  [] ;  // Return nothing when all points are processed

function getNodesFromLDB(LDB, NB, x_last=0,y_last=0,ang_last=0, index=0) = 
    index < NB ?
        let (len = LDB[index+1][Blen])  // index+1 because first beam is not for display
        let (ang = LDB[index][Bang])
        let (sum_ang = ang + ang_last) // add up to get new angle
        let (x = len*cos(sum_ang))
        let (y = len*sin(sum_ang))
        let (x_end = x + x_last) // add up to get new x
        let (y_end = y + y_last) // add up to get new y 

//echo(index=index,ang_last=ang_last,ang=ang,sum_ang=sum_ang)
        (index==0) ? concat([ [x_last,y_last],[x,y]  ], getNodesFromLDB(LDB, NB, x_end,y_end, sum_ang, index +1 ))
            : concat([ [ x_end , y_end ] ], getNodesFromLDB(LDB, NB, x_end,y_end, sum_ang, index + 1)) 
            : [] ; // return nothing when index is up

module LOG_STRESS(n,loadScale,Results,NODES,LOADS) { 
    echo(str(">>>>STEP=",n+1,",load scale=",loadScale,
    ",σ MAX=",max_tree(Results,Zstressmax),
             ",σ MIN=",min_tree(Results,Zstressmin),
             ",MIN MS=",min_tree(Results,Zms),
             ",MAX MS=",max_tree(Results,Zms),
             ",ENERGY=",sum_fwd(Results,0,Zenergy)));
}
/* test of beam modeler
TESTpts=[[0,0],[0,3],[3,3],[3,0]]; 
pts_ADD = addPoints(TESTpts,1.0);  // add points at 0.5 spacing
draw_points(pts_ADD,dia=0.05);
BEAM1 = BEAM_FROM_NODES(nodes=pts_ADD,TBEAMS=1,TENDS=.5,w=.3);
echo(BEAM1=BEAM1);
MAKE_BEAM_UNDEFORMED(BEAM1,THK=1);
Nodes = getNodesFromLDB(BEAM1, len(BEAM1)-1);
color("yellow") draw_points(Nodes,dia=.1); 
*/

// UNDEFORMED BEAM MODELER
// Creates an up and down profile and concatinate into an outline.
module MAKE_BEAM_UNDEFORMED(BEAM,THK,idx=0) {
    
    start_ang = BEAM[0][Bang]; // starting angle of the beam is in beam[0]
    OUTLINE_U = outline_beam_undeformed(BEAM=BEAM,UP=true,ang_start=start_ang);
    OUTLINE_D = outline_beam_undeformed(BEAM=BEAM,UP=false,ang_start=start_ang);
    
    OUTLINE = concat(OUTLINE_U,reverse_array(OUTLINE_D));
    
    linear_extrude(THK,convexity=10,center=true) 
        polygon(OUTLINE);
}

// Create a point-array profile on one side of the beam.
// Call it twice, for each side of the beam (UP boolean)
// Thickness at a node is the average of the beam to either side.
// Parameters x1,y1,index are for recursion.
function outline_beam_undeformed(BEAM,UP=true,ang_start=0,x1=0,y1=0,index=0) =  
    let (ROT = (UP) ? 90 : -90) // rotation from the beam vector direction
    index < len(BEAM)-1 ? // -1 FOR NEW METHOD ONLY
        index == 0 ? //  first point, first beam
           let (T = (BEAM[index][Bthk] + BEAM[index+1][Bthk])/4)
           concat([ [x1 + T*cos(ang_start+ROT),y1+ T*sin(ang_start+ROT) ] ],
            outline_beam_undeformed(BEAM,UP, BEAM[index][Bang],x1,y1,index+1) )
        :  // middle beams
            let (T = (BEAM[index][Bthk] + BEAM[index+1][Bthk])/4)
            let (LEN = BEAM[index][Blen])
            let (END_ANG = BEAM[index][Bang])
            let (ANG = ang_start)
            let (AVG_ANG = ANG+END_ANG/2)
//echo(index=index,ANG=ANG,END_ANG=END_ANG,AVG_ANG=AVG_ANG)
            let (x_end = x1 + LEN*cos(ANG)) 
            let (y_end = y1 + LEN*sin(ANG))
            let (TMOD = T/cos(END_ANG/2))
            concat([ [ x_end + TMOD*cos(AVG_ANG+ROT) , y_end + TMOD*sin(AVG_ANG+ROT)] ] ,
            outline_beam_undeformed(BEAM,UP,END_ANG + ANG,x_end,y_end,index+1) ): [] ;

// convert the margin of safety into a red to green value
function val_red(i) = i < .5 ? 1 : i > 1 ? 0 : 2-i*2 ;

function val_green(i) = i < 0 ? 0 : i > .5 ? 1 : i*2 ;

// Recursive module that draws the deformed beam
// Each beam is colored to show MS level (red=low, green=high)
// Parameter idx is used for recursion.
module DRAW_DEFORMED_BEAM(LDB,results,nodes,SUBMS=0.0,x0=0, y0=0, idx = 0) {
    w = LDB[1][Bw];  // W IS CONSTANT
    if(idx < len(LDB)-2) {
        t = LDB[idx+1][Bthk];
        a = nodes[idx+1][Nx];
        b = nodes[idx+1][Ny];
        ms_prior = idx==0 ? 0 : results[idx-1][Zms];
        //ms = (ms_prior+results[idx][Zms])/2 - SUBMS; 
        ms = results[idx][Zms] - SUBMS; 
        RGB = (ms <= 0.0 ) ? [0,0,0] : [val_red(ms),val_green(ms),0];
//echo(idx=idx,x0=x0,y0=y0,a=a,b=b);
        
        // Draw a round-end beam, using hull
        color (c=RGB,alpha=1) 
            linear_extrude(height=w,center=true) 
            hull() { 
                translate([x0,y0,0]) 
                    circle(d=t,$fn=16);
                translate([a,b,0])
                    circle(d=t,$fn=16);
            }
            
        // Recursive call generating the next beam
        DRAW_DEFORMED_BEAM(LDB=LDB,results=results, nodes=nodes, SUBMS=SUBMS, x0=a,y0=b,idx=idx + 1);
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

// Test of MAKE_BEAM_UNDEFORMED
nb=49;
L=10;
pts2=[for (i=[0:nb]) [L*(i/nb),0]];
//echo(pts2=pts2);
// Tests here
bt = BEAM_FROM_NODES(nodes=pts2,TBEAMS=1.2,TENDS=1.7,w=1,THICKEN_ENDS=true,T_MID=true,TUP=1.03,S=10,index=0, prior_ang=0);
//echo(bt=bt);
//MAKE_BEAM_UNDEFORMED(BEAM=bt,THK=.2);

// NEW WRAPPER FUNCTION FOR BEAM FROM NODES
// THAT ADDS FIRST AND LAST INVISIBLE BEAMS
function BEAM_FROM_NODES(nodes,TBEAMS,TENDS,w,THICKEN_ENDS=false,T_MID=false,TUP = 1.03,S=9,index=0,prior_ang=0) =
    let (length = sqrt((nodes[0][0]-nodes[1][0])^2 + (nodes[0][1]-nodes[1][1])^2))
    let (ang = atan2((nodes[1][1]-nodes[0][1]),(nodes[1][0]-nodes[0][0])))
    concat([[Qstart,length,TENDS,w,ang]],
    concat(beamFromNodes(nodes=nodes,t=TBEAMS,w=w,THICKEN_ENDS=THICKEN_ENDS,T_MID=T_MID, TUP=TUP,S=S,index=0,prior_ang=ang),[[Qend,length,TENDS,w,ang]]) );

// DO NOT CALL THIS DIRECTLY
function beamFromNodes(nodes,t,w,THICKEN_ENDS=false,T_MID=false,TUP = 1.03,S=9,index=0,prior_ang=0) =
    // beam stresses at the fix endS can be larger than reported, due to stress concentrations
    // THICKEN_ENDS option will gradually increase thickenss of the ends
    // TUP is scaler for thickening up the ends
    // S is the number of nodes from each end to thicken
    // More nodes, increase S, decrease TUP
    let (n = len(nodes)-1)
    index < n ? 
    let (T_NEW_1 = (index < S && THICKEN_ENDS) ? t*TUP^(S-index) : t) // Thicken Start
    let (T_NEW_2 = (index > n-S && THICKEN_ENDS) ? t*TUP^(index-(n-S-1)) : T_NEW_1) // Thicken End
    // Thicken up the middle
    let (MID_N = floor(n/2))
    let (nfm = index-MID_N) // neg before mid, pos after
    let (T_NEW_3 = (nfm <= 0 && nfm > -S && T_MID) ? t*TUP^(S+nfm) : T_NEW_2) 
    let (T_NEW_4 = (nfm > 0 && nfm < S && T_MID) ? t*TUP^(S-nfm) : T_NEW_3)
    let (T_NEW = T_NEW_4)
    let (length = sqrt((nodes[index][0]-nodes[index+1][0])^2 + (nodes[index][1]-nodes[index+1][1])^2))
    let (dx=nodes[index+1][0]-nodes[index][0])
    let (dy=nodes[index+1][1]-nodes[index][1])
    let (ang = index==n-1 ? prior_ang : atan2((nodes[index+2][1]-nodes[index+1][1]),(nodes[index+2][0]-nodes[index+1][0])))
    //echo(index=index,T_NEW) 
    concat([[Qbeam,length,T_NEW,w,ang-prior_ang]],beamFromNodes(nodes,t,w,THICKEN_ENDS,T_MID,TUP,S,index+1,ang))  : [] ;

//junk = [[0,1],[0,2],[0,3]];
//new_first = SWAP_FIRST(ARRAY=junk,NEW_FIRST=[111,10]);
//new_last = SWAP_LAST(ARRAY=junk,NEW_LAST=[999,9]);
//echo(junk=junk,new_first=new_first,new_last=new_last);

// FUNCTION TO SWAP LAST MEMBER IN AN ARRAY
function SWAP_LAST(ARRAY, NEW_LAST) =
    let (n = len(ARRAY))
    [ for (i=[0:1:n-1]) (i<n-1) ? ARRAY[i] : NEW_LAST ] ;

// FUNCTION TO REMOVE LAST MEMBER IN AN ARRAY
function RMV_LAST(ARRAY) =
    let (n = len(ARRAY))
    [ for (i=[0:1:n-2])  ARRAY[i] ] ;

// FUNCTION TO SWAP FIRST MEMBER IN AN ARRAY
function SWAP_FIRST(ARRAY, NEW_FIRST) =
    let (n = len(ARRAY))
    [ for (i=[0:1:n-1]) (i==0) ? NEW_FIRST : ARRAY[i] ] ;
    
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
    //echo("BEAM",i=i,count=count,LDB[i][Ztype])
        ((LDB[i][Ztype] == Qbeam) ? 
            count_beams(LDB,i+1,count+1) 
            : count_beams(LDB,i+1,count) )
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
    for (i=[0:numPts-1]) translate([pts[i][0],pts[i][1],0]) color("black") cylinder(h=1,d=dia,$fn=8);
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
function Scale_Loads(int_loads,scale=1) = 
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

// function to sum moments from tail of tree to root
function sum_moments(moments) =
    let (n = len(moments))
    [ for (i=[0:1:n-1]) (sum_range(moments,i,n-1)) ];
    
// function to add beam_moments to node_loads
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
    echo(BEAMNO=BEAMNO,"##### NEGATIVE MARGIN OF SAFETY, GAUGE UP! NEW THK=",newTHK) 
    newTHK 
    : thk ;  // return current thickness is MS is positive

// module that draws the loads 
module draw_loads(nodes,loads,torques,scale=1,z_offset=1) {
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
