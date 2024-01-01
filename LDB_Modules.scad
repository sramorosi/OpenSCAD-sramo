// Large Displacement Beam, Modules and Functions
include <LDB_Indexes.scad>
//use <ME_lib.scad>
use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

// Display intermediate load steps?
Display_steps = false;
// Scale of Force & Moment Display
force_scale = 1.0; // [0.05:.05:2.0]
// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI), PLA=340000,PETG=300000,Polycar=320000
E = 340000;  // GLOBAL VARIABLE USED IN MANY FUNCTIONS AND MODULES
// ~Stress level at which the part will fail (PSI)
Failure_Stress = 10000;
// This could be tensile failure, compression failure, bending, etc.
// material density (lb per inch^3)
density = 0.043;
STEPS = 5;  // A GOOD NUMBER OF STEPS IS DEPENDENT ON THE PROBLEM

    // CANTILEVER BEAM WITH FORCE, 7 SEGMENT, 125 g
    // TEST CASE: L=7, t=0.062, w=1.06, F=0.278 lb
    NumberBeams=7; 
    L=7;
    LN = L/NumberBeams;
    t=0.062;
    w=1.06;
    weight=0.278; // lbs
    //E = 320000; // Polycarbonate
    //density = 0.043;
    // MEASURED TEST DATA
    pts=[[0,0],[1,.1],[1.95,0.5],[2.85,1],[3.7,1.6],[4.5,2.2],[5.3,2.9],[6,3.6]];
    draw_points(pts,dia=0.05);

    BEAM1 = [for (i=[1:NumberBeams]) [Qbeam,LN,t,w,0]];
    echo(BEAM1=BEAM1);
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[0,weight,0]]);
    echo(LOADS1=LOADS1);
    
    draw_beam_undeformed(BEAM1);
    Do_Analysis(BEAM1,LOADS1,force_scale*.5,Display_steps,Failure_Stress,density,steps=STEPS);

module Do_Analysis(LDB_DEF,NODE_LOADS,f_scale=1,Display_steps=true,Failure_Stress=5000,density=0.05,origin=[0,0,0],steps=STEPS) {
    echo("******* LARGE DISPLACEMENT 2D BEAM ANALYSIS BASED ON COMPLIANT MECHANISM PRBM *******");
    echo(E=E,Failure_Stress=Failure_Stress,density=density);
    
    // perform data checks
    num_beams = count_beams(LDB_DEF);
    echo("NUMBER OF BEAMS IS ",num_beams);
    ttl_load = sum_loads(NODE_LOADS);
    echo(" TOTAL LOAD IS ",ttl_load);
    if (num_beams > 0 && ttl_load > 0) {
    
        // Spread Loads. Moments don't include force-moments at this time
        initial_loads = spread_ext_loads(NODE_LOADS);
        echo(initial_loads=initial_loads);

        // Generate GLOBAL Beam ANGLES, undeformed
        beam_angles = global_angles(LDB_DEF);
        echo("INITIAL ",beam_angles=beam_angles);

        Results=compute_steps(LDB_DEF,Failure_Stress,density,initial_loads, beam_angles,STEPS=steps,n=steps); 

        translate(origin) union () draw_beam_deformed(LDB_DEF,Results);

        FinalNodes = concat([origin],getNodeFromResults(Results));
        echo(FinalNodes = FinalNodes);
        color("red") draw_points(FinalNodes,dia=0.05);

        echo("X MAX ",max_tree(FinalNodes,Nx)+origin[0],
             "  X MIN ",min_tree(FinalNodes,Nx)+origin[0]);
        echo("Y MAX ",max_tree(FinalNodes,Ny)+origin[1],
             "  Y MIN ",min_tree(FinalNodes,Ny)+origin[1]);
             
        echo("STRESS MAX ",max_tree(Results,Zstressmax),
             "  STRESS MIN ",min_tree(Results,Zstressmin));
           
        echo("MIN Margin of Safety ",min_tree(Results,Zms),
             " MAX MS ",max_tree(Results,Zms));
             //sum_fwd(ARRAY,i,thing,start=0)
        echo("WEIGHT ",sum_fwd(Results,0,Zweight),  // FIX THIS
             " ENERGY ",sum_fwd(Results,0,Zenergy));

    } else echo("**NUMBER OF BEAMS OR LOADS IS ZERO, TERMINATING**"); 
}

// Recursive function to perform analysis in steps (n)
function compute_steps(LDB_DEF,Failure_Stress,density,loads,beam_angles, STEPS=1,n=1) =
    let (loadScale = (((STEPS+1)-n)/STEPS))
    // scale internal loads
    let (loads_scaled = scale_int_loads(loads,loadScale))
    let (newResults = compute_iteration(LDB_DEF,Failure_Stress,density,loads_scaled,beam_angles,n))
    let (Nodes0 = concat([[0,0]],getNodeFromResults(newResults)))
    let (newAngleVec = getAnglesFromNodes(Nodes0))
echo(n=n," load scale = ",loadScale,newAngleVec=newAngleVec)
    ( n==1 ? newResults : compute_steps(LDB_DEF,Failure_Stress,density,loads,newAngleVec,STEPS,n-1) );

function compute_iteration(LDB_DEF,Failure_Stress,density,loads,beam_angles,n=0) =     
    // Compute an iteration (to update moments due to forces)
    // Convert internal global forces to beam-local forces 
    let (loads_local = rotate_int_loads(loads,beam_angles))
echo(n,loads_local=loads_local)
    // Calculate moments due to forces
    let (force_mom_temp = momentsDueToForce(loads_local, LDB_DEF, beam_angles))
echo(n,force_mom_temp=force_mom_temp)
    // Sum moments due to forces, so that each beam has correct moment
    let (beam_moments = sum_moments(force_mom_temp))
echo(n,beam_moments=beam_moments)
    // Add moments-due-to-forces with internal loads
    let (NEW_loads_local = add_moments_to_loads(loads_local,beam_moments))
echo(n,NEW_loads_local=NEW_loads_local)
    // call function compute results
    compute_results(LDB_DEF,NEW_loads_local);

function compute_results(LDBdef,loads) = 
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) 
        getBeamAngleFromLoads(LEN=LDBdef[i][Zlen] ,THK=LDBdef[i][Zthk] ,W=LDBdef[i][Zw], Fx=loads[i][Zfx], Fy=loads[i][Zfy], M=loads[i][Zm]) ];

// Given a Beam Element and Loads, Calculate the deflection
function getBeamAngleFromLoads(LEN=2,THK=0.15,W=0.8,Fx=0,Fy=10,M=0) = 
    let (Iz = ((W*THK*THK*THK)/12))
    let (AREA = THK*W)
    let (BT = beam_type(Fy,M,LEN))
    let (K = spring_rate(BT,Iz,LEN,E))//force per radian, DEAD CODE
    let (cr = characteristic_radius(BT))
    let (theta = spring_angle(BT,Fy,M,LEN,E,Iz)) // degrees.  THETA NEEDS TO CONVERGE.
    let (t_rad = theta * PI / 180)      // radians
    let (theta_end = end_angle(BT,theta)) // degrees
    let (a = a_position(LEN,cr,theta))  // non linear displacement
    let (b = b_position(LEN,cr,theta))  // non linear displacement
    let (bLinear = Fy*LEN*LEN*LEN/(3*E*Iz))  // linear displacement
    let (bRR = (b>0? abs(1-(bLinear/b)): 0 ))  //error between linear and non linear
    let (c = THK / 2)              // half thickness
    let (m_total = M - Fx * b + Fy * a)
    let (stressmax = m_total*c/Iz + Fx/a)
    let (stressmin = -m_total*c/Iz + Fx/a)
    let (ms = (abs(stressmax) > abs(stressmin) ? Failure_Stress/abs(stressmax)-1 : Failure_Stress/abs(stressmin)-1))
    let (energy = 0.5* K * (t_rad*t_rad))  // PE = 1/2 * K * x ^2
    let (weight = AREA*L*density)
//echo(BT=BT,cr=cr,theta=theta,theta_end=theta_end,a=a,b=b,bLinear=bLinear, bRR=bRR,ms=ms)
    // QUALITY CHECKING HERE:
    // if beam type is vertical force and has diverged from linear
    (bRR > 0.15 && BT==Zvertforce ? 
        echo("***** NON LINEAR BEAM - DECREASE ELEMENT LENGTH *****") 
       [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,weight] 
       : // no quality problems
       [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,weight]);

// get beam type from forces 
// PE = 1/2 * K * x ^2
// check energy of both?
function beam_type(fy=1,m=.2,L=1) = 
    ((abs(fy*L)>(abs(m)/2) )? Zvertforce : Zendmoment );  // 0 use A.1.2 vertical force constants,  1 use A.1.5 moment
    
function characteristic_radius(bt) = 
    (bt == Zvertforce ? 0.85 : 0.7346);
    
function spring_rate(bt,i,len,E) = 
    (bt == Zvertforce ? 2.258*E*i/len : 1.5164*E*i/len);
    
function spring_angle(bt,fy,moment,L,E,inertia) =
// RETURE THE ANGLE GIVEN THE LOAD (DEGREES)
// Theta is a function of  COSINE THETA.  Need to iterate a few times to decrease the error.
    let (rad_deg = 180/PI)
    let (fm = moment/L)
    let (force = fm + fy)
    let (Kforce = 2.258*E*inertia/L)
    let (Kmoment = 1.5164*E*inertia/L)
    let (CRforce = 0.85)
    let (CRmoment = 0.7346)
    let (ang_est = rad_deg*(moment/(Kforce)))
    let (angle1 = rad_deg*(force/Kforce)*CRforce*L*cos(ang_est))
    let (angle2 = rad_deg*(force/Kforce)*CRforce*L*cos(angle1))
    let (angle3 = rad_deg*(force/Kforce)*CRforce*L*cos(angle2))
    let (angle4 = rad_deg*(force/Kforce)*CRforce*L*cos(angle3))
    let (error1 = angle1-ang_est)
    let (error2 = angle2-angle1)
    let (error3 = angle3-angle2)
    let (error4 = angle4-angle3)
//echo(bt=bt,error1=error1,error2=error2,error3=error3,error4=error4)
    //rad_deg*(force/Kmoment)*L;
    //rad_deg*(force/Kforce)*CRforce*L*cos(angle4);
    //(bt == Zvertforce ? rad_deg*(fy/K)*cr*L*cos(angle4) : rad_deg*((fm+fy)/K)*L );
    (bt == Zvertforce ? rad_deg*(fy/Kforce)*CRforce*L*cos(angle4) : 
                  rad_deg*(fm/Kmoment)*L ); // best for pure moments?
    //           rad_deg*(fm/Kmoment)*L*CRmoment*cos(angle4) + rad_deg*(fy/Kforce)*CRforce*L*cos(angle4) );
    //(bt == Zvertforce ? rad_deg*(fy/K)*cr*L*cos(angle4) : rad_deg*(moment/K + (fy/K)*cr*L*cos(angle4)) );
    
function end_angle(bt,angle) = 
    (bt == Zvertforce ? angle*1.24 : angle*1.5164);

function a_position(L,cr,theta) = L*(1-cr*(1-cos(theta)));
function b_position(L,cr,theta) = cr*L*sin(theta);

// recursive module that draws the undeformed beam.
module draw_beam_undeformed(LDBdef,idx = 0) {
// LDBdef (LDB definition)
    elem_type = LDBdef[idx][Ztype];
//echo(idx=idx,elem_type=elem_type);
    if (elem_type == Qbeam) { // Note: undefined causes recursion to stop
        L = LDBdef[idx][Zlen];
        t = LDBdef[idx][Zthk];
        w = LDBdef[idx][Zw];
        z_ang = LDBdef[idx][Zang];
        
        // draw the beam segment
        rotate([0,0,z_ang]) translate([0,-t/2,0]) cube([L,t,w]);

        // Recursive call generating the next beam
        rotate([0,0,z_ang]) translate([L,0,0])
                draw_beam_undeformed(LDBdef,idx + 1);
    } 
}

// recursive module that draws the deformed beam.
module draw_beam_deformed(LDBdef,results,idx = 0) {
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
        color ([val_red(ms),val_green(ms),0.2]) linear_extrude(height=w,center=true) hull() { 
            rotate([0,0,LDBdef_ang]) translate([babyL,0,0]) circle(d=t,$fn=16);
            circle(d=t,$fn=16);  // ZERO,ZERO
        }
        color ([val_red(ms),val_green(ms),0]) linear_extrude(height=w,center=true) hull() { 
            rotate([0,0,LDBdef_ang]) translate([babyL,0,0]) circle(d=t,$fn=16);
            rotate([0,0,LDBdef_ang]) translate([a,b,0]) circle(d=t,$fn=16);
        }
        // Recursive call generating the next beam
        rotate([0,0,LDBdef_ang]) translate([a,b,0]) rotate([0,0,end_ang]) 
                draw_beam_deformed(LDBdef,results,idx + 1);
    } 
}

// TO BE FIXED:
// recursive module that draws loads given beam node locations
module draw_loads(nodes,loads,scale=1,node_color="blue",idx = 0) {
    elem_type = loads[idx][Ztype];
    if (elem_type == Qload) { // typically this is the last node
        // do nothing, Recursive call
        x = nodes[idx][Zdx];
        y = nodes[idx][Zdy];
        // draw the node 
        color(node_color) linear_extrude(height=4,center=true) hull() { 
            translate([x,y,0]) circle(d=.1,$fn=16); }
        draw_loads(nodes,loads,scale,node_color,idx + 1);
    } else if (elem_type == Qskip) {
        x = nodes[idx][Zdx];
        y = nodes[idx][Zdy];
        fx = loads[idx][Zfx];
        fy = loads[idx][Zfy];
        moment = loads[idx][Zm];
        // draw the node
        color(node_color) linear_extrude(height=4,center=true) hull() { 
            translate([x,y,0]) circle(d=.1,$fn=16); }
//echo(idx=idx,elem_type=elem_type,x=x,y=y,fx=fx,fy=fy,moment=moment);
        // draw forces and torques
        fmag = sqrt(fx*fx + fy*fy);
        if (abs(fmag)>0.1) color ("red") 
            translate([x,y,0]) force_arrow([0,0,0],[fx,fy,0],mag=fmag*scale);
        if (abs(moment)>0.1) color ("blue")
            translate([x,y,0]) torque_arrow([0,0,0],mag=moment*scale);
        // Recursive call generating the next beam
        draw_loads(nodes,loads,scale,node_color,idx + 1);
    } else if (elem_type == Qfork ) {
        // FORK, PROCESS TWO BEAM SEGMENTS
        // Recursive call generating the first fork
        draw_loads(nodes[idx+1][1],loads[idx][1],scale,node_color,idx=0);
        // Recursive call generating the second fork
        draw_loads(nodes[idx+1][2],loads[idx][2],scale,node_color,idx=0);
    }
}

// module that draws the loads at ground (not recursive) AND REPORTS REACTIONS
module draw_ground_reactions(results,scale=1,origin=[0,0,0],z_rot=0,out=true,nodes) {
    // Assumes that the ground reaction is element 0
    fx = -results[0][ZPx];
    fy = -results[0][ZPy];
    moment = -results[0][ZPm];
    lNodes = len(nodes);
    //echo(lNodes=lNodes);
    globalMoment = fx*nodes[lNodes-2][2] + fy*nodes[lNodes-2][1];
    fmag = sqrt(fx*fx + fy*fy);
    // draw forces and torques
    if (abs(fmag)>0.1) color ("red") 
        translate(origin) rotate ([0,0,z_rot]) force_arrow([0,0,0],[fx,fy,0],mag=fmag*scale);
    if (abs(moment)>0.1) color ("blue")
        translate(origin) rotate ([0,0,z_rot]) torque_arrow([0,0,0],mag=moment*0.01*scale);
    // REPORT REACTIONS
    if (out) 
        echo("GROUND REACTIONS ",fx=rot_x(fx,fy,z_rot),fy=rot_y(fx,fy,z_rot),moment=moment,globalMoment=globalMoment);
}

// sum angles along segments to get global angles.
function global_angles(LDBdef,prior_ang=0) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) 
    let (new_sum = sum_fwd(LDBdef,i,Zang)) 
//echo("ANGLES",i=i,prior_ang=prior_ang, new_sum=new_sum)
    new_sum+prior_ang ] ;

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
    echo("SPREAD EXT LOADS",n=n)
    [ for (i=[0:1:n-1]) 
    [sum_fwd(ext_loads,n-1,Zfx,i), sum_fwd(ext_loads,n-1,Zfy,i), sum_fwd(ext_loads,n-1,Zm,i)]];

// recursive function to rotate the internal forces from a global system to a beam-local system.  Moments are copied
function rotate_int_loads(int_loads,beam_angles) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) (
        let (fx = int_loads[i][Zfx])
        let (fy = int_loads[i][Zfy])
        let (ang = (i==0? 0 :-beam_angles[i-1])) // special case
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
// Modified example from ChatGPT
function getNodeFromResults(resultsArray, index=0,x_past=0,y_past=0,ang_past=0) = 
    index < len(resultsArray) ?
        let (x = resultsArray[index][Za])
        let (y = resultsArray[index][Zb])
        let (ang = (index==0 ? 0 : resultsArray[index-1][Zthetaend]))
        let (sum_ang = ang + ang_past) // add up to get new angle
        let (x_new = rot_x(x,y,sum_ang) + x_past) // add up to get new x
        let (y_new = rot_y(x,y,sum_ang) + y_past) // add up to get new y
        //echo(index=index,x=x,y=y,y_past=y_past,sum_ang=sum_ang)
        concat([ [x_new , y_new ] ],
        // Recursive call to process the next point
        getNodeFromResults(resultsArray, index + 1, x_new , y_new , sum_ang) ) 
    :  [] ;  // Return nothing when all points are processed

function getAnglesFromNodes(NodesArray, index=1,x_past=0,y_past=0) = 
    index < len(NodesArray) ?
        let (x = NodesArray[index][Nx])
        let (y = NodesArray[index][Ny])
        let (beamAng = atan2(y-y_past,x-x_past))
   //echo(index=index,x=x,y=y,y_past=y_past,beamAng=beamAng)
        concat([ beamAng ],
           getAnglesFromNodes(NodesArray, index + 1, x , y ) ) 
    :  [] ;  // Return nothing when all points are processed
