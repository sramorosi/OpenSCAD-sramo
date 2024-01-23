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
E = 340000;  // GLOBAL VARIABLE USED IN MANY FUNCTIONS AND MODULES
// ~Stress level at which the part will fail (PSI)
Failure_Stress = 10000;
// This could be tensile failure, compression failure, bending, etc.
// material density (lb per inch^3)
density = 0.043;
Load_Steps = 3;  // A GOOD NUMBER OF STEPS IS DEPENDENT ON THE PROBLEM

    // beam from points
    t=.06;  
    w=0.5;
    HGT = 6;  // height of overall beam
    //pts=[for (i=[0:NumberBeams]) [X*cos(180*(i/NumberBeams))-X,Y*sin(180*(i/NumberBeams))] ]; // sine shape
    //
    pts=[[0,0],[0,HGT]];  // frame shape
    new_pts = addPoints(pts,.5);
    echo(new_pts=new_pts);
    //draw_points(new_pts,dia=0.1);
    
    BEAM1 = beamFromNodes(new_pts,t,w);
    
    NumberBeams = len(BEAM1);
    echo(BEAM1=BEAM1," n=", len(BEAM1));
        
    Fx = .8;
    Mz = Fx*HGT/2.2;

    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,0,Mz]]);
    //LOADS1 = concat ( concat(concat([for (i=[1:(NumberBeams/2)-1]) [0,0,0]],[[Fx,0,Fx*HGT/2]]),[for (j=[1:(NumberBeams/2)]) [0,0,0]]),[[0,0,0]]); // << last load is back solved to hold node in location  [-.4,.1,2.4]
       
    echo(LOADS1=LOADS1," n=", len(LOADS1));

    ORIGIN = [2,0,0];

    translate(ORIGIN) draw_beam_undeformed(BEAM1); 
    
    Do_Analysis(BEAM1,LOADS1,force_scale,Display_steps,Failure_Stress,density,ORIGIN,steps=4);

module Do_Analysis(LDB_DEF,NODE_LOADS,fscale,Display_steps,Failure_Stress,density,Origin,steps) {
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
    
        // Spread Loads. Moments don't include force-moments at this time
        initial_loads = spread_ext_loads(NODE_LOADS);
        //echo(initial_loads=initial_loads);

        // Generate GLOBAL Beam ANGLES, undeformed
        beam_angles = global_angles(LDB_DEF);
        echo("INITIAL ",beam_angles=beam_angles);

        // MAIN ANALYSIS (MODULE)
        computeStepsModule(f_scale=fscale,LDB_DEF=LDB_DEF,Failure_Stress=Failure_Stress,density=density,loads=initial_loads,NODE_LOADS=NODE_LOADS, beam_angles=beam_angles,original_angles=beam_angles,Origin=ORIGIN,STEPS=steps,n=steps,displaySteps=Display_steps); 
        
    } else echo("**NUMBER OF BEAMS OR LOADS IS ZERO, TERMINATING**"); 
}

module output_STRESS_MS_Energy(n,loadScale,Results) { 
    echo(n=n," load scale = ",loadScale,
    " σ MAX ",max_tree(Results,Zstressmax),
             " σ MIN ",min_tree(Results,Zstressmin),
             " MIN MS ",min_tree(Results,Zms),
             " MAX MS ",max_tree(Results,Zms),
             " ENERGY ",sum_fwd(Results,0,Zenergy),
            " WEIGHT ",sum_fwd(Results,0,Zweight));
}

// Recursive Module to perform analysis in steps (n)
module computeStepsModule(f_scale=1,LDB_DEF,Failure_Stress,density,loads,NODE_LOADS,beam_angles,original_angles, Origin, STEPS=1,n=1,displaySteps=true) {
    
    loadScale = (((STEPS)-n)/STEPS);
    // scale internal loads
    loads_scaled = scale_int_loads(loads,loadScale);
    
    newResults = compute_iteration(LDB_DEF,Failure_Stress,density,loads_scaled,beam_angles,n);
    
    Nodes = concat([Origin],getNodeFromResults(newResults,original_angles, x_past=Origin[0], y_past=Origin[1]));
    newAngleVec = getAnglesFromNodes(Nodes); // update the beam_angles array
//echo(newAngleVec=newAngleVec);
       
    output_STRESS_MS_Energy(n,loadScale,newResults);
    
    if ( n>0 ) { // recursion.  Counts down.
        computeStepsModule(f_scale,LDB_DEF,Failure_Stress,density,loads,NODE_LOADS,newAngleVec,original_angles, Origin, STEPS, n-1 , displaySteps);
        
        if (displaySteps) { color("red",loadScale) draw_points(Nodes,dia=0.05); }
    }
    if (n==0) {  // last iteration
        
        if (displaySteps) { color("red",loadScale) draw_points(Nodes,dia=0.05); }

        translate(Origin) union () draw_beam_deformed(LDB_DEF,newResults);
        
        draw_loads(nodes=Nodes, loads=loads_scaled, results=newResults,scale=f_scale);

        echo("X MAX ",max_tree(Nodes,Nx)+Origin[0],
             "  X MIN ",min_tree(Nodes,Nx)+Origin[0]);
        echo("Y MAX ",max_tree(Nodes,Ny)+Origin[1],
             "  Y MIN ",min_tree(Nodes,Ny)+Origin[1]);
    }
}

function compute_iteration(LDB_DEF,Failure_Stress,density,loads,beam_angles,n=0) =    
    // Compute an iteration (to update moments due to forces)
    // Returns a Results array
    // Convert internal global forces to beam-local forces 
    let (loads_local = rotate_int_loads(loads,beam_angles))
//echo(n=n,loads_local=loads_local)
    // Calculate moments due to forces
    let (force_mom_temp = momentsDueToForce(loads_local, LDB_DEF, beam_angles))
//echo(n=n,force_mom_temp=force_mom_temp)
    // Sum moments due to forces, so that each beam has correct moment
    let (beam_moments = sum_moments(force_mom_temp))
//echo(n=n,beam_moments=beam_moments)
    // Add moments-due-to-forces with internal loads
    let (NEW_loads_local = add_moments_to_loads(loads_local,beam_moments))
//echo(n=n,NEW_loads_local=NEW_loads_local)
echo("STEP ",n=n," GROUND REACTIONS: Fx=",NEW_loads_local[0][0]," Fy=",NEW_loads_local[0][1]," M=",NEW_loads_local[0][2])
    // Loop thru all beam elements and compute angular deflections
    let (n = len(LDB_DEF))
    [ for (i=[0:1:n-1]) 
        getBeamAngleFromLoads(LEN=LDB_DEF[i][Zlen] ,THK=LDB_DEF[i][Zthk] ,W=LDB_DEF[i][Zw], Fx=NEW_loads_local[i][Zfx], Fy=NEW_loads_local[i][Zfy], M=NEW_loads_local[i][Zm], BEAM_NO = i) ];

// Given a Beam Element and Loads, Calculate the deflection
function getBeamAngleFromLoads(LEN=2,THK=0.15,W=0.8,Fx=0,Fy=10,M=0, BEAM_NO) = 
    let (Iz = ((W*THK*THK*THK)/12))
    let (AREA = THK*W)
    let (BT = beam_type(Fy,M,LEN))
    let (Krotate = spring_rate(BT,Iz,LEN,E))//force per radian, Only used for energy calc
    let (cr = characteristic_radius(BT))
    let (theta = spring_angle(BT,Fy,M,LEN,E,Iz)) // degrees. 
    let (t_rad = theta * PI / 180)      // radians
    let (theta_end = end_angle(BT,theta)) // degrees
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
    let (weight = AREA*LEN*density)
//echo(BT=BT,cr=cr,theta=theta,theta_end=theta_end,a=a,b=b,bLinear=bLinear, bRR=bRR,ms=ms)
//echo(Krotate=Krotate,Kaxial=Kaxial,LEN=LEN,NEW_LEN=NEW_LEN)
//echo(EnergyRotate=EnergyRotate,EnergyAxial=EnergyAxial,energy=energy)
    // QUALITY CHECKING HERE:
    // if beam type is vertical force and has diverged from linear
    (bRR > 0.15 && BT==Zvertforce ? 
        echo("***** NON LINEAR BEAM - DECREASE ELEMENT LENGTH *****") 
       [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,weight,newTHK,-Fx,-Fy,-m_total] 
       : // no quality problems
       [theta,theta_end,a,b,cr,ms,stressmin,stressmax,energy,weight,newTHK,-Fx,-Fy,-m_total]);

function getSafetyMargin(stressMax,stressMin,FailureStress) = 
    (abs(stressMax) > abs(stressMin)) ? 
        FailureStress/abs(stressMax)-1 
        : FailureStress/abs(stressMin)-1 ;

function checkMS(ms,thk,BEAMNO) = 
    ms < 0.0 ?
    let (newTHK = thk*(1-ms))
    echo(BEAMNO=BEAMNO,"##### NEGATIVE MARGIN OF SAFETY, GAUGE UP!! ###### NEW THK=",newTHK) newTHK 
    : thk ;  // return current thickness is MS is positive
    
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
    let (angle0 = rad_deg*(moment/(Kforce)))
    let (angle1 = rad_deg*(force/Kforce)*CRforce*L*cos(angle0))
    let (angle2 = rad_deg*(force/Kforce)*CRforce*L*cos(angle1))
    let (angle3 = rad_deg*(force/Kforce)*CRforce*L*cos(angle2))
    //let (angle4 = rad_deg*(force/Kforce)*CRforce*L*cos(angle3))
    let (check_ang = (abs(angle3) > 15) ? echo("###BEAM ANGLE LIMIT EXCEEDED###",angle3=angle3) : angle3)
//echo(bt=bt,angle0=angle0,angle1=angle1,angle2=angle2,angle3=angle3)
    //let (error1 = angle1-ang_est)
    //let (error2 = angle2-angle1)
    //let (error3 = angle3-angle2)
    //let (error4 = angle4-angle3)
//echo(bt=bt,error1=error1,error2=error2,error3=error3,error4=error4)
    (bt == Zvertforce ? rad_deg*(fy/Kforce)*CRforce*L*cos(angle3) : 
                  rad_deg*(fm/Kmoment)*L ); 

function end_angle(bt,angle) = 
    (bt == Zvertforce ? angle*1.24 : angle*1.5164);

function a_position(L,cr,theta) = L*(1-cr*(1-cos(theta)));
function b_position(L,cr,theta) = cr*L*sin(theta);

// Draws the undeformed beam (runs analysis with zero loads)
module draw_beam_undeformed(LDBdef) {
    // LDBdef (LDB definition).  All else is derived
    ZEROLOADS = concat([for (i=[1:len(LDBdef)]) [0,0,0]],[[0,0,0]]);
    beam_angles = global_angles(LDBdef);
    //echo(beam_angles=beam_angles);
    results = compute_iteration(LDBdef,Failure_Stress,density,ZEROLOADS,beam_angles,1);
    //echo(results=results);
    draw_beam_deformed(LDBdef,results); 
}

// recursive module that draws the deformed beam.
module draw_beam_deformed(LDBdef,results,idx = 0,prior_ang=0) {
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
                draw_beam_deformed(LDBdef,results,idx + 1,LDBdef_ang+prior_ang);
    } 
}

// module that draws the loads 
module draw_loads(nodes,loads,results,scale=1,z_rot=0) {
    //echo("DRAW ",nodes=nodes,loads=loads);
    rotate ([0,0,z_rot]) {
        for (i=[0:len(loads)-1]) {
            fx = loads[i][Zfx];
            fy = loads[i][Zfy];
            moment = results[i][ZReactm];
            fmag = sqrt(fx*fx + fy*fy);
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
// NEED TO APPEND INITIAL ANGLES
function getNodeFromResults(resultsArray,initAngles,x_past=0,y_past=0,ang_past=0, index=0) = 
    index < len(resultsArray) ?
        let (x = resultsArray[index][Za])
        let (y = resultsArray[index][Zb])
        let (ang = (index==0 ? 0 : resultsArray[index-1][Zthetaend]))
        let (initAng = initAngles[index])
        let (sum_ang = initAng + ang + ang_past) // add up to get new angle
        let (x_new = rot_x(x,y,sum_ang) + x_past) // add up to get new x
        let (y_new = rot_y(x,y,sum_ang) + y_past) // add up to get new y 
//echo(index=index,x=x,y=y,y_past=y_past,initAng=initAng,sum_ang=sum_ang)
        concat([ [x_new , y_new ] ],
        // Recursive call to process the next point
        getNodeFromResults(resultsArray, initAngles, x_new , y_new , sum_ang, index + 1) ) 
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