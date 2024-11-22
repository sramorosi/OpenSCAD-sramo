// Large Displacement Beam, Modules and Functions
include <NEW_LDB_Indexes.scad>
//use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

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

/*
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
    
// Function to reverse an array of points
function reverse_array(arr) =
    let(len_arr = len(arr)) 
        [for (i = [0 : len_arr - 1]) arr[len_arr - i - 1]] ;

/*
function getAnglesFromNodes(NodesArray,x_start=0,y_start=0, index=1) = 
    index < len(NodesArray) ?
        let (x = NodesArray[index][Nx])
        let (y = NodesArray[index][Ny])
        let (beamAng = atan2(y-y_start,x-x_start))
 //echo(index=index,x=x,x_start=x_start,y=y,y_start=y_start,beamAng=beamAng)
        concat([ beamAng ],
           getAnglesFromNodes(NodesArray, x , y , index + 1) ) 
    :  [] ;  // Return nothing when all points are processed
*/

/*
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
*/