// Large Displacement Beam, Modules and Functions
include <LDB_Indexes.scad>
use <force_lib.scad>

// recursive module that draws the undeformed beam.
module draw_beam_undeformed(dna,idx = 0) {
    elem_type = dna[idx][Ztype];
//echo(idx=idx,elem_type=elem_type);
    if (elem_type == Qbeam) {
        L = dna[idx][Zlen];
        t = dna[idx][Zthk];
        w = dna[idx][Zw];
        z_ang = dna[idx][Zang];
        // draw the beam segment
        color ("green") linear_extrude(height=w,center=true) hull() { 
            rotate([0,0,z_ang]) translate([L,0,0]) circle(d=t,$fn=16);
        }
        // Recursive call generating the next beam
        rotate([0,0,z_ang]) translate([L,0,0])
                draw_beam_undeformed(dna,idx + 1);
    } else if (elem_type == Qfork ) {
        // FORK, PROCESS TWO BEAM SEGMENTS
        // Recursive call generating the first fork
//echo (" in fork 1");
        draw_beam_undeformed(dna[idx][1]);
        // Recursive call generating the second fork
//echo (" in fork 2");
        draw_beam_undeformed(dna[idx][2]);
    } else if (elem_type == Qload) {
                draw_beam_undeformed(dna,idx + 1);
    }  
    // Note: that an undefined causes the recursion to stop
}


// recursive module that draws loads given beam node locations
module draw_loads(nodes,loads,scale=1,idx = 0) {
    elem_type = loads[idx][Ztype];
//echo(idx=idx,elem_type=elem_type);
    //if ((elem_type == Qload) || (elem_type == Qskip)) {
    //    x = ((elem_type == Qskip) ? 0 : nodes[idx][Zdx]);
    //    y = ((elem_type == Qskip) ? 0 : nodes[idx][Zdy]);
    if (elem_type == Qload) {
        // do nothing, Recursive call
        x = nodes[idx][Zdx];
        y = nodes[idx][Zdy];
        // draw the beam segment
        //color ("blue") linear_extrude(height=2,center=true) hull() { 
        //    translate([x,y,0]) circle(d=.1,$fn=16); }

        draw_loads(nodes,loads,scale,idx + 1);
    } else if (elem_type == Qskip) {
        x = nodes[idx][Zdx];
        y = nodes[idx][Zdy];
        fx = loads[idx][Zfx];
        fy = loads[idx][Zfy];
        moment = loads[idx][Zm];
        // draw the beam segment
        //color ("blue") linear_extrude(height=2,center=true) hull() { 
        //    translate([x,y,0]) circle(d=.1,$fn=16); }
//echo(idx=idx,elem_type=elem_type,x=x,y=y,fx=fx,fy=fy,moment=moment);
        // draw forces and torques
        fmag = sqrt(fx*fx + fy*fy);
        if (abs(fmag)>0.1) color ("red") 
            translate([x,y,0]) force_arrow([0,0,0],[fx,fy,0],mag=fmag*scale);
        if (abs(moment)>0.1) color ("blue")
            translate([x,y,0]) torque_arrow([0,0,0],mag=moment*scale);
        // Recursive call generating the next beam
        draw_loads(nodes,loads,scale,idx + 1);
    } else if (elem_type == Qfork ) {
        // FORK, PROCESS TWO BEAM SEGMENTS
        // Recursive call generating the first fork
        draw_loads(nodes[idx+1][1],loads[idx][1],scale,idx=0);
        // Recursive call generating the second fork
        draw_loads(nodes[idx+1][2],loads[idx][2],scale,idx=0);
    }
}


// recursive function to generate moment of inertia
function gen_Iz(dna) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (dna[i][Ztype] == Qbeam ? 
        Iz_func(dna[i][Zw],dna[i][Zthk]) : 
    (dna[i][Ztype] == Qfork ? 
    [Qfork, gen_Iz(dna[i][1]),gen_Iz(dna[i][2])] : Qskip ) ) ];
    
//  simple function to calculate moment of inertia about Z axis
function Iz_func(w=1,t=.1) = ((w*t*t*t)/12);
    
// recursive function to generate Area (no summation)
function gen_Area(dna) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (dna[i][Ztype] == Qbeam ? 
        dna[i][Zthk]*dna[i][Zw] : 
     (dna[i][Ztype] == Qfork ? 
    [Qfork, gen_Area(dna[i][1]),gen_Area(dna[i][2])] : Qskip )) ];

// sum angles along segments to get global angles.
//  undeformed = true performs the sum on the DNA tree
function global_angles(dna,prior_ang=0) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (dna[i][Ztype] == Qbeam ? 
    let (new_sum = sum_fwd(dna,i,Zang)) 
//echo("ANGLES",i=i,prior_ang=prior_ang, new_sum=new_sum)
    new_sum+prior_ang : 
    (dna[i][Ztype] == Qfork ? 
    let (new_prior_ang = sum_fwd(dna,i-1,Zang) + prior_ang)
     [Qfork, global_angles(dna[i][1],new_prior_ang),global_angles(dna[i][2],new_prior_ang)] :
    Qskip ) ) ];
    
// recursive forward sumation function to sum "thing"
// from the start (or s'th element) to the i'th element - remember elements are zero based
function sum_fwd(dna,i,thing,s=0) = 
    let (val = dna[i][thing])
    (i==s ? val : val + sum_fwd(dna,i-1,thing,s));

// Simple list sum fwd
function sum_fwd2(list,i,s=0) = 
    let (val = list[i])
    (i==s ? val : val + sum_fwd2(list,i-1,s));

// recursive tail sumation function to sum "thing"  ASSUMING A FLAT VECTOR
// from the i'th element to the last (or n'th element) - remember elements are zero based
function sum_tail(dna,i,thing) = 
    let (n = len(dna)-1)
    let (val = dna[i][thing])
    (i==n ? val : val + sum_tail(dna,i+1,thing));

// recursive tail sumation function to sum "thing"  AND ASSUMING A TREE (NESTED VECTOR)
// from the i'th element to the last (or n'th element)
function sum_tail2(dna,i,thing) = 
    let (val = dna[i][thing])
    let (type = dna[i][Ztype]) 
//echo(i=i,type=type,val=val)
    (type==Qfork ?  sum_tail2(dna[i][1],1,thing) + sum_tail2(dna[i][2],1,thing) : 
        (type==Qload ? val + sum_tail2(dna,1+i,thing) : 
        (val==undef ? 0 : val )));

// recursive function to generate DX AND DY of undeformed beam
function gen_dxdy_undeformed(dna,angles) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) 
    ( dna[i][Ztype] == Qbeam ? 
        let (L = dna[i][Zlen])
        let (ang = angles[i]) 
        let (x = L*cos(ang))
        let (y = L*sin(ang))
//echo(i=i,L=L,ang=ang)
        [Qbeam,x,y] : 
    (dna[i][Ztype] == Qfork ? 
    [Qfork, gen_dxdy_undeformed(dna[i][1],angles[i][1]),gen_dxdy_undeformed(dna[i][2],angles[i][2])] : 
    [Qskip,0,0,0,0] )) ];  

// recursive function to generate DX AND DY of deformed beam
function gen_dxdy_deformed(dna,results,angles) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) 
        let (x = results[i][Za])
        let (y = results[i][Zb])
        let (ang = angles[i]) 
    ( dna[i][Ztype] == Qbeam ? 
        [Qbeam,rot_x(x,y,ang),rot_y(x,y,ang)] : 
    (dna[i][Ztype] == Qfork ? 
    [Qfork, gen_dxdy_deformed(dna[i][1],results[i][1],angles[i][1]),
        gen_dxdy_deformed(dna[i][2],results[i][2],angles[i][2])] : 
    [Qskip,0,0,0,0] )) ];  

// recursive function to generate global nodes
function gen_nodes(dxdy,origin=[Qbeam,0,0]) =
    let (n = len(dxdy))
    [ origin, for (i=[0:1:n-1]) 
        let (x = sum_fwd(dxdy,i,Zdx) + origin[Zdx])
        let (y = sum_fwd(dxdy,i,Zdy) + origin[Zdy])
    ( dxdy[i][Ztype] == Qbeam ? 
        [Qbeam,x,y] : 
    (dxdy[i][Ztype] == Qfork ? 
        let (x2 = sum_fwd(dxdy,i-1,Zdx) + origin[Zdx])
        let (y2 = sum_fwd(dxdy,i-1,Zdy) + origin[Zdy])
//echo("NODE FORK",x2=x2,y2=y2)
    [Qfork, gen_nodes(dxdy[i][1],origin=[Qbeam,x2,y2]),
            gen_nodes(dxdy[i][2],origin=[Qbeam,x2,y2]) ] : 
    [Qskip,0,0,0,0] )) ];  

// generate a loads tree from the dna tree, to make the programming easier to read
function loads_to_beams(dna) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (dna[i][Ztype] == Qload ? 
        let (fx = dna[i][Zfx])
        let (fy = dna[i][Zfy])
        let (moment = dna[i][Zm])
        [Qskip,fx,fy,moment] : 
    (dna[i][Ztype] == Qfork ? 
     [Qfork, loads_to_beams(dna[i][1]),loads_to_beams(dna[i][2])] :
       [Qload,0,0,0,0] ) ) ];

// recursive function to spread the external forces and moments from tail of tree to root
function spread_ext_loads(ext_loads) =
    let (n = len(ext_loads))
    //echo("SPREAD EXT LOADS",n=n)
    [ for (i=[0:1:n-1]) (ext_loads[i][Ztype] == Qload ? 
    // echo(Qload,i=i,sum_tail2(ext_loads,i,Zfx))
    [Qload,sum_tail2(ext_loads,i,Zfx),sum_tail2(ext_loads,i,Zfy),sum_tail2(ext_loads,i,Zm)]: 
    (ext_loads[i][Ztype] == Qfork ? 
    [Qfork, spread_ext_loads(ext_loads[i][1]), spread_ext_loads(ext_loads[i][2])] :
    //echo(Qskip,i=i)
    [Qskip,ext_loads[i][Zfx],ext_loads[i][Zfy],ext_loads[i][Zm]] ) ) ];

// recursive function to rotate the internal forces from a global system to a beam-local system.  Moments are copied
function rotate_int_loads(int_loads,beam_angles) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) (int_loads[i][Ztype] == Qload ? 
        let (fx = int_loads[i][Zfx])
        let (fy = int_loads[i][Zfy])
        let (ang = -beam_angles[i])
//echo(i=i,fx=fx,f=fy,ang=ang)
        [Qload, rot_x(fx,fy,ang) , rot_y(fx,fy,ang) ,int_loads[i][Zm] ]: 
    (int_loads[i][Ztype] == Qfork ? 
    [Qfork, rotate_int_loads(int_loads[i][1],beam_angles[i][1]), rotate_int_loads(int_loads[i][2],beam_angles[i][2])] :  [Qskip,0,0,0,0] ) ) ];

// calculate moments due to forces on current beam from  beam(s) down the tree
function moments_due_to_forces(int_loads, dna) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) (int_loads[i][Ztype] == Qload ? 
        (int_loads[i+1][Ztype] == Qfork ? 
                // At a fork need to reach down into the tree
                let (fy1 = int_loads[i+1][1][0][Zfy])
                let (dx1 = dna[i+1][1][0][Zlen])
                let (fy2 = int_loads[i+1][2][0][Zfy])
                let (dx2 = dna[i+1][2][0][Zlen])
                [Qload, fy1*dx1 + fy2*dx2 ] 
    :
                let (fy = int_loads[i+1][Zfy])
                let (dx = dna[i+1][Zlen])
                let (m = (fy*dx == undef ? 0 : fy*dx))
    //echo(Qload,i=i,fy=fy,dx=dx)
                [Qload, m ] 
            ) : 
            (int_loads[i][Ztype] == Qfork ? 
        [Qfork, moments_due_to_forces(int_loads[i][1], dna[i][1]), moments_due_to_forces(int_loads[i][2], dna[i][2])] :
    [Qskip,0,0,0,0] ) ) ];

// recursive function to sum moments from tail of tree to root
function sum_moments(moments) =
    let (n = len(moments))
    [ for (i=[0:1:n-1]) (moments[i][Ztype] == Qload ? 
        [Qload, sum_tail2(moments,i,1) ]: 
    (moments[i][Ztype] == Qfork ? [Qfork, sum_moments(moments[i][1]), sum_moments(moments[i][2])] :
    [Qskip,moments[i][1],0,0,0] ) ) ];

// recursive function to add moments to loads
function add_moments_to_loads(loads,more_moments) =
    let (n = len(loads))
    [ for (i=[0:1:n-1]) (loads[i][Ztype] == Qload ? 
        let (fx = loads[i][Zfx])
        let (fy = loads[i][Zfy])
        let (moment = loads[i][Zm])
        let (moment2 = more_moments[i][1])
        [Qload,fx,fy,moment+moment2] : 
    (loads[i][Ztype] == Qfork ? 
     [Qfork, add_moments_to_loads(loads[i][1],more_moments[i][1]),add_moments_to_loads(loads[i][2],more_moments[i][2])] :
       [Qskip,0,0,0,0] ) ) ];
    
// recursion - find the sum of the values in a vector (array) by calling itself
// from the start (or s'th element) to the i'th element - remember elements are zero based
// From the OpenSCAD user manual on recursive calls
//function sumv(v,i,s=0) = (i==s ? v[i] : v[i] + sumv(v,i-1,s));
    
// select a sub vector from a 2d vector
//function select(vector,index) = [ for (i = [0:1:len(vector)-1]) vector[i][index] ];

function rot_x (x,y,a) = x*cos(a)-y*sin(a);

function rot_y (x,y,a) = x*sin(a)+y*cos(a);
    
function compute_results(dna,loads,I,area,E,Ftu,rho) = 
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (loads[i][Ztype] == Qload ? 
        let (L = dna[i][Zlen])   // FUTURE, ADJUST BASED ON X LOAD
        let (fx = loads[i][Zfx])  // axial load
        let (fy = loads[i][Zfy])  // bending load
        let (moment = loads[i][Zm])
        let (Iz = I[i])
        let (a = area[i])
        let (bt = beam_type(fy,moment,L))
        let (K = spring_rate(bt,Iz,L,E))       // force per radian
        let (cr = characteristic_radius(bt))
        let (theta = spring_angle(bt,K,fy,moment,cr,L)) // degrees
        let (t_rad = theta * PI / 180)               // radians
        let (theta_end = end_angle(bt,theta))           // degrees
        let (a = a_position(L,cr,theta))
        let (b = b_position(L,cr,theta))
        let (m_total = moment + fx * b + fy * a)
        let (c = dna[i][Zthk] / 2)              // half thickness
        let (stressmax = m_total*c/Iz + fx/a)
        let (stressmin = -m_total*c/Iz + fx/a)
        let (ms = (abs(stressmax) > abs(stressmin) ? 1-abs(stressmax)/Ftu : 1-abs(stressmin)/Ftu))
        let (energy = 0.5* K * (t_rad*t_rad))  // PE = 1/2 * K * x ^2
        let (weight = a*L*rho)
//echo(n=n,fx=fx,fy=fy,m_total=m_total,stressmax=stressmax,stressmin=stressmin,energy=energy)
        [Qresult,bt,cr,K,theta,theta_end,a,b,stressmax,stressmin,energy,weight,ms] : 
    (loads[i][Ztype] == Qfork ? 
     [Qfork, compute_results(dna[i][1],loads[i][1],I[i][1],area[i][1],E,Ftu,rho),
             compute_results(dna[i][2],loads[i][2],I[i][2],area[i][2],E,Ftu,rho)] :
       [Qskip,0,0,0,0,0,0,0,0] ) ) ];

// get beam type from forces 
function beam_type(fy=1,m=.2,L=1) = 
    ((abs(fy*L)>abs(m) )? Zvertforce : Zendmoment );  // 0 use A.1.2 vertical force constants,  1 use A.1.5 moment
    
function characteristic_radius(bt) = 
    (bt == Zvertforce ? 0.85 : 0.7346);
    
function spring_rate(bt,i,len,E) = 
    (bt == Zvertforce ? 2.258*E*i/len : 1.5164*E*i/len);
    
function spring_angle(bt,K,fy,moment,cr,L) =
    let (rad_deg = 180/PI)
    (bt == Zvertforce ? rad_deg*(fy/K)*cr*L : rad_deg*(moment/K + (fy/K)*cr*L) );
    
function end_angle(bt,angle) = 
    (bt == Zvertforce ? angle*1.24 : angle*1.5164);

function a_position(L,cr,theta) = L*(1-cr*(1-cos(theta)));
function b_position(L,cr,theta) = cr*L*sin(theta);

// convert the margin of safety into a red to green value
function val_red(i) = i < .5 ? 1 : i > 1 ? 0 : 2-i*2 ;

function val_green(i) = i < 0 ? 0 : i > .5 ? 1 : i*2 ;

// recursive module that draws the deformed beam.
module draw_beam_deformed(dna,angles,results,idx = 0) {
    elem_type = dna[idx][Ztype];
    if (elem_type == Qbeam) {
//echo(idx=idx,elem_type=elem_type,z_ang=z_ang,end_ang=end_ang,a=a,b=b);
        L = dna[idx][Zlen];
        t = dna[idx][Zthk];
        w = dna[idx][Zw];
        dna_ang = dna[idx][Zang];
        z_ang = angles[idx];
        cr = results[idx][Zrad];
        babyL = L*(1-cr);
        end_ang = results[idx][Zthetaend];
        a = results[idx][Za];
        b = results[idx][Zb];
        ms = results[idx][Zms];
        // draw the two beam segments 
        color ([val_red(ms),val_green(ms),0.2]) linear_extrude(height=w,center=true) hull() { 
            rotate([0,0,dna_ang]) translate([babyL,0,0]) circle(d=t,$fn=16);
            circle(d=t,$fn=16);  // ZERO,ZERO
        }
        color ([val_red(ms),val_green(ms),0]) linear_extrude(height=w,center=true) hull() { 
            rotate([0,0,dna_ang]) translate([babyL,0,0]) circle(d=t,$fn=16);
            rotate([0,0,dna_ang]) translate([a,b,0]) circle(d=t,$fn=16);
        }
        // Recursive call generating the next beam
        rotate([0,0,dna_ang]) translate([a,b,0]) rotate([0,0,end_ang]) 
                draw_beam_deformed(dna,angles,results,idx + 1);
    } else if (elem_type == Qfork ) {
        // FORK, PROCESS TWO BEAM SEGMENTS
        // Recursive call generating the first fork
        draw_beam_deformed(dna[idx][1],angles[idx][1],results[idx][1]);
        // Recursive call generating the second fork
        draw_beam_deformed(dna[idx][2],angles[idx][2],results[idx][2]);
    } else if (elem_type == Qload) {
        draw_beam_deformed(dna,angles,results,idx + 1);
    }
    // Note: that an undefined causes the recursion to stop
}

// recursive function to add deflected angles to initial angles
function add_angles(initial,results,prior_ang = 0) =
    let (n = len(initial))
    [ for (i=[0:1:n-1]) (results[i][Ztype] == Qresult ? 
        let (ang1 = initial[i])
        let (ang2 = (i==0 ? prior_ang : sum_fwd(results,i-1,Zthetaend) + prior_ang ))
//echo(i=i,ang1=ang1,ang2=ang2)
        ang1+ang2  : 
    (results[i][Ztype] == Qfork ? 
        let (ang3 = sum_fwd(results,i-1,Zthetaend) + prior_ang )
//echo("FORK",i=i,ang3=ang3)
     [Qfork, add_angles(initial[i][1],results[i][1],ang3),
             add_angles(initial[i][2],results[i][2],ang3)] :
       Qskip ) )  ];

