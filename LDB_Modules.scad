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
module draw_loads(nodes,loads,scale=1,node_color="blue",idx = 0) {
    elem_type = loads[idx][Ztype];
    if (elem_type == Qload) {
        // do nothing, Recursive call
        x = nodes[idx][Zdx];
        y = nodes[idx][Zdy];
        // draw the beam segment
        color(node_color) linear_extrude(height=2,center=true) hull() { 
            translate([x,y,0]) circle(d=.1,$fn=16); }
        draw_loads(nodes,loads,scale,node_color,idx + 1);
    } else if (elem_type == Qskip) {
        x = nodes[idx][Zdx];
        y = nodes[idx][Zdy];
        fx = loads[idx][Zfx];
        fy = loads[idx][Zfy];
        moment = loads[idx][Zm];
        // draw the beam segment
        color(node_color) linear_extrude(height=2,center=true) hull() { 
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

// module that draws the loads at ground (not recursive)
module draw_ground_reactions(results,scale=1,origin=[0,0,0],z_rot=0,out=true) {
    fx = -results[0][ZPx];
    fy = -results[0][ZPy];
    moment = -results[0][ZPm];
    // draw forces and torques
    fmag = sqrt(fx*fx + fy*fy);
    if (abs(fmag)>0.1) color ("red") 
        translate(origin) rotate ([0,0,z_rot]) force_arrow([0,0,0],[fx,fy,0],mag=fmag*scale);
    if (abs(moment)>0.1) color ("blue")
        translate(origin) rotate ([0,0,z_rot]) torque_arrow([0,0,0],mag=moment*scale);
    if (out) echo("GROUND REACTIONS ",fx=rot_x(fx,fy,z_rot),fy=rot_y(fx,fy,z_rot),moment=moment);
}
    

// recursive function to generate moment of inertia (Iz) of each beam
function gen_Iz(dna) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (dna[i][Ztype] == Qbeam ? 
        Iz_func(dna[i][Zw],dna[i][Zthk]) : 
    (dna[i][Ztype] == Qfork ? 
    [Qfork, gen_Iz(dna[i][1]),gen_Iz(dna[i][2])] : Qskip ) ) ];
    
//  simple function to calculate moment of inertia about Z axis
function Iz_func(w=1,t=.1) = ((w*t*t*t)/12);
    
// recursive function to generate cross section Area of each beam
function gen_Area(dna) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (dna[i][Ztype] == Qbeam ? 
        dna[i][Zthk]*dna[i][Zw] : 
     (dna[i][Ztype] == Qfork ? 
    [Qfork, gen_Area(dna[i][1]),gen_Area(dna[i][2])] : Qskip )) ];

// sum angles along segments to get global angles.
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
    
// recursive forward summation function to sum "thing"
// from the start (or s'th element) to the i'th element - remember elements are zero based
function sum_fwd(dna,i,thing,s=0) = 
    let (val = dna[i][thing])
    (i==s ? val : val + sum_fwd(dna,i-1,thing,s));

// recursive tail summation function to sum "thing"  AND ASSUMING A TREE (NESTED VECTOR)
// from the i'th element to the last (or n'th element)
function sum_tail2(dna,i,thing,vec_type=Qload) = 
    let (val = dna[i][thing])
    let (type = dna[i][Ztype]) 
    (type==Qfork ?  sum_tail2(dna[i][1],1,thing,vec_type) + 
                    sum_tail2(dna[i][2],1,thing,vec_type) : 
//echo("SUM ",i=i,type=type,val=val)
        ((type==vec_type || type==Qskip) ? 
                val + sum_tail2(dna,1+i,thing,vec_type) : 
                (val==undef ? 0 : val )));

// recursive function to find maximum "thing"  ASSUMING A TREE (NESTED VECTOR)
function max_tree(tree,i=0,thing) = 
    let (val = tree[i][thing])
    let (type = tree[i][Ztype]) 
//echo("MAX ",i=i,type=type,val=val)
    (type==Qfork ?  max(max_tree(tree[i][1],1,thing),max_tree(tree[i][2],1,thing)) : 
        (type==Qskip ? max(-9999999,max_tree(tree,1+i,thing)) :
        (type==undef ? -99999999 : max(val,max_tree(tree,1+i,thing)))));

// recursive function to find minimum "thing"  ASSUMING A TREE (NESTED VECTOR)
function min_tree(tree,i=0,thing) = 
    let (val = tree[i][thing])
    let (type = tree[i][Ztype]) 
//echo("MIN ",i=i,type=type,val=val)
    (type==Qfork ?  min(min_tree(tree[i][1],1,thing),min_tree(tree[i][2],1,thing)) : 
        (type==Qskip ? min(9999999,min_tree(tree,1+i,thing)) :
        (type==undef ? 99999999 : min(val,min_tree(tree,1+i,thing)))));

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
//echo(Qload,i=i,sum_tail2(ext_loads,i,Zfy))
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

// recursive function to scale the internal forces and moments
function scale_int_loads(int_loads,scale=1) = 
    let (n = len(int_loads))
    [ for (i=[0:1:n-1]) (int_loads[i][Ztype] == Qload ? 
        let (fx = int_loads[i][Zfx] * scale)
        let (fy = int_loads[i][Zfy] * scale)
        let (moment = int_loads[i][Zm] * scale)
        [Qload, fx,fy,moment ]: 
    (int_loads[i][Ztype] == Qfork ? 
    [Qfork, scale_int_loads(int_loads[i][1],scale), scale_int_loads(int_loads[i][2],scale)] :  [Qskip,0,0,0,0] ) ) ];

// calculate moments due to forces on current beam from  beam(s) down the tree
function moments_due_to_forces(loads, dna, angles) = 
    let (n = len(loads))
    [ for (i=[0:1:n-1]) (loads[i][Ztype] == Qload ? 
        (loads[i+1][Ztype] == Qfork ? 
                // At a fork need to reach down into the tree
                let (fy1 = loads[i+1][1][0][Zfy])
                let (dx1 = dna[i+1][1][0][Zlen])
                let (fy2 = loads[i+1][2][0][Zfy])
                let (dx2 = dna[i+1][2][0][Zlen])
                [Qload, fy1*dx1 + fy2*dx2 ] 
    :
                let (fx = loads[i+1][Zfx])
                let (fy = loads[i+1][Zfy])
                let (ang = angles[i+1])
                let (dx = dna[i+1][Zlen]*cos(ang))
                let (dy = dna[i+1][Zlen]*sin(ang))
                let (m = (fy*dx == undef ? 0 : fy*dx-fx*dy))
                [Qload, m ] 
            ) : 
            (loads[i][Ztype] == Qfork ? 
        [Qfork, moments_due_to_forces(loads[i][1], dna[i][1]), moments_due_to_forces(loads[i][2], dna[i][2])] :
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
    
function compute_results(dna,loads,I,area,E,failure_stress,density) = 
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (loads[i][Ztype] == Qload ? 
        let (L = dna[i][Zlen])   // FUTURE, ADJUST L BASED ON X LOAD
        let (fx = loads[i][Zfx])  // axial load
        let (fy = loads[i][Zfy])  // bending load
        let (moment = loads[i][Zm])
        let (Iz = I[i])
        let (Area = area[i])
        let (bt = beam_type(fy,moment,L))
        let (K = spring_rate(bt,Iz,L,E))       // force per radian // DEAD CODE
        let (cr = characteristic_radius(bt))   // DEAD CODE
        let (theta = spring_angle(bt,fy,moment,L,E,Iz)) // degrees.  THETA NEEDS TO CONVERGE.
        let (t_rad = theta * PI / 180)               // radians
        let (theta_end = end_angle(bt,theta))           // degrees
        let (a = a_position(L,cr,theta))
        let (b = b_position(L,cr,theta))
        let (m_total = moment - fx * b + fy * a)
        let (c = dna[i][Zthk] / 2)              // half thickness
        let (stressmax = m_total*c/Iz + fx/a)
        let (stressmin = -m_total*c/Iz + fx/a)
        let (ms = (abs(stressmax) > abs(stressmin) ? failure_stress/abs(stressmax)-1 : failure_stress/abs(stressmin)-1))
        let (energy = 0.5* K * (t_rad*t_rad))  // PE = 1/2 * K * x ^2
        let (weight = Area*L*density)
echo(i=i,theta=theta,fx=fx,fy=fy,moment=moment,a=a,b=b,m_total=m_total)
        [Qresult,bt,cr,K,theta,theta_end,a,b,stressmax,stressmin,energy,weight,ms,fx,fy,m_total] : 
    (loads[i][Ztype] == Qfork ? 
     [Qfork, compute_results(dna[i][1],loads[i][1],I[i][1],area[i][1],E,failure_stress,density),
             compute_results(dna[i][2],loads[i][2],I[i][2],area[i][2],E,failure_stress,density)] :
       [Qskip,0,0,0,0,0,0,0,0,0,0,0,0] ) ) ];

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

// recursive function to count the number of branches
function count_branches(dna,i=0,count=0) =
    let (type = dna[i][Ztype]) 
    (type == Qbeam || type == Qload || type == QdispX ? count_branches(dna,i+1,1)  : 
        (type == Qfork ? ( count_branches(dna[i][1],0,1) + 
                        count_branches(dna[i][2],0,1) + count) : count ) );
    
// recursive function to count the tree depth
function tree_depth(dna,i=0,depth=0) =
    let (type = dna[i][Ztype]) 
    (type == Qbeam || type == Qload || type == QdispX ? tree_depth(dna,i+1,depth)  : 
        (type == Qfork ? ( max(tree_depth(dna[i][1],0,0), 
                        tree_depth(dna[i][2],0,0)) + 1) : depth ) );

// recursive function to count the number of beams and check data
function count_beams(dna,i=0,count=0) =
    let (type = dna[i][Ztype]) 
    (type == Qbeam ? 
        let (length=(dna[i][Zlen] == 0 ? echo("** FOUND ZERO LENGTH BEAM **",i=i):0))
        let (thick=(dna[i][Zthk] == 0 ? echo("** FOUND ZERO THICKNESS BEAM **",i=i):0))
        let (width=(dna[i][Zw] == 0 ? echo("** FOUND ZERO WIDTH BEAM **",i=i):0))
//echo("BEAM",i=i,count=count)
        count_beams(dna,i+1,count+1) 
    : (type == Qfork ?(count_beams(dna[i][1],0,1)+count_beams(dna[i][2],0,1)) 
    : (type == undef ? count : count_beams(dna,i+1,count)) ) );

// recursive function to count the number of loads and check data
function count_loads(dna,i=0,count=0) =
    let (type = dna[i][Ztype]) 
    (type == Qload ? 
        let (Fx=dna[i][Zfx])
        let (Fy=dna[i][Zfy])
        let (m=dna[i][Zm])
        let (sum=(Fx+Fy+m == 0 ? echo("** FOUND ZERO LOAD LOAD **",i=i):0))
        count_loads(dna,i+1,count+1)  
    : (type == Qfork ? ( count_loads(dna[i][1],0,count) + 
                        count_loads(dna[i][2],0,count)) 
    : (type == undef ? count : count_loads(dna,i+1,count) ) ) );

// recursive function to find load target displacement and compare to actual displacement
function check_displacement_target(dna,def_nodes,init_nodes) =
    let (n = len(dna))
    [ for (i=[0:1:n-1]) (dna[i][Ztype] == Qload ? 
        let (x_targ = dna[i][Ztargetx])
        let (x_node = def_nodes[i][Zdx])
        let (x_err = x_targ-x_node)
        let (fx = dna[i][Zfx])
        let (x_init = init_nodes[i][Zdx])
        let (Kx = fx/(x_node-x_init))
        let (NEW_fx = Kx*(x_targ-x_init))
echo(i=i,x_targ=x_targ,x_node=x_node,x_err=x_err,fx=fx,x_init=x_init,Kx=Kx,NEW_fx=NEW_fx)
        let (y_targ = dna[i][Ztargety])
        let (y_node = def_nodes[i][Zdy])
        let (y_err = y_targ-y_node)
        let (fy = dna[i][Zfy])
        let (y_init = init_nodes[i][Zdy])
        let (Ky = fy/(y_node-y_init))
        let (NEW_fy = Ky*(y_targ-y_init))
echo(i=i,y_targ=y_targ,y_node=y_node,y_err=y_err,fy=fy,y_init=y_init,Ky=Ky,NEW_fy=NEW_fy)
        [Qdisp,x_targ,y_targ,x_node,y_node] : 
    (dna[i][Ztype] == Qfork ? 
     [Qfork, 
    check_displacement_target(dna[i][1],def_nodes[i+1][1],init_nodes[i+1][1]),
    check_displacement_target(dna[i][2],def_nodes[i+1][2],init_nodes[i+1][2])] :
       [Qskip,0,0,0,0] ) ) ];

