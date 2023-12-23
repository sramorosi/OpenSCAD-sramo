// Large Displacement Beam, Modules and Functions
include <LDB_Indexes.scad>
//use <ME_lib.scad>
use <../MAKE3-Arm/openSCAD-code/ME_lib.scad>

// Display intermediate load steps?
Display_steps = false;
// Scale of Force & Moment Display
force_scale = 0.2;
// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI), PLA=340000,PETG=300000,Polycar=320000
E = 340000;
// ~Stress level at which the part will fail (PSI)
Failure_Stress = 10000;
// This could be tensile failure, compression failure, bending, etc.
// material density (lb per inch^3)
density = 0.045;
STEPS = 4;  // A GOOD NUMBER OF STEPS IS DEPENDANT ON THE PROBLEM

BEAM1 = [[Qbeam,10,1,3,0],[Qbeam,10,1,3,0],[Qbeam,10,1,3,0],[Qload,0,200,0]];
draw_beam_undeformed(BEAM1);
Do_Analysis(BEAM1,force_scale*.5,Display_steps,E,Failure_Stress,density,steps=STEPS);

// recursive module that draws the undeformed beam.
module draw_beam_undeformed(LDBdef,idx = 0) {
// LDBdef (LDB definition)
// [[beam, len, thk, width, angle],...]

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
    } else if (elem_type == Qfork ) {
        // FORK, PROCESS TWO BEAM SEGMENTS
        // Recursive call generating the first fork
        draw_beam_undeformed(LDBdef[idx][1]);
        // Recursive call generating the second fork
        draw_beam_undeformed(LDBdef[idx][2]);
    } else if (elem_type == Qload) { // skip and go to next
                draw_beam_undeformed(LDBdef,idx + 1);
    }  
}

// recursive module that draws the deformed beam.
module draw_beam_deformed(LDBdef,results,idx = 0) {
    elem_type = LDBdef[idx][Ztype];
    if (elem_type == Qbeam) {  // Note: undefined causes the recursion to stop
//echo(idx=idx,elem_type=elem_type,z_ang=z_ang,end_ang=end_ang,a=a,b=b);
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
    } else if (elem_type == Qfork ) {
        // FORK, PROCESS TWO BEAM SEGMENTS
        // Recursive call generating the first fork
        draw_beam_deformed(LDBdef[idx][1],results[idx][1]);
        // Recursive call generating the second fork
        draw_beam_deformed(LDBdef[idx][2],results[idx][2]);
    } else if (elem_type == Qload) {
        draw_beam_deformed(LDBdef,results,idx + 1);
    }
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
        translate(origin) rotate ([0,0,z_rot]) torque_arrow([0,0,0],mag=moment*0.008*scale);
    if (out) echo("GROUND REACTIONS ",fx=rot_x(fx,fy,z_rot),fy=rot_y(fx,fy,z_rot),moment=moment);
}
    

// recursive function to generate moment of inertia (Iz) of each beam
function gen_Iz(LDBdef) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) (LDBdef[i][Ztype] == Qbeam ? 
        Iz_func(LDBdef[i][Zw],LDBdef[i][Zthk]) : 
    (LDBdef[i][Ztype] == Qfork ? 
    [Qfork, gen_Iz(LDBdef[i][1]),gen_Iz(LDBdef[i][2])] : Qskip ) ) ];
    
//  simple function to calculate moment of inertia about Z axis
function Iz_func(w=1,t=.1) = ((w*t*t*t)/12);
    
// recursive function to generate cross section Area of each beam
function gen_Area(LDBdef) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) (LDBdef[i][Ztype] == Qbeam ? 
        LDBdef[i][Zthk]*LDBdef[i][Zw] : 
     (LDBdef[i][Ztype] == Qfork ? 
    [Qfork, gen_Area(LDBdef[i][1]),gen_Area(LDBdef[i][2])] : Qskip )) ];

// sum angles along segments to get global angles.
function global_angles(LDBdef,prior_ang=0) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) (LDBdef[i][Ztype] == Qbeam ? 
    let (new_sum = sum_fwd(LDBdef,i,Zang)) 
//echo("ANGLES",i=i,prior_ang=prior_ang, new_sum=new_sum)
    new_sum+prior_ang : 
    (LDBdef[i][Ztype] == Qfork ? 
    let (new_prior_ang = sum_fwd(LDBdef,i-1,Zang) + prior_ang)
     [Qfork, global_angles(LDBdef[i][1],new_prior_ang),global_angles(LDBdef[i][2],new_prior_ang)] :
    Qskip ) ) ];
    
// recursive forward summation function to sum "thing"
// from the start (or s'th element) to the i'th element - remember elements are zero based
function sum_fwd(LDBdef,i,thing,s=0) = 
    let (val = LDBdef[i][thing])
    (i==s ? val : val + sum_fwd(LDBdef,i-1,thing,s));

// recursive tail summation function to sum "thing"  AND ASSUMING A TREE (NESTED VECTOR)
// from the i'th element to the last (or n'th element)
function sum_tail2(LDBdef,i,thing,vec_type=Qload) = 
    let (val = LDBdef[i][thing])
    let (type = LDBdef[i][Ztype]) 
    (type==Qfork ?  sum_tail2(LDBdef[i][1],1,thing,vec_type) + 
                    sum_tail2(LDBdef[i][2],1,thing,vec_type) : 
//echo("SUM ",i=i,type=type,val=val)
        ((type==vec_type || type==Qskip) ? 
                val + sum_tail2(LDBdef,1+i,thing,vec_type) : 
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
function gen_dxdy_undeformed(LDBdef,angles) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) 
    ( LDBdef[i][Ztype] == Qbeam ? 
        let (L = LDBdef[i][Zlen])
        let (ang = angles[i]) 
        let (x = L*cos(ang))
        let (y = L*sin(ang))
//echo(i=i,L=L,ang=ang)
        [Qbeam,x,y] : 
    (LDBdef[i][Ztype] == Qfork ? 
    [Qfork, gen_dxdy_undeformed(LDBdef[i][1],angles[i][1]),gen_dxdy_undeformed(LDBdef[i][2],angles[i][2])] : 
    [Qskip,0,0,0,0] )) ];  

// recursive function to generate DX AND DY of deformed beam
function gen_dxdy_deformed(LDBdef,results,angles) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) 
        let (x = results[i][Za])
        let (y = results[i][Zb])
        let (ang = angles[i]) 
    ( LDBdef[i][Ztype] == Qbeam ? 
        [Qbeam,rot_x(x,y,ang),rot_y(x,y,ang)] : 
    (LDBdef[i][Ztype] == Qfork ? 
    [Qfork, gen_dxdy_deformed(LDBdef[i][1],results[i][1],angles[i][1]),
        gen_dxdy_deformed(LDBdef[i][2],results[i][2],angles[i][2])] : 
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

// generate a loads tree from the LDBdef tree, to make the programming easier to read
function loads_to_beams(LDBdef) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) (LDBdef[i][Ztype] == Qload ? 
        let (fx = LDBdef[i][Zfx])
        let (fy = LDBdef[i][Zfy])
        let (moment = LDBdef[i][Zm])
        [Qskip,fx,fy,moment] :  
    (LDBdef[i][Ztype] == Qfork ? 
     [Qfork, loads_to_beams(LDBdef[i][1]),loads_to_beams(LDBdef[i][2])] :
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
function moments_due_to_forces(loads, LDBdef, angles) = 
    let (n = len(loads))
    [ for (i=[0:1:n-1]) (loads[i][Ztype] == Qload ? 
        (loads[i+1][Ztype] == Qfork ? 
                // At a fork need to reach down into the tree
                let (fy1 = loads[i+1][1][0][Zfy])
                let (dx1 = LDBdef[i+1][1][0][Zlen])
                let (fy2 = loads[i+1][2][0][Zfy])
                let (dx2 = LDBdef[i+1][2][0][Zlen])
                [Qload, fy1*dx1 + fy2*dx2 ] 
    :
                let (fx = loads[i+1][Zfx])
                let (fy = loads[i+1][Zfy])
                let (ang = angles[i])
                let (dx = LDBdef[i+1][Zlen]*cos(ang))
                let (dy = LDBdef[i+1][Zlen]*sin(ang))
                let (m = (fy*dx == undef ? 0 : fy*dx-fx*dy))
    //echo(i=i,ang=ang,m=m)
                [Qload, m ] 
            ) : 
            (loads[i][Ztype] == Qfork ? 
        [Qfork, moments_due_to_forces(loads[i][1], LDBdef[i][1]), moments_due_to_forces(loads[i][2], LDBdef[i][2])] :
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
    
function compute_results(LDBdef,loads,I,area,E,failure_stress,density) = 
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) (loads[i][Ztype] == Qload ? 
        let (L = LDBdef[i][Zlen])   // FUTURE, ADJUST L BASED ON X LOAD
        let (fx = loads[i][Zfx])  // axial load
        let (fy = loads[i][Zfy])  // bending load
        let (moment = loads[i][Zm])  // moment on beam
        let (Iz = I[i])
        let (Area = area[i])
        let (bt = beam_type(fy,moment,L))
        let (K = spring_rate(bt,Iz,L,E))//force per radian // DEAD CODE
        let (cr = characteristic_radius(bt))// DEAD CODE
        let (theta = spring_angle(bt,fy,moment,L,E,Iz)) // degrees.  THETA NEEDS TO CONVERGE.
        let (t_rad = theta * PI / 180)      // radians
        let (theta_end = end_angle(bt,theta)) // degrees
        let (a = a_position(L,cr,theta))
        let (b = b_position(L,cr,theta))
        let (m_total = moment - fx * b + fy * a)
        let (m_error =  moment - m_total)   // MOMENT BALANCE CHECK
        let (c = LDBdef[i][Zthk] / 2)              // half thickness
        let (stressmax = m_total*c/Iz + fx/a)
        let (stressmin = -m_total*c/Iz + fx/a)
        let (ms = (abs(stressmax) > abs(stressmin) ? failure_stress/abs(stressmax)-1 : failure_stress/abs(stressmin)-1))
        let (energy = 0.5* K * (t_rad*t_rad))  // PE = 1/2 * K * x ^2
        let (weight = Area*L*density)
echo(i=i,theta=theta,fx=fx,fy=fy,moment=moment,a=a,b=b,m_total=m_total)
        [Qresult,bt,cr,K,theta,theta_end,a,b,stressmax,stressmin,energy,weight,ms,fx,fy,m_total,m_error] : 
    (loads[i][Ztype] == Qfork ? 
     [Qfork, compute_results(LDBdef[i][1],loads[i][1],I[i][1],area[i][1],E,failure_stress,density),
             compute_results(LDBdef[i][2],loads[i][2],I[i][2],area[i][2],E,failure_stress,density)] :
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
function count_branches(LDBdef,i=0,count=0) =
    let (type = LDBdef[i][Ztype]) 
    (type == Qbeam || type == Qload || type == QdispX ? count_branches(LDBdef,i+1,1)  : 
        (type == Qfork ? ( count_branches(LDBdef[i][1],0,1) + 
                        count_branches(LDBdef[i][2],0,1) + count) : count ) );
    
// recursive function to count the tree depth
function tree_depth(LDBdef,i=0,depth=0) =
    let (type = LDBdef[i][Ztype]) 
    (type == Qbeam || type == Qload || type == QdispX ? tree_depth(LDBdef,i+1,depth)  : 
        (type == Qfork ? ( max(tree_depth(LDBdef[i][1],0,0), 
                        tree_depth(LDBdef[i][2],0,0)) + 1) : depth ) );

// recursive function to count the number of beams and check data
function count_beams(LDBdef,i=0,count=0) =
    let (type = LDBdef[i][Ztype]) 
    (type == Qbeam ? 
        let (length=(LDBdef[i][Zlen] == 0 ? echo("** FOUND ZERO LENGTH BEAM **",i=i):0))
        let (thick=(LDBdef[i][Zthk] == 0 ? echo("** FOUND ZERO THICKNESS BEAM **",i=i):0))
        let (width=(LDBdef[i][Zw] == 0 ? echo("** FOUND ZERO WIDTH BEAM **",i=i):0))
//echo("BEAM",i=i,count=count)
        count_beams(LDBdef,i+1,count+1) 
    : (type == Qfork ?(count_beams(LDBdef[i][1],0,1)+count_beams(LDBdef[i][2],0,1)) 
    : (type == undef ? count : count_beams(LDBdef,i+1,count)) ) );

// recursive function to count the number of loads and check data
function count_loads(LDBdef,i=0,count=0) =
    let (type = LDBdef[i][Ztype]) 
    (type == Qload ? 
        let (Fx=LDBdef[i][Zfx])
        let (Fy=LDBdef[i][Zfy])
        let (m=LDBdef[i][Zm])
        let (sum=(Fx+Fy+m == 0 ? echo("** FOUND ZERO LOAD LOAD **",i=i):0))
        count_loads(LDBdef,i+1,count+1)  
    : (type == Qfork ? ( count_loads(LDBdef[i][1],0,count) + 
                        count_loads(LDBdef[i][2],0,count)) 
    : (type == undef ? count : count_loads(LDBdef,i+1,count) ) ) );

// recursive function to find load target displacement and compare to actual displacement
function check_displacement_target(LDBdef,def_nodes,init_nodes) =
    let (n = len(LDBdef))
    [ for (i=[0:1:n-1]) (LDBdef[i][Ztype] == Qload ? 
        let (x_targ = LDBdef[i][Ztargetx])
        let (x_node = def_nodes[i][Zdx])
        let (x_err = x_targ-x_node)
        let (fx = LDBdef[i][Zfx])
        let (x_init = init_nodes[i][Zdx])
        let (Kx = fx/(x_node-x_init))
        let (NEW_fx = Kx*(x_targ-x_init))
echo(i=i,x_targ=x_targ,x_node=x_node,x_err=x_err,fx=fx,x_init=x_init,Kx=Kx,NEW_fx=NEW_fx)
        let (y_targ = LDBdef[i][Ztargety])
        let (y_node = def_nodes[i][Zdy])
        let (y_err = y_targ-y_node)
        let (fy = LDBdef[i][Zfy])
        let (y_init = init_nodes[i][Zdy])
        let (Ky = fy/(y_node-y_init))
        let (NEW_fy = Ky*(y_targ-y_init))
echo(i=i,y_targ=y_targ,y_node=y_node,y_err=y_err,fy=fy,y_init=y_init,Ky=Ky,NEW_fy=NEW_fy)
        [Qdisp,x_targ,y_targ,x_node,y_node] : 
    (LDBdef[i][Ztype] == Qfork ? 
     [Qfork, 
    check_displacement_target(LDBdef[i][1],def_nodes[i+1][1],init_nodes[i+1][1]),
    check_displacement_target(LDBdef[i][2],def_nodes[i+1][2],init_nodes[i+1][2])] :
       [Qskip,0,0,0,0] ) ) ];

module Do_Analysis(LDB_DEF,f_scale=1,Display_steps=true,E=300000,Failure_Stress=5000,density=0.05,origin=[0,0,0],steps=4) {
    echo("**********");
    echo("LARGE DISPLACEMENT 2D BEAM ANALYSIS BASED ON COMPLIANT MECHANISM PRBM");
    echo(E=E,Failure_Stress=Failure_Stress,density=density);
    
    // perform data checks
    num_branches = count_branches(LDB_DEF);
    if (num_branches > 0 ) {
        echo("NUMBER OF BRANCHES IS ",num_branches," NUMBER OF FORKS IS ",(num_branches-1)/2);
        echo("TREE DEPTH IS ",tree_depth(LDB_DEF));
        num_beams = count_beams(LDB_DEF);
        num_loads = count_loads(LDB_DEF);
        if (num_beams > 0 && num_loads > 0) {
            echo("NUMBER OF BEAMS IS ",num_beams," NUMBER OF LOADS IS ",num_loads);
    
            // Generate Beam Inertias and Cross Section Areas
            Iz = gen_Iz(LDB_DEF);
            Area = gen_Area(LDB_DEF);
            // Generate internal Global forces from external forces
            // Move external loads from the LDB_DEF to a separate loads-only vector
            ext_loads_temp = loads_to_beams(LDB_DEF);
            // Spread the external Global forces from the tails to the root
            // Moments don't include force-moments at this time
            initial_loads = spread_ext_loads(ext_loads_temp);
            // Generate Beam GLOBAL ANGLES, undeformed
            //echo(ext_loads_temp=ext_loads_temp);
            //echo(initial_loads=initial_loads);
            beam_angles = global_angles(LDB_DEF);
            //    echo("INITIAL VECTORS, THESE DON'T CHANGE");
            //    echo (LDB_DEF=LDB_DEF);
            //    echo(Iz=Iz);
            //    echo(Area=Area);
            //    echo(initial_loads=initial_loads);

            // This is only needed to create a results array for the next step
            results0 = compute_iteration(LDB_DEF,Iz,Area,E,Failure_Stress,density,initial_loads, beam_angles ,scale=0); 
            InitialNodes = compute_nodes(LDB_DEF,results0,beam_angles);
            
            FinalResults = compute_steps(LDB_DEF,Iz,Area,E,Failure_Stress,density,initial_loads, beam_angles,results0,STEPS,n=steps);
            
            //echo("TOTAL MOMENT ERROR = ",totalMomentError(FinalResults,3));
            
            FinalBeamAngles = add_angles(beam_angles,FinalResults);
            FinalNodes = compute_nodes(LDB_DEF,FinalResults,FinalBeamAngles);
            echo(FinalNodes = FinalNodes);
            translate(origin) draw_loads(FinalNodes,ext_loads_temp,f_scale,"yellow",0);

            draw_ground_reactions(FinalResults ,f_scale,origin ,LDB_DEF[0][Zang]);

            translate(origin) union () draw_beam_deformed(LDB_DEF,FinalResults);
            //echo(FinalNodes=FinalNodes);
            echo("X MAX ",max_tree(FinalNodes,0,Zdx)+origin[0],
                 "  X MIN ",min_tree(FinalNodes,0,Zdx)+origin[0]);
            echo("Y MAX ",max_tree(FinalNodes,0,Zdy)+origin[1],
                 "  Y MIN ",min_tree(FinalNodes,0,Zdy)+origin[1]);
            //echo(FinalResults=FinalResults);
            echo("STRESS MAX ",max_tree(FinalResults,0,Zstressmax),
                 "  STRESS MIN ",min_tree(FinalResults,0,Zstressmin));
            echo("MIN Margin of Safety ",min_tree(FinalResults,0,Zms),
                 " MAX MS ",max_tree(FinalResults,0,Zms));
            echo("WEIGHT ",sum_tail2(FinalResults,0,Zweight,Qresult),
                 " ENERGY ",sum_tail2(FinalResults,0,Zenergy,Qresult));

        } else echo("**NUMBER OF BEAMS OR LOADS IS ZERO, TERMINATING**"); 
    } else echo("**NUMBER OF BRANCHES IS ZERO, TERMINATING**");
}

// Recursive function to perform analysis in steps (n)
function compute_steps(LDB_DEF,Iz,Area,E,Failure_Stress,density,loads,beam_angles,results,STEPS=4,n=4) =
    let (newAngleVec = add_angles(beam_angles,results))
echo(n=n," load scale = ",(((STEPS+1)-n)/STEPS),newAngleVec=newAngleVec)
    let (newResults = compute_iteration(LDB_DEF,Iz,Area,E,Failure_Stress,density,loads,newAngleVec,((STEPS+1)-n)/STEPS))
    ( n==1 ? newResults : compute_steps(LDB_DEF,Iz,Area,E,Failure_Stress,density,loads,newAngleVec,newResults,STEPS,n-1) );

function compute_iteration(LDB_DEF,Iz,Area,E,Failure_Stress,density,loads,beam_angles,scale=1) =     
    // Compute an iteration (to update moments due to forces)
    // scale internal loads
    let (loads_scaled = scale_int_loads(loads,scale))
    // Convert internal global forces to beam-local forces 
    let (loads_local = rotate_int_loads(loads_scaled,beam_angles))
//echo(beam_angles=beam_angles)
    // Calculate moments due to forces
    let (force_mom_temp = moments_due_to_forces(loads_local, LDB_DEF, beam_angles))
    // Sum moments due to forces, starting at tail
    let (force_moments = sum_moments(force_mom_temp))
    // Add moments-due-to-forces with internal loads
    let (NEW_loads_local = add_moments_to_loads(loads_local,force_moments))
    //let (INTERNALMOMENTS = sum_moments(loads,FinalResults,STEPS))
    //        echo(INTERNALMOMENTS=INTERNALMOMENTS);
    // call function compute results
    compute_results(LDB_DEF,NEW_loads_local,Iz,Area,E,Failure_Stress,density);

function compute_nodes(LDB_DEF,results,angles) =
// combine gen_dxdy_deformed with gen_nodes into single function
    let (dxdy=gen_dxdy_deformed(LDB_DEF,results,angles))
    gen_nodes(dxdy);
    
// recursive function to sum moments in the beam
function totalMomentError(results,n) =
    echo(n=n," loads Error ",results[n-1][ZMerror])
    ( n==1 ? 0 : (results[n-1][ZMerror] + totalMomentError(results,n-1)));
