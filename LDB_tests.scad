// Large Displacement Beam, Test Models
include <LDB_Indexes.scad>
use <LDB_Modules.scad>

// Pick which beam definition to use
ACTIVE_BEAM=1.1; // [1:2seg+M,1.1:FORK,2:4seg+M,2.1:8seg+M(test shape),3:2seg+F,4:4seg+F,5:6seg+F,5.1:8seg+f(test shape),6:U beam,7:Compliant Claw,8:Diagonal,9:Column]

// ECHO values?
output_console = true;

// Force & Moment Display SCALE
force_scale = .1;

// MATERIAL PROPERTIES.  Make this an ARRAY and have a drop down.
// Modulus of Elasticity (PSI)
E = 340000;
// Tensile Ultimate Strength (PSI)
Ftu = 10000;
// material density (lb per cubic inch)
rho = 0.05;

// Beam thickness
t=.15;  
// beam width
w=.8;
// beam angle at fixed end
ang_fixed = 0;

if (ACTIVE_BEAM == 1) {
    // CANTELIVER BEAM WITH MOMENT, 2 SEGMENT
    DNA= [[Qbeam,1.5,t,w,ang_fixed],[Qbeam,1.5,t,w,0],[Qload,0,0,18*$t]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
    
    // multiple unique beams can run at once
    DNA2= [[Qbeam,1.5,t,w,ang_fixed+180],[Qbeam,1.5,t,w,0],[Qload,0,0,-18*$t]];
    
    Do_Analysis(DNA2,force_scale,output_console,E,Ftu,rho);

}
else if (ACTIVE_BEAM == 2) {
    // CANTELIVER BEAM WITH MOMENT, 4 SEGMENT
    ELEM = [[Qbeam,.75,t,w,ang_fixed],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qload,0,0,100*$t]];
    
    Do_Analysis(ELEM,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 2.1) {
    // CANTELIVER BEAM WITH MOMENT, 8 SEGMENT
    L=2;
    DNA = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,0,30*$t]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
    
    translate([0,2.55,0]) cylinder(h=1,r=2.5,center=true,$fn=32);
}

else if (ACTIVE_BEAM == 3) {
    // CANTELIVER BEAM WITH FORCE, 2 SEGMENT
    DNA = [[Qbeam,1.5,t,w,ang_fixed],[Qbeam,1.5,t,w,0],[Qload,0,10,0]];
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 4) {
    // CANTELIVER BEAM WITH FORCE, 4 SEGMENT
    DNA = [[Qbeam,.75,t,w,ang_fixed],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qload,0,10,0]];
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 5) {
    // CANTELIVER BEAM WITH FORCE, 6 SEGMENT
    // TEST CASE: L=3, t=0.15, w=.8, F=10,Roark defection=1.2,min MS=-0.06
DNA = [[Qbeam,.5,t,w,ang_fixed],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,0,10,0] ]; 
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 5.1) {
    // CANTELIVER BEAM WITH FORCE, 8 SEGMENT
    // TEST CASE: L=4, t=0.15, w=.8, F=10,Roark defection=2.79
    
    pts=[[0,0],[0.4961, 0.0626],[0.9639,0.2397],[1.3822,0.5147],[1.7340,0.8715],[2.0056,1.2936],[2.1826,1.7647],[2.2438,2.2685],[2.262,2.7887]];
    
    for (i=[0:8]) translate([pts[i][0],pts[i][1],1]) color("black") circle(.04,$fn=8);
        
    DNA = [[Qbeam,.5,t,w,ang_fixed],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,0,10*$t,0]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 6) {
    // U shaped beam, 10 segment:
    ELEM = [[Qbeam,1,.07,w,ang_fixed],[Qbeam,1,.07,w,0],[Qbeam,.313,.07,w,18],[Qbeam,.313,.07,w,18],[Qbeam,.313,.07,w,18],[Qbeam,.313,.07,w,18],[Qbeam,1,.07,w,18],[Qbeam,1,.07,w,0],[Qload,-1*$t,.02*$t,0,0],[Qbeam,2,.07,w,0],[Qbeam,2,.07,w,0],[Qload,2*$t,0,0,0]];
    
    Do_Analysis(ELEM,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 7) {
    // Compliant Claw, 33 segment 
    s1=0.135; // Length of Lower Arc Segments
    a1=10.3;   // Angle of Lower Arc Segments
    s2=0.35;   // Length of upper linear segments
    t=0.075;   // thickness of beams
    w=1.5;    // width of beams
    P=1*$t; // load pulling at servo
    DNA = [[Qbeam,s1,t,w,-90+a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qload,-P,-.27*P,0,0,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qload,.3*P,0,0,0,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 8) {
    // Straight diagonal Test Beam, 6 segment:
    p=10*$t;
    start_ang = 45;
    DNA = [[Qbeam,.5,t,w,start_ang],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,-p*cos(start_ang),p*sin(start_ang),0]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 9) {
    // Commpression Test Column, 6 segment:
    //  Euler Column Load Limit is about 3 lb for t = 0.05,  L = 3
    t = 0.05;  // beam thickness
    DNA = [[Qbeam,.5,t,w,1.],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,-2],[Qbeam,.5,t,w,0],[Qbeam,.5,t,.8,0], [Qload,-10*$t,,0,0]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 1.1) {
    // FORK TEST
    DNA = [[Qbeam,1.5,t,w,0],
    [Qbeam,1.5,t,w,20],
    [Qfork, 
        [ [Qbeam,1.5,t,2*w,90],[Qbeam,.7,t,w,90],[Qload,0,0,6] ] , 
        [ [Qbeam,1,t,w,-45], [Qbeam,.4,t,w,-30], [Qbeam,.2,t,w,-15], [Qload,2,.2,0] ] 
        ]
    ];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else {
    echo("NO BEAM ACTIVE");
}

module Do_Analysis(DNA,f_scale=1,output_console=true,E=300000,Ftu=5000,rho=0.05) {
    if(output_console) echo (DNA=DNA);
    
    draw_beam_undeformed(DNA);
    
    // Generate Beam Inertias and Cross Section Areas
    Iz = gen_Iz(DNA);
    Area = gen_Area(DNA);
    if(output_console) echo(Iz=Iz);
    if(output_console) echo(Area=Area);
    
    // Generate internal Global forces from external forces
    // Move the external loads from the dna to a seperate loads-only vector
    ext_loads_temp = loads_to_beams(DNA);
    if(output_console) echo(ext_loads_temp=ext_loads_temp);
    
    // Spread the external Global forces from the tail to the root
    // Note: the internal forces are correct, but moments don't include force-moments
    int_loads_temp = spread_ext_loads(ext_loads_temp);
    if(output_console) echo(int_loads_temp=int_loads_temp);
    draw_loads(beams_dxdy,ext_loads_temp,f_scale);
    
    // Generate Beam GLOBAL ANGLES, undeformed
    beam_angles = global_angles(DNA,undeformed=true);
    
    // Generate dx,dy for each beam          USED TO DRAW LOADS
    beams_dxdy = gen_dxdy(DNA,beam_angles);
    if(output_console) echo(beams_dxdy=beams_dxdy);
    
    // below here needs to be recomputer each iteration (to update moments-due-to-forces)
    
    // Convert internal global forces to beam-local forces 
    int_loads_temp_local = rotate_int_loads(int_loads_temp,beam_angles);
    if(output_console) echo(int_loads_temp_local=int_loads_temp_local);
    //draw_loads_V(beams_dxdy,int_loads_temp_local,f_scale);
    
    // Calculate moments due to forces
    force_mom_temp = moments_due_to_forces(int_loads_temp_local, DNA);
    if(output_console) echo(force_mom_temp=force_mom_temp);
    
    // Sum moments due to forces, starting at tail
    force_moments = sum_moments(force_mom_temp);
    if(output_console) echo(force_moments=force_moments);
    
    // Add moments-due-to-forces with internal loads
    int_loads_local = add_moments_to_loads(int_loads_temp_local,force_moments);
    if(output_console) echo(int_loads_local=int_loads_local);
    //draw_loads_V(beams_dxdy,int_loads_local,f_scale);
    
    results1 = compute_results(DNA,int_loads_local,Iz,Area,E,Ftu,rho);
    if(output_console) echo(results1=results1);
    
    //color ("blue") union () draw_beam_deformed(DNA,,beam_angles,results1);
    
    // NEXT ITERATION:
    beam_angles2 = add_angles(beam_angles,results1);
    if(output_console) echo(beam_angles=beam_angles);
    if(output_console) echo(beam_angles2=beam_angles2);
    
    // Convert internal global forces to beam-local forces 
    int_loads_temp_local2 = rotate_int_loads(int_loads_temp,beam_angles2);
    if(output_console) echo(int_loads_temp_local2=int_loads_temp_local2);
    //draw_loads_V(beams_dxdy,int_loads_temp_local,f_scale);
    
    // Calculate moments due to forces
    force_mom_temp2 = moments_due_to_forces(int_loads_temp_local2, DNA);
    if(output_console) echo(force_mom_temp2=force_mom_temp2);
    
    // Sum moments due to forces, starting at tail
    force_moments2 = sum_moments(force_mom_temp2);
    if(output_console) echo(force_moments2=force_moments2);
    
    // Add moments-due-to-forces with internal loads
    int_loads_local2 = add_moments_to_loads(int_loads_temp_local2,force_moments2);
    if(output_console) echo(int_loads_local2=int_loads_local2);
    //draw_loads_V(beams_dxdy,int_loads_local,f_scale);
    
    results2 = compute_results(DNA,int_loads_local2,Iz,Area,E,Ftu,rho);
    if(output_console) echo(results1=results2);
    
    union () draw_beam_deformed(DNA,beam_angles,results2);
    
    deformed_beam_angles = global_angles(DNA,undeformed=false,prior_ang=0,beam_angles2);
    echo(deformed_beam_angles=deformed_beam_angles);
    beams_dxdy2 = gen_dxdy(DNA,deformed_beam_angles);
    draw_loads(beams_dxdy2,ext_loads_temp,f_scale);
    
}
