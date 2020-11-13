// Large Displacement Beam, Test Models
include <LDB_Indexes.scad>
use <LDB_Modules.scad>

// Pick which beam definition to use
ACTIVE_BEAM=1.1; // [1:4seg End Moment, 1.1:8seg End Moment, 2:double 2seg End Moment, 3:2seg Normal Force, 4:4seg Normal Force, 5:6seg Normal Force, 5.05:6seg Normal Force Diagonal, 5.1:8seg Normal Force (test shape), 6.1:FORK single, 6.2:FORK single symmetric, 6.3:FORK double fork, 6.4:Tree 7 forks, 7:Compliant Claw with fork,9:Column]

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
    // CANTELIVER BEAM WITH MOMENT, 4 SEGMENT
    L = 4;
    ELEM = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,0,30*$t]];
    
    Do_Analysis(ELEM,force_scale*.5,output_console,E,Ftu,rho);
    
    translate([0,2.55,-1]) cylinder(h=1,r=2.5,center=true,$fn=32);

}
else if (ACTIVE_BEAM == 1.1) {
    // CANTELIVER BEAM WITH MOMENT, 8 SEGMENT
    L=2;
    DNA = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,0,30*$t]];
    
    Do_Analysis(DNA,force_scale*.5,output_console,E,Ftu,rho);
    
    translate([0,2.55,-1]) cylinder(h=1,r=2.5,center=true,$fn=32);
}

else if (ACTIVE_BEAM == 2) {
    // CANTELIVER BEAM WITH MOMENT, 2 SEGMENT
    L=2;
    DNA= [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qload,0,0,30*$t]];
    
    Do_Analysis(DNA,force_scale*.5,output_console,E,Ftu,rho,[1,0,0]);
    
    // multiple unique beams can run at once
    DNA2= [[Qbeam,L,t,w,ang_fixed+180],[Qbeam,L,t,w,0],[Qload,0,0,-30*$t]];
    
    Do_Analysis(DNA2,force_scale*.5,output_console,E,Ftu,rho,[-1,0,0]);
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
else if (ACTIVE_BEAM == 5.05) {
    // Straight diagonal Test Beam, 6 segment:
    p=10*$t;
    start_ang = 45;
    DNA = [[Qbeam,.5,t,w,start_ang],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,-p*cos(start_ang),p*sin(start_ang),0]];
    
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
else if (ACTIVE_BEAM == 6.1) {
    // Single FORK TEST
    DNA = [[Qbeam,1.5,t,w,10],
    [Qbeam,1.5,t,w,20],
    [Qfork, 
        [ [Qbeam,1.5,t,2*w,90],[Qbeam,.7,t,w,90],[Qload,0,0,10*$t] ] , 
        [ [Qbeam,1,t,w,-45], [Qbeam,.4,t,w,-30], [Qbeam,.2,t,w,-15], [Qload,4*$t,2*$t,0] ] 
        ]
    ];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 6.2) {
    // Symmetric Single FORK TEST 2
    DNA = [[Qbeam,1.5,t,w,90],[Qbeam,1.5,t,w,0],
        [Qfork, 
            [[Qbeam,2,t,w,90],[Qbeam,2,t,w,0],[Qload,0,0,10*$t]] , 
            [[Qbeam,2,t,w,-90],[Qbeam,2,t,w,0],[Qload,0,0,-20*$t]] 
        ]
    ];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 6.3) {
    // Double FORK TEST 3 
    DNA = [[Qbeam,1.5,t,w,90],[Qbeam,1.5,t,w,0],
    [Qfork, 
        [ [Qbeam,2,t,w,90],[Qbeam,2,t,w,0],[Qload,0,0,20*$t] ] , 
        [ [Qbeam,2,t,w,-90],[Qbeam,2,t,w,0],
            [Qfork,
                [[Qbeam,2,t,w,90],[Qbeam,2,t,w,0],[Qload,0,0,-20*$t]],
                [[Qbeam,2,t,w,-90],[Qbeam,2,t,w,0],[Qload,0,0,10*$t]]
            ]
        ]
    ]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 6.4) {
    // TREE, seven forks
    s1 = 1;
    s2 = 0.7;
    s3 = 0.6;
    t1 = 0.12;
    t2 = 0.08;
    t3 = 0.04;
    t4 = 0.025;
    w1 = 1;
    a1 = 35;  // branch angle
    leaf_load = .4*$t;
    
    tree = [[Qbeam,s1,t1,w1,90],[Qbeam,s1,t1,w1,0],
    [Qfork, 
        [ [Qbeam,s2,t2,w1,a1],[Qbeam,s2,t2,w1,0] , 
            [Qfork,
                [[Qbeam,s1,t3,w1,a1],[Qbeam,s1,t3,w1,0],
                    [Qfork,
                        [[Qbeam,s3,t4,w1,a1],[Qbeam,s3,t4,w1,0],[Qload,leaf_load,0,0]],
                        [[Qbeam,s2,t4,w1,-a1],[Qbeam,s2,t4,w1,0],[Qload,leaf_load,0,0]]
                    ],
                ],
                [[Qbeam,s2,t3,w1,-a1],[Qbeam,s2,t3,w1,0],
                    [Qfork,
                        [[Qbeam,s3,t4,w1,a1],[Qbeam,s3,t4,w1,0],[Qload,leaf_load,0,0]],
                        [[Qbeam,s2,t4,w1,-a1],[Qbeam,s2,t4,w1,0],[Qload,leaf_load,0,0]]
                    ],
                ]
            ]
        ],
        [ [Qbeam,s1,t2,w1,-a1],[Qbeam,s1,t2,w1,0],
            [Qfork,
                [[Qbeam,s2,t3,w1,a1],[Qbeam,s2,t3,w1,0],
                    [Qfork,
                        [[Qbeam,s3,t4,w1,a1],[Qbeam,s3,t4,w1,0],[Qload,leaf_load,0,0]],
                        [[Qbeam,s2,t4,w1,-a1],[Qbeam,s2,t4,w1,0],[Qload,leaf_load,0,0]]
                    ],
                ],
                [[Qbeam,s1,t3,w1,-a1],[Qbeam,s1,t3,w1,0],
                    [Qfork,
                        [[Qbeam,s3,t4,w1,a1],[Qbeam,s3,t4,w1,0],[Qload,leaf_load,0,0]],
                        [[Qbeam,s2,t4,w1,-a1],[Qbeam,s2,t4,w1,0],[Qload,leaf_load,0,0]]
                    ],
                ],
            ]
        ]
    ]];
    
    Do_Analysis(tree,force_scale*10,output_console,E,Ftu,rho);
}
else if (ACTIVE_BEAM == 7) {
    // Compliant Claw
    s1=0.135; // Length of Lower Arc Segments
    a1=10.8;   // Angle of Lower Arc Segments
    s2=0.362;   // Length of upper linear segments
    s3=0.3;    // Length of puller-link segments
    t=0.071;   // thickness of beams
    w=1.5;    // width of beams
    
    /* case 1 wide open
    Pr=1.9*$t; // load pulling at servo
    Pangr=-27; // P load direction
    Rr=0;  // load reacting on claw
    // Left Claw Load
    Pl=1.4*$t; // load pulling at servo
    Pangl=180; // P load direction
    Rl=0;  // load reacting on claw
    */
    
    // Right Claw Load
    Pr=3.4*$t; // load pulling at servo
    Pangr=154; // P load direction
    Rr=.35*$t;  // load reacting on claw
    // Left Claw Load
    Pl=5.2*$t; // load pulling at servo
    Pangl=-10; // P load direction
    Rl=1.1*$t;  // load reacting on claw
    //
    
    RightClaw = [[Qbeam,s1,t,w,-90+a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],
    [Qfork,
        [[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w*.8,0],[Qload,Rr,0,0,0,0]],
        
        [[Qbeam,s3,t,w*.8,90],[Qbeam,s3,t,w*.7,0],[Qbeam,s3,t,w*.6,0],[Qbeam,s3,t,w*.5,0],[Qbeam,s3,t,w*.4,0],[Qbeam,s3,t,w*.3,0],[Qload,Pr*cos(Pangr),Pr*sin(Pangr),0,0,0]]
    ]];

    LeftClaw = [[Qbeam,s1,t,w,-90-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],
    [Qfork,
        [[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w*.8,0],[Qload,-Rl,0,0,0,0]],
        
        [[Qbeam,s3,t,w*.8,-90],[Qbeam,s3,t,w*.7,0],[Qbeam,s3,t,w*.6,0],[Qbeam,s3,t,w*.5,0],[Qbeam,s3,t,w*.4,0],[Qbeam,s3,t,w*.3,0],[Qload,Pl*cos(Pangl),Pl*sin(Pangl),0,0,0]]
    ]];
    
    Origin_x=0.856;
    Origin_y=0.784;
    
    Do_Analysis(RightClaw,force_scale*4,output_console,E,Ftu,rho,[Origin_x,Origin_y,0]);
    
    Do_Analysis(LeftClaw,force_scale*4,output_console,E,Ftu,rho,[-Origin_x,Origin_y,0]);

    
    //Draw servo center and target point for link
    color("blue") translate([0,1.2+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([.7,1.2+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([-.35,1.8+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([.35,.6+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([-.7,1.2+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    
    //Draw block being grabbed
    color("pink") translate([-1,4,-1]) cube(2);

}
else if (ACTIVE_BEAM == 9) {
    // Commpression Test Column, 6 segment:
    //  Euler Column Load Limit is about 3 lb for t = 0.05,  L = 3
    t = 0.05;  // beam thickness
    DNA = [[Qbeam,.5,t,w,1.],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,-2],[Qbeam,.5,t,w,0],[Qbeam,.5,t,.8,0], [Qload,-10*$t,,0,0]];
    
    Do_Analysis(DNA,force_scale,output_console,E,Ftu,rho);
}


else {
    echo("NO BEAM ACTIVE");
}

module Do_Analysis(DNA,f_scale=1,output_console=true,E=300000,Ftu=5000,rho=0.05,origin=[0,0,0]) {
    if(output_console) echo (DNA=DNA);
    
    //translate(origin) draw_beam_undeformed(DNA);
    
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
    // Moments don't include force-moments at this time
    int_loads_temp = spread_ext_loads(ext_loads_temp);
    if(output_console) echo(int_loads_temp=int_loads_temp);
    
    // Generate Beam GLOBAL ANGLES, undeformed
    beam_angles = global_angles(DNA);
    
// below here needs to be recomputer each iteration (to update moments due to forces)
    
    // Convert internal global forces to beam-local forces 
    int_loads_temp_local = rotate_int_loads(int_loads_temp,beam_angles);
    if(output_console) echo(int_loads_temp_local=int_loads_temp_local);
    
    // Calculate moments due to forces
    force_mom_temp = moments_due_to_forces(int_loads_temp_local, DNA);
    if(output_console) echo(force_mom_temp=force_mom_temp);
    
    // Sum moments due to forces, starting at tail
    force_moments = sum_moments(force_mom_temp);
    if(output_console) echo(force_moments=force_moments);
    
    // Add moments-due-to-forces with internal loads
    int_loads_local = add_moments_to_loads(int_loads_temp_local,force_moments);
    if(output_console) echo(int_loads_local=int_loads_local);
    
    results1 = compute_results(DNA,int_loads_local,Iz,Area,E,Ftu,rho);
    
    beam_angles1 = add_angles(beam_angles,results1);
    
    translate(origin) union () draw_beam_deformed(DNA,beam_angles1,results1);
    
    // Generate dx,dy for each beam          USED TO DRAW LOADS
    beam_dxdy = gen_dxdy_undeformed(DNA,beam_angles);
    beam_dxdy1 = gen_dxdy_deformed(DNA,results1,beam_angles1);
    beam_nodes = gen_nodes(beam_dxdy);
    beam_nodes1 = gen_nodes(beam_dxdy1);
    translate(origin) draw_loads(beam_nodes1,ext_loads_temp,f_scale,0);
   // draw_loads(beam_nodes,ext_loads_temp,f_scale,0);
    
    if(output_console) {
        echo("GLOBAL ANGLES");
        echo("ANGLES UNLOADED ",beam_angles = beam_angles);
        echo("ANGLES LOADED ",beam_angles1=beam_angles1);
        echo("DXDY");
        echo("UNLOADED ",beam_dxdy=beam_dxdy);
        echo("LOADED ",beam_dxdy1=beam_dxdy1);
        echo("NODES");
        echo("nodes UNLOADED ",beam_nodes=beam_nodes);
        echo("nodes LOADED ",beam_nodes1=beam_nodes1);
        echo("RESULTS 1");
        echo(results1=results1);
    }  
}
