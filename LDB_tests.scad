// Large Displacement Beam, Test Models
include <LDB_Indexes.scad>
use <LDB_Modules.scad>

// Pick which beam definition to use
ACTIVE_BEAM=7; // [1:4seg End Moment, 1.1:8seg End Moment, 2:double 2seg End Moment, 3:2seg Normal Force, 4:4seg Normal Force, 5:6seg Normal Force, 5.05:6seg Normal Force Diagonal, 5.1:8seg Normal Force (test shape), 5.2:test (125g), 5.3:test (545g),5.4:test-reaction (545g),6.1:FORK single, 6.2:FORK single symmetric, 6.3:FORK double fork, 6.4:Tree 7 forks, 7:Compliant Claw with fork,9:Column,10:BAD DATA]

// Display intermediate load steps?
Display_steps = false;

// Scale of Force & Moment Display
force_scale = .1;

// MATERIAL PROPERTIES. 

// Modulus of Elasticity (PSI), PLA=340000,PETG=300000,Polycar=320000
E = 340000;
// Approximate Stress level at which the part will fail (PSI)
Failure_Stress = 10000;
// This could be tensile failure, compression failure, bending, etc.

// material density (lb per inch^3)
density = 0.045;

// Beam thickness
t=.15;  
// beam width
w=.8;
// beam angle at fixed end
ang_fixed = 0;

if (ACTIVE_BEAM == 1) {
    // CANTILEVER BEAM WITH MOMENT, 4 SEGMENT
    L = 4;
    ELEM = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,0,30*$t,0,0,0]];
    
    Do_Analysis(ELEM,force_scale*.5,Display_steps,E,Failure_Stress,density);
    
    translate([0,2.55,-1]) cylinder(h=1,r=2.5,center=true,$fn=32);

}
else if (ACTIVE_BEAM == 1.1) {
    // CANTILEVER BEAM WITH MOMENT, 8 SEGMENT
    L=2;
    DNA = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,0,30*$t,0,0,0]];
    
    Do_Analysis(DNA,force_scale*.5,Display_steps,E,Failure_Stress,density);
    
    translate([0,2.55,-1]) cylinder(h=1,r=2.5,center=true,$fn=32);
}

else if (ACTIVE_BEAM == 2) {
    // CANTILEVER BEAM WITH MOMENT, 2 SEGMENT
    L=2;
    DNA= [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qload,0,0,30*$t]];
    
    Do_Analysis(DNA,force_scale*.5,Display_steps,E,Failure_Stress,density,[1,0,0]);
    
    // multiple unique beams can run at once
    DNA2= [[Qbeam,L,t,w,ang_fixed+180],[Qbeam,L,t,w,0],[Qload,0,0,-30*$t]];
    
    Do_Analysis(DNA2,force_scale*.5,Display_steps,E,Failure_Stress,density,[-1,0,0]);
}
else if (ACTIVE_BEAM == 3) {
    // CANTILEVER BEAM WITH FORCE, 2 SEGMENT
    DNA = [[Qbeam,1.5,t,w,ang_fixed],[Qbeam,1.5,t,w,0],[Qload,0,10,0]];
    //DNA = [[Qbeam,3,t,w,ang_fixed],[Qload,0,10,0]];
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 4) {
    // CANTILEVER BEAM WITH FORCE, 4 SEGMENT
    DNA = [[Qbeam,.75,t,w,ang_fixed],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qload,0,-10,0]];
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5) {
    // CANTILEVER BEAM WITH FORCE, 6 SEGMENT
    // TEST CASE: L=3, t=0.15, w=.8, F=10,Roark defection=1.2,min MS=-0.06
DNA = [[Qbeam,.5,t,w,ang_fixed],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,0,10,0] ]; 
    
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5.05) {
    // Straight diagonal Test Beam, 6 segment:
    p=10*$t;
    start_ang = 45;
    DNA = [[Qbeam,.5,t,w,start_ang],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,-p*cos(start_ang),p*sin(start_ang),0]];
    
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5.1) {
    // CANTILEVER BEAM WITH FORCE, 8 SEGMENT
    // TEST CASE: L=4, t=0.15, w=.8, F=10,Roark defection=2.79
    
    pts=[[0,0],[0.4961, 0.0626],[0.9639,0.2397],[1.3822,0.5147],[1.7340,0.8715],[2.0056,1.2936],[2.1826,1.7647],[2.2438,2.2685],[2.262,2.7887]];
    
    for (i=[0:8]) translate([pts[i][0],pts[i][1],1]) color("black") circle(.04,$fn=8);
        
    DNA = [[Qbeam,.5,t,w,ang_fixed],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,0,10*$t,0]];
    
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5.2) {
    // CANTILEVER BEAM WITH FORCE, 7 SEGMENT, 125 g
    // TEST CASE: L=7, t=0.062, w=1.06, F=0.278 lb
    L=1;
    t=0.062;
    w=1.06;
    weight=0.278; // lbs
    E = 320000; // Polycarbonate
    density = 0.043;

    pts=[[0,0],[1,.1],[1.95,0.5],[2.85,1],[3.7,1.6],[4.5,2.2],[5.3,2.9],[6,3.6]];
    
    for (i=[0:7]) translate([pts[i][0],pts[i][1],1]) color("black") circle(.03,$fn=8);
        
    DNA = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,weight,0,6,3.6,0]];
    
    Do_Analysis(DNA,force_scale*10,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5.3) {
    // CANTILEVER BEAM WITH FORCE, 7 SEGMENT, 545 g
    // TEST CASE: L=7, t=0.062, w=1.06, F=1.203 lb
    L=1;
    t=0.062;
    w=1.06;
    weight=1.2; // lbs
    E = 320000; // Polycarbonate
    density = 0.043;
    
    pts=[[0,0],[.95,0.3],[1.7,1],[2.3,1.8],[2.7,2.75],[3,3.7],[3.3,4.7],[3.5,5.75]];
    
    for (i=[0:7]) translate([pts[i][0],pts[i][1],1]) color("black") circle(.03,$fn=8);
        
    DNA = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,weight,0,3.5,5.75,0]];
    
    Do_Analysis(DNA,force_scale*4,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5.4) {
    // TEST BEAM WITH FORCE AND REACTION
    // 7 SEGMENT, 545 g
    // TEST CASE: L=7, t=0.062, w=1.06, F=1.203 lb
    L=1;
    t=0.062;
    w=1.06;
    weight=1.2; // lbs
    react=1.58;  // the linear answer should be ~1.59
    E = 320000; // Polycarbonate
    density = 0.043;
    
    pts=[[0,0],[1,0],[2,.15],[3,.4],[4,.5],[4.9,.8],[5.7,1.4],[6.5,2.1]];
    
    for (i=[0:7]) translate([pts[i][0],pts[i][1],1]) color("black") circle(.03,$fn=8);
        
    DNA = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,-react,0,4.9,1],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,weight,0,6.5,2.1]];
    
    Do_Analysis(DNA,force_scale*6,Display_steps,E,Failure_Stress,density);
}

else if (ACTIVE_BEAM == 6.1) {
    // Single FORK TEST
    DNA = [[Qbeam,1.5,t,w,10],
    [Qbeam,1.5,t,w,20],
    [Qfork, 
        [ [Qbeam,1.5,t,2*w,90],[Qbeam,.7,t,w,90],[Qload,0,0,10*$t] ] , 
        [ [Qbeam,1,t,w,-45], [Qbeam,.4,t,w,-30], [Qbeam,.2,t,w,-15], [Qload,4*$t,6*$t,0] ] 
        ]
    ];
    
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 6.2) {
    // Symmetric Single FORK TEST 2
    DNA = [[Qbeam,1.5,t,w,90],[Qbeam,1.5,t,w,0],
        [Qfork, 
            [[Qbeam,2,t,w,90],[Qbeam,2,t,w,0],[Qload,0,0,10*$t]] , 
            [[Qbeam,2,t,w,-90],[Qbeam,2,t,w,0],[Qload,0,0,-20*$t]] 
        ]
    ];
    
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
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
    
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
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
    leaf_load = .6*$t;
    
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
    
    Do_Analysis(tree,force_scale*10,Display_steps,E,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 7) {
    // Compliant Claw
    s1=0.135; // Length of Lower Arc Segments
    a1=10.8;   // Angle of Lower Arc Segments
    s2=0.362;   // Length of upper linear segments
    s3=0.3;    // Length of puller-link segments
    t=0.069;   // thickness of beams
    w=1.5;    // width of beams

    Origin_x=0.856;
    Origin_y=0.784;
    
    /* case 1 wide open
    Pr=1.6*$t; // load pulling at servo
    Pangr=-33; // P load direction
    Rr=0;  // load reacting on claw
    // Left Claw Load
    Pl=1.2*$t; // load pulling at servo
    Pangl=191; // P load direction
    Rl=0;  // load reacting on claw
    */
    
    // Right Claw Load
    Prx=-3.3*$t; // load pulling at servo
    Pry=1.6*$t; // P load direction
    Rr=.4*$t;  // load reacting on claw
    // Left Claw Load
    Plx=5.1*$t; // load pulling at servo
    Ply=-2.2*$t; // P load direction
    Rl=-1.1*$t;  // load reacting on claw
    //
    
    RightClaw = [[Qbeam,s1,t,w,-90+a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s1,t,w,a1],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],
    [Qfork,
        [[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w*.8,0],[Qload,Rr,0,0,1-Origin_x,6-Origin_y]],
        
        [[Qbeam,s3,t,w*.8,90],[Qbeam,s3,t,w*.7,0],[Qbeam,s3,t,w*.6,0],[Qbeam,s3,t,w*.5,0],[Qbeam,s3,t,w*.4,0],[Qbeam,s3,t,w*.3,0],[Qload,Prx,Pry,0,-Origin_x-0.35,2-Origin_y+.6]]
    ]];

    LeftClaw = [[Qbeam,s1,t,w,-90-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s1,t,w,-a1],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],
    [Qfork,
        [[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w,0],[Qbeam,s2,t,w*.8,0],[Qload,Rl,0,0,-1+Origin_x,6-Origin_y]],
        
        [[Qbeam,s3,t,w*.8,-90],[Qbeam,s3,t,w*.7,0],[Qbeam,s3,t,w*.6,0],[Qbeam,s3,t,w*.5,0],[Qbeam,s3,t,w*.4,0],[Qbeam,s3,t,w*.3,0],[Qload,Plx,Ply,0,.35+Origin_x,2-Origin_y-.6]]
    ]];
    
    echo(" RIGHT CLAW +X");
    Do_Analysis(RightClaw,force_scale*4,Display_steps,E,Failure_Stress,density,[Origin_x,Origin_y,0]);
    
    echo(" LEFT CLAW -X");
    Do_Analysis(LeftClaw,force_scale*4,Display_steps,E,Failure_Stress,density,[-Origin_x,Origin_y,0]);
    
    //Draw servo center and target point for link
    color("blue") translate([0,1.2+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([.7,1.2+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([-.35,1.8+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([.35,.6+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    translate([-.7,1.2+Origin_y,-1]) cylinder(h=.5,r=.05,center=true,$fn=16);
    
    //Draw block being grabbed
    color("pink") translate([-1,4,-3]) cube(2);
}
else if (ACTIVE_BEAM == 9) {
    // Compression Test Column, 6 segment:
    //  Euler Column Load Limit is about 3 lb for t = 0.05,  L = 3
    t = 0.05;  // beam thickness
    DNA = [[Qbeam,.5,t,w,1.],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,-2],[Qbeam,.5,t,w,0],[Qbeam,.5,t,.8,0], [Qload,-10*$t,,0,0]];
    
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}

else if (ACTIVE_BEAM == 10 ) {
    // Junk case
    DNA = 5;
    echo("JUNK TEST CASE");
    Do_Analysis(DNA,force_scale,Display_steps,E,Failure_Stress,density);
}

module Do_Analysis(DNA,f_scale=1,Display_steps=true,E=300000,Failure_Stress=5000,density=0.05,origin=[0,0,0]) {
    echo("**********");
    echo("LARGE DISPLACEMENT 2D BEAM ANALYSIS BASED ON COMPLIANT MECHANISM PRBM");
    echo("TIME $t = ",$t,E=E,Failure_Stress=Failure_Stress,density=density);
    
    // perform data checks
    num_branches = count_branches(DNA);
    if (num_branches > 0 ) {
    
    echo("NUMBER OF BRANCHES IS ",num_branches,"  NUMBER OF FORKS IS ",(num_branches-1)/2);
    
    echo("TREE DEPTH IS ",tree_depth(DNA));
    
    num_beams = count_beams(DNA);
    num_loads = count_loads(DNA);
        
    if (num_beams > 0 && num_loads > 0) {
    echo("NUMBER OF BEAMS IS ",num_beams," NUMBER OF LOADS IS ",num_loads);

    // Generate Beam Inertias and Cross Section Areas
    Iz = gen_Iz(DNA);
    Area = gen_Area(DNA);
    // Generate internal Global forces from external forces
    // Move the external loads from the dna to a separate loads-only vector
    ext_loads_temp = loads_to_beams(DNA);
    // Spread the external Global forces from the tails to the root
    // Moments don't include force-moments at this time
    initial_loads = spread_ext_loads(ext_loads_temp);
    // Generate Beam GLOBAL ANGLES, undeformed
//echo(ext_loads_temp=ext_loads_temp);
//echo(initial_loads=initial_loads);
    beam_angles = global_angles(DNA);

    //    echo("INITIAL VECTORS, THESE DON'T CHANGE");
    //    echo (DNA=DNA);
    //    echo(Iz=Iz);
    //    echo(Area=Area);
    //    echo(initial_loads=initial_loads);

// Step the load application to minimize angle oscilation
    sc0=0;
    sc1=0.16667;
    sc2=0.33333;
    sc3=0.5;
    sc4=0.66667;
    sc5=0.83333;
    sc6=1.0;
    
    results0 = compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,initial_loads,beam_angles,scale=sc0); 
    beam_nodes = compute_nodes(DNA,results0,beam_angles);
    if(Display_steps) translate(origin) draw_loads(beam_nodes,ext_loads_temp,f_scale*sc0,"green",0);
    
    results1 = compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,initial_loads,beam_angles,scale=sc1); 
    beam_angles1 = add_angles(beam_angles,results1);
    beam_nodes1 = compute_nodes(DNA,results1,beam_angles1);
    if(Display_steps) translate(origin) draw_loads(beam_nodes1,ext_loads_temp,f_scale*sc1,"Lavender",0);
    
    results2 = compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,initial_loads,beam_angles1,scale=sc2); 
    beam_angles2 = add_angles(beam_angles,results2);
    beam_nodes2 = compute_nodes(DNA,results2,beam_angles2);
    if(Display_steps) translate(origin) draw_loads(beam_nodes2,ext_loads_temp,f_scale*sc2,"Fuchsia",0);
    
    results3 = compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,initial_loads,beam_angles2,scale=sc3); 
    beam_angles3 = add_angles(beam_angles,results3);
    beam_nodes3 = compute_nodes(DNA,results3,beam_angles3);
    if(Display_steps) translate(origin) draw_loads(beam_nodes3,ext_loads_temp,f_scale*sc3,"Aqua",0);

    results4 = compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,initial_loads,beam_angles3,scale=sc4); 
    beam_angles4 = add_angles(beam_angles,results4);
    beam_nodes4 = compute_nodes(DNA,results4,beam_angles4);
    if(Display_steps) translate(origin) draw_loads(beam_nodes4,ext_loads_temp,f_scale*sc4,"Blue",0);

    results5 = compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,initial_loads,beam_angles4,scale=sc5); 
    beam_angles5 = add_angles(beam_angles,results5);
    beam_nodes5 = compute_nodes(DNA,results5,beam_angles5);
    if(Display_steps) translate(origin) draw_loads(beam_nodes5,ext_loads_temp,f_scale*sc5,"red",0);

    results6 = compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,initial_loads,beam_angles5,scale=sc6); 
    beam_angles6 = add_angles(beam_angles,results6);
    beam_nodes6 = compute_nodes(DNA,results6,beam_angles6);
    translate(origin) draw_loads(beam_nodes6,ext_loads_temp,f_scale*sc6,"yellow",0);

     draw_ground_reactions(results6,f_scale*sc6,origin,DNA[0][Zang]);

    translate(origin) union () draw_beam_deformed(DNA,beam_angles6,results6);
    //echo(beam_nodes6=beam_nodes6);
    echo("X MAX ",max_tree(beam_nodes6,0,Zdx)+origin[0],
         "  X MIN ",min_tree(beam_nodes6,0,Zdx)+origin[0]);
    echo("Y MAX ",max_tree(beam_nodes6,0,Zdy)+origin[1],
         "  Y MIN ",min_tree(beam_nodes6,0,Zdy)+origin[1]);
    //echo(results6=results6);
    echo("STRESS MAX ",max_tree(results6,0,Zstressmax),
         "  STRESS MIN ",min_tree(results6,0,Zstressmin));
    echo("MIN Margin of Safety ",min_tree(results6,0,Zms),
         " MAX MS ",max_tree(results6,0,Zms));
    echo("WEIGHT ",sum_tail2(results6,0,Zweight,Qresult),
         " ENERGY ",sum_tail2(results6,0,Zenergy,Qresult));

//displacements = check_displacement_target(DNA,beam_nodes6,beam_nodes);
//echo(displacements=displacements);

} else echo("**NUMBER OF BEAMS OR LOADS IS ZERO, TERMINATING**"); 
} else echo("**NUMBER OF BRANCHES IS ZERO, TERMINATING**");
}

function compute_iteration(DNA,Iz,Area,E,Failure_Stress,density,loads,beam_angles,scale=1) =     
// Compute an iteration (to update moments due to forces)
    // scale internal loads
    let (loads_scaled = scale_int_loads(loads,scale))
    // Convert internal global forces to beam-local forces 
    let (loads_local = rotate_int_loads(loads_scaled,beam_angles))
    // Calculate moments due to forces
    let (force_mom_temp = moments_due_to_forces(loads_local, DNA, beam_angles))
    // Sum moments due to forces, starting at tail
    let (force_moments = sum_moments(force_mom_temp))
    // Add moments-due-to-forces with internal loads
    let (NEW_loads_local = add_moments_to_loads(loads_local,force_moments))
    // call function compute results
    compute_results(DNA,NEW_loads_local,Iz,Area,E,Failure_Stress,density);

function compute_nodes(DNA,results,angles) =
// combine gen_dxdy_deformed with gen_nodes into single function
    let (dxdy=gen_dxdy_deformed(DNA,results,angles))
    gen_nodes(dxdy);