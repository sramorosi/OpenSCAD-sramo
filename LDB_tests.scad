// Large Displacement Beam, Test Models
include <LDB_Indexes.scad>
use <LDB_Modules.scad>

// Pick which beam definition to use
ACTIVE_BEAM=1; // [1:1-4seg End Moment-CIRCLE, 1.1:1.1-8seg End Moment-CIRCLE, 2:2-double 2seg End Moment, 3:3-2seg Normal Force, 4:4-4seg Normal Force, 5:5-6seg Normal Force, 5.05:5.05-6seg Normal Force Diagonal, 5.1:5.1-8seg Normal Force (test shape), 5.2:5.2-test (125g), 5.3:5.3-test (545g),5.4:5.4-test-reaction (545g),6.1:6.1-FORK single, 6.2:6.2-FORK single symmetric, 6.3:6.3-FORK double fork, 6.4:6.4-Tree 7 forks, 7:7-Compliant Claw with fork,9:9-Column,10:10-BAD DATA]

// Display intermediate load steps?
Display_steps = false;
// Scale of Force & Moment Display
force_scale = 1.0; // [0.05:.05:2.0]
// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI), PLA=340000,PETG=300000,Polycar=320000
//E = 320000; // MUST MODIFY IN MODULES 
// ~Stress level at which the part will fail (PSI)
Failure_Stress = 10000;
// This could be tensile failure, compression failure, bending, etc.
// material density (lb per inch^3)
density = 0.045;

// Beam thickness
t=.11;  
// beam width
w=.8;
// beam angle at fixed end
ang_fixed = 0;

if (ACTIVE_BEAM == 1) {
    // CANTILEVER BEAM WITH MOMENT 
    t=0.1;
    NumberBeams=20; 
    L = 31.415;  // circle len = pi()*d  (d=10 or r=5)
    LN = L/NumberBeams;
    ELEM = [for (i=[1:NumberBeams]) [Qbeam,LN,t,w,0]];
    echo(ELEM=ELEM);
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[0,0,4.53]]);
    echo(LOADS1=LOADS1);

    Do_Analysis(ELEM,LOADS1,force_scale,Display_steps,Failure_Stress,density);
    
    // The beam should roughly wrap the cylinder
    translate([0,5,-1]) cylinder(h=1,r=5,center=true,$fn=32);

}
else if (ACTIVE_BEAM == 1.1) {
    // CANTILEVER BEAM WITH MOMENT, 8 SEGMENT
    L=2;
    LDB_DEF = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,0,30*1,0,0,0]]; // 30*$t
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
    
    translate([0,2.55,-1]) cylinder(h=1,r=2.5,center=true,$fn=32);
}

else if (ACTIVE_BEAM == 2) {
    // CANTILEVER BEAM WITH MOMENT, 2 SEGMENT
    L=2;
    LDB_DEF= [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qload,0,0,30*1]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density,[1,0,0]);
    
    // multiple unique beams can run at once
    LDB_DEF2= [[Qbeam,L,t,w,ang_fixed+180],[Qbeam,L,t,w,0],[Qload,0,0,-30*1]];
    
    Do_Analysis(LDB_DEF2,force_scale,Display_steps,Failure_Stress,density,[-1,0,0]);
}
else if (ACTIVE_BEAM == 3) {
    // CANTILEVER BEAM WITH FORCE, 2 SEGMENT
    LDB_DEF = [[Qbeam,1.5,t,w,ang_fixed],[Qbeam,1.5,t,w,0],[Qload,0,10,0]];
    //LDB_DEF = [[Qbeam,3,t,w,ang_fixed],[Qload,0,10,0]];
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 4) {
    // CANTILEVER BEAM WITH FORCE, 4 SEGMENT
    LDB_DEF = [[Qbeam,.75,t,w,ang_fixed],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qbeam,.75,t,w,0],[Qload,0,-10,0]];
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5) {
    // CANTILEVER BEAM WITH FORCE, 6 SEGMENT
    // TEST CASE: L=3, t=0.15, w=.8, F=10,Roark defection=1.2,min MS=-0.06
    TOTAL_LEN = 3;
    SUB_LEN = TOTAL_LEN/6;
    THK = 0.15;
    LDB_DEF = [[Qbeam,SUB_LEN,THK,w,ang_fixed,0],
    [Qbeam,SUB_LEN,THK,w,0,0],
    [Qbeam,SUB_LEN,THK,w,0,0],
    [Qbeam,SUB_LEN,THK,w,0,0],
    [Qbeam,SUB_LEN,THK,w,0,0],
    [Qbeam,SUB_LEN,THK,w,0,0],
    [Qload,0,20,0] ]; 
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5.05) {
    // Straight diagonal Test Beam, 6 segment:
    p=10*1;  // 10*$t
    start_ang = 45;
    LDB_DEF = [[Qbeam,.5,t,w,start_ang],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,-p*cos(start_ang),p*sin(start_ang),0]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 5.1) {
    // CANTILEVER BEAM WITH FORCE, 8 SEGMENT
    // TEST CASE: L=4, t=0.15, w=.8, F=10,Roark defection=2.79
    
    // DATA FROM SOME OTHER PROGRAM
    pts=[[0,0],[0.4961, 0.0626],[0.9639,0.2397],[1.3822,0.5147],[1.7340,0.8715],[2.0056,1.2936],[2.1826,1.7647],[2.2438,2.2685],[2.262,2.7887]];
    draw_points(pts,dia=0.05);
        
    LDB_DEF = [[Qbeam,.5,t,w,ang_fixed],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qload,0,30,0]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
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

    // MEASURED TEST DATA
    pts=[[0,0],[1,.1],[1.95,0.5],[2.85,1],[3.7,1.6],[4.5,2.2],[5.3,2.9],[6,3.6]];
    draw_points(pts,dia=0.05);
        
    LDB_DEF = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,weight,0,6,3.6,0]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
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

    // MEASURED TEST DATA    
    pts=[[0,0],[.95,0.3],[1.7,1],[2.3,1.8],[2.7,2.75],[3,3.7],[3.3,4.7],[3.5,5.75]];
    draw_points(pts,dia=0.05);
        
    LDB_DEF = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,weight,0,3.5,5.75,0]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
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
    
    // MEASURED TEST DATA
    pts=[[0,0],[1,0],[2,.15],[3,.4],[4,.5],[4.9,.8],[5.7,1.4],[6.5,2.1]];
    draw_points(pts,dia=0.05);
        
    LDB_DEF = [[Qbeam,L,t,w,ang_fixed],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,-react,0,4.9,1],[Qbeam,L,t,w,0],[Qbeam,L,t,w,0],[Qload,0,weight,0,6.5,2.1]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}

else if (ACTIVE_BEAM == 6.1) {
    // Single FORK TEST
    LDB_DEF = [[Qbeam,1.5,t,w,10],
    [Qbeam,1.5,t,w,20],
    [Qfork, 
        [ [Qbeam,1.5,t,2*w,90],[Qbeam,.7,t,w,90],[Qload,0,0,10*$t] ] , 
        [ [Qbeam,1,t,w,-45], [Qbeam,.4,t,w,-30], [Qbeam,.2,t,w,-15], [Qload,4*$t,6*$t,0] ] 
        ]
    ];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 6.2) {
    // Symmetric Single FORK TEST 2
    LDB_DEF = [[Qbeam,1.5,t,w,90],[Qbeam,1.5,t,w,0],
        [Qfork, 
            [[Qbeam,2,t,w,90],[Qbeam,2,t,w,0],[Qload,0,0,10*$t]] , 
            [[Qbeam,2,t,w,-90],[Qbeam,2,t,w,0],[Qload,0,0,-20*$t]] 
        ]
    ];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}
else if (ACTIVE_BEAM == 6.3) {
    // Double FORK TEST 3 
    LDB_DEF = [[Qbeam,1.5,t,w,90],[Qbeam,1.5,t,w,0],
    [Qfork, 
        [ [Qbeam,2,t,w,90],[Qbeam,2,t,w,0],[Qload,0,0,20*$t] ] , 
        [ [Qbeam,2,t,w,-90],[Qbeam,2,t,w,0],
            [Qfork,
                [[Qbeam,2,t,w,90],[Qbeam,2,t,w,0],[Qload,0,0,-20*$t]],
                [[Qbeam,2,t,w,-90],[Qbeam,2,t,w,0],[Qload,0,0,10*$t]]
            ]
        ]
    ]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
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
    
    Do_Analysis(tree,force_scale,Display_steps,Failure_Stress,density);
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
    Rr=0;  // load reacting on claw=
    // Left Claw Load
    Pl=1.2*$t; // load pulling at servo
    Pangl=191; // P load direction
    Rl=0;  // load reacting on claw
    */
    
    // Right Claw Load
    load_scaler = 1.0; // or $t for animation
    
    Prx=-3.3*load_scaler; // load pulling at servo
    Pry=1.6*load_scaler; // P load direction
    Rr=.4*load_scaler;  // load reacting on claw
    // Left Claw Load
    Plx=5.1*load_scaler; // load pulling at servo
    Ply=-2.2*load_scaler; // P load direction
    Rl=-1.1*load_scaler;  // load reacting on claw
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
    Do_Analysis(RightClaw,force_scale,Display_steps,Failure_Stress,density,[Origin_x,Origin_y,0]);
    
    echo(" LEFT CLAW -X");
    Do_Analysis(LeftClaw,force_scale,Display_steps,Failure_Stress,density,[-Origin_x,Origin_y,0]);
    
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
    LDB_DEF = [[Qbeam,.5,t,w,1.],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,0],[Qbeam,.5,t,w,-2],[Qbeam,.5,t,w,0],[Qbeam,.5,t,.8,0], [Qload,-10*$t,,0,0]];
    
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}

else if (ACTIVE_BEAM == 10 ) {
    // Junk case
    LDB_DEF = 5;
    echo("JUNK TEST CASE");
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,density);
}

