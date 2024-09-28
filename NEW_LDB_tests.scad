// Large Displacement Beam, Test Models
include <NEW_LDB_Indexes.scad>
use <NEW_LDB_Modules.scad>

// Pick which beam definition to use
ACTIVE_BEAM=1; // [1:1-CANTILEVER BEAM w End Moment-CIRCLE, 2:2-CANTILEVER BEAM w Force & Moment, 3:3-SINE WAVE BEAM, 4:4-CANTILEVER BEAM w Distributed Force, 5:5-8seg Normal Force (test shape), 6:6-test (125g), 7:7-test (545g),8:8-test-reaction (545g) TBD,9:9-Column,10:10-FRAME,11:11-Parallel Flex AIRPLANE LAUNCHER,12:12-CROSS FRAME,13:13-Compression Beam,99:99-BAD DATA]

// Display intermediate load steps?
Display_steps = false;
// Load Steps
Load_Steps = 4; // [1:1:20]
// Scale of Force & Moment Display
force_scale = 0.5; // [0.05:.05:2.0]

// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI)
E_PLA_PSI = 340000;  // USED IN MANY FUNCTIONS AND MODULES
E_PLA_NSMM = 2344;  // Modulus of Elasticity (NEWTONS PER mm^2), PLA
E_PETG_NSMM = 2068;  // Modulus of Elasticity (NEWTONS PER mm^2), PETG
E_PSI = E_PLA_PSI; // MUST MODIFY IN MODULES 

// ~Stress level at which the material will fail
FAILURE_STRESS_PLA_PSI = 6600;  // PLA, IN PSI
FAILURE_STRESS_PLA_METRIC = 45;  // ~Stress level at which PLA will fail (NEWTONS per mm^2)
FAILURE_STRESS_PETG_METRIC = 60;  // ~Stress level at which PETG will fail (NEWTONS per mm^2)
// This could be tensile failure, compression failure, bending, etc.
Failure_Stress = FAILURE_STRESS_PLA_PSI;

DENSITY_PLA_IMPERIAL = 0.045;  // material density (lb per inch^3)
DENSITY_PLA_METRIC = 0.0012318;  // material density (gram per mm^3)
DENSITY_PETG_METRIC = 0.0012733;  // material density (gram per mm^3)
density = DENSITY_PLA_IMPERIAL;

// Beam thickness
t=.15;  // inch
// Number of Beams
NumberBeams=20; // [2:1:40]

// beam width
w=.8;
// beam angle at fixed end
ang_fixed = 0; // [-90:10:90]

if (ACTIVE_BEAM == 1) { // CANTILEVER BEAM WITH MOMENT 
    t=0.1;
    L = 30;  // circle len = pi()*d  
    NBeams = 80;  // the more beams, the closer it gets
    LN = L/NBeams;
    ELEM_VIS = [for (i=[1:NBeams]) [Qbeam,LN,t,w,0]];
        
    // NEW METHOD, CONCATINATE A BEAM AT START AND END
    ELEM = concat([[Qbeam,LN,t*1,w,0]] ,concat(ELEM_VIS,[[Qbeam,LN,t,w,0]]));  
    
    //echo(ELEM=ELEM);
    //MAKE_BEAM_UNDEFORMED(ELEM,w);
    
    // THE CORRECT MOMENT IS === 5 
    LOADS1 = concat([for (i=[1:NBeams]) [0,0,0]],[[0,0,5]]);
    //echo(LOADS1=LOADS1);

    // This problem only needs 1 load step!
    DO_ANALYSIS(ELEM,LOADS1,force_scale,Display_steps,Failure_Stress,E_PSI,density,Origin=[0,0,0],steps=1);

    // The beam should roughly wrap the cylinder
    translate([0,L/(2*PI),-1]) cylinder(h=1,r=L/(2*PI),center=true,$fn=32);
    
} else if (ACTIVE_BEAM == 2) { // CANTILEVER BEAM WITH FORCE
    // TEST CASE: L=3, t=0.15, w=.8, F=10,Roark defection=1.2,min MS=-0.06
    t=.1;  
    w=0.8;
    L=3;
    LN = L/NumberBeams;
    //Fx=1200; // Axial Load Test
    //Kax=E*(t*w)/L;
    //New_len = Fx/Kax + L;
    //echo(Kax=Kax,New_len=New_len);
    Fx=0;
    Fy=16; // lbs
    Mz = -Fy*L/2;
    BEAM1 = [[Qbeam,LN,t,w,ang_fixed], for (i=[1:NumberBeams-1]) [Qbeam,LN,t,w,0]];
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,Fy,Mz]]);
   
    draw_beam_undeformed(BEAM1);

    Do_Analysis(BEAM1,LOADS1,force_scale*0.1,Display_steps,Failure_Stress,E_PSI,density,steps=Load_Steps);
} else if (ACTIVE_BEAM == 3) { // Sinewave beam
    X=-2;  // inch, width of sine
    Y=12; // inch, height of sine
    t=.2;
    pts=[for (i=[0:NumberBeams]) [X*cos(180*(i/NumberBeams))-X,Y*sin(180*(i/NumberBeams))] ];
    draw_points(pts,dia=0.1);
    
    BEAM1 = beamFromNodes(pts,t,w);
    
    echo(BEAM1=BEAM1);
        
    Fx = 1.8;
    //LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,0,0]]);
    //LOADS1 = concat(concat([for (i=[1:NumberBeams/2]) [0,0,0]],[[Fx,0,0]]),[for (i=[1:NumberBeams/2]) [0,0,0]]);
    LOADS1 = concat ( concat(concat([for (i=[1:NumberBeams/2]) [0,0,0]],[[Fx,0,0]]),[for (j=[1:(NumberBeams/2)-1]) [0,0,0]]),[[0,0,Fx*5]]); // << last load is back solved to hold node in location

    draw_beam_undeformed(BEAM1);
    Do_Analysis(BEAM1,LOADS1,force_scale*0.5,Display_steps,Failure_Stress,E_PSI,density,steps=Load_Steps);

} else if (ACTIVE_BEAM == 4) { // CANTILEVER BEAM W DISTRIBUTED VERTICAL FORCE
    // TEST CASE: L=3, t=0.15, w=.8, F=10,Roark defection=1.2,min MS=-0.06

    L=3;
    LN = L/NumberBeams;
    Fy=1; // lbs
    
    BEAM1 = [[Qbeam,LN,t,w,ang_fixed], for (i=[1:NumberBeams-1]) [Qbeam,LN,t,w,0]];
        
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,Fy,0]],[[0,Fy,0]]);
   
    draw_beam_undeformed(BEAM1);

    Do_Analysis(BEAM1,LOADS1,force_scale*0.1,Display_steps,Failure_Stress,E_PSI,density,steps=Load_Steps);

} else if (ACTIVE_BEAM == 5) { // CANTILEVER BEAM TEST CASE FROM OTHER SOURCE
    // VERTICAL FORCE, 8 SEGMENTS
    // TEST CASE: L=4, t=0.15, w=.8, F=10,Roark defection=2.79
    
    // DATA FROM SOME OTHER PROGRAM ???
    pts=[[0,0],[0.4961, 0.0626],[0.9639,0.2397],[1.3822,0.5147],[1.7340,0.8715],[2.0056,1.2936],[2.1826,1.7647],[2.2438,2.2685],[2.262,2.7887]];
    draw_points(pts,dia=0.05);
        
    L=4;
    LN = L/NumberBeams;
    t=0.15;
    w=0.8;
    Fy=18; // lbs
    BEAM1 = [[Qbeam,LN,t,w,ang_fixed], for (i=[1:NumberBeams-1]) [Qbeam,LN,t,w,0]];
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[0,Fy,0]]);
   
    draw_beam_undeformed(BEAM1);

    Do_Analysis(BEAM1,LOADS1,force_scale*0.1,Display_steps,Failure_Stress,E_PSI,density,steps=6);
} else if (ACTIVE_BEAM == 6) { // CANTILEVER BEAM TEST CASE F=126 g
    // W FORCE, 7 SEGMENT, 125 g
    // TEST CASE: L=7, t=0.062, w=1.06, F=0.278 lb
    E = 320000; // Polycarbonate
    density = 0.043;

    // MEASURED TEST DATA
    pts=[[0,0],[1,.1],[1.95,0.5],[2.85,1],[3.7,1.6],[4.5,2.2],[5.3,2.9],[6,3.6]];
    draw_points(pts,dia=0.05);
        
    NumberBeams=7; 
    L=7;
    LN = L/NumberBeams;
    t=0.062;
    w=1.06;
    Fy=0.278; // lbs

    BEAM1 = [[Qbeam,LN,t,w,ang_fixed], for (i=[1:NumberBeams-1]) [Qbeam,LN,t,w,0]];
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[0,Fy,0]]);
    
    draw_beam_undeformed(BEAM1);
    
    Do_Analysis(BEAM1,LOADS1,force_scale,Display_steps,Failure_Stress,E_PSI,density,steps=Load_Steps);
} else if (ACTIVE_BEAM == 7) { // CANTILEVER BEAM TEST CASE F=545G
    // WITH FORCE, 7 SEGMENT, 545 g
    // TEST CASE: L=7, t=0.062, w=1.06, F=1.203 lb
    E = 320000; // Polycarbonate
    density = 0.043;

    // MEASURED TEST DATA    
    pts=[[0,0],[.95,0.3],[1.7,1],[2.3,1.8],[2.7,2.75],[3,3.7],[3.3,4.7],[3.5,5.75]];
    draw_points(pts,dia=0.05);
        
    NumberBeams=30;  // NEED TO HAVE LOTS OF BEAMS
    L=7;
    LN = L/NumberBeams;
    t=0.062;
    w=1.06;
    Fy=1.203 * 0.96; // lbs
    
    Load_Steps = 10;  // SUPER SENSITIVE TO STEPS

    BEAM1 = [[Qbeam,LN,t,w,ang_fixed], for (i=[1:NumberBeams-1]) [Qbeam,LN,t,w,0]];
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[0,Fy,0]]);
    
    draw_beam_undeformed(BEAM1);
    
    Do_Analysis(BEAM1,LOADS1,force_scale,Display_steps,Failure_Stress,E_PSI,density,steps=Load_Steps);
} else if (ACTIVE_BEAM == 8) { // TEST BEAM WITH FORCE AND REACTION
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
        
} else if (ACTIVE_BEAM == 9) { // Compression Test Column, 6 seg
    //  Euler Column Load Limit is about 3 lb for t = 0.05,  L = 3
    L=3;
    t=0.05;
    LN = L/NumberBeams;
    Fx=-3.1; // lbs
    
    // SENSITIVE TO THE NUMBER OF BEAMS AND STEPS!!!
    
    BEAM1 = [[Qbeam,LN,t,w,0.01], for (i=[1:NumberBeams-1]) [Qbeam,LN,t,w,0]];
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,0,0]]);
   
    draw_beam_undeformed(BEAM1);

    Do_Analysis(BEAM1,LOADS1,force_scale*0.5,Display_steps,Failure_Stress,E_PSI,density,steps=10);
} else if (ACTIVE_BEAM == 10) { // FRAME OF BEAMS from points
    t=.06;  
    w=0.5;
    HGT = 6;  // height of overall frame
    WIDTH = 2; // width of frame
    ORIGIN = [0,0,0];

    pts_UP=[[0,0],[0,HGT]]; 
    pts_UP_ADD = addPoints(pts_UP,0.5);  // add points at spacing of second parameter
    BEAM_UP = beamFromNodes(pts_UP_ADD,t,w,true);  // creates the beam elements
    *color("blue") draw_points(pts_UP_ADD,dia=0.1);
    *draw_beam_undeformed(BEAM_UP); 

    pts_OVER =[[0,0],[0,-WIDTH]];  // HAVE TO ROTATE -90 TO GET STARTING ANGLE = -90
    pts_OVER_ADD = addPoints(pts_OVER,0.5);  // add points at spacing of second parameter
    BEAM_OVER = beamFromNodes(pts_OVER_ADD,t*4,w,false);  // creates the beam elements
    *color("yellow") draw_points(pts_OVER_ADD,dia=0.08);
    *draw_beam_undeformed(BEAM_OVER); 

    pts_DOWN=[[WIDTH,HGT],[WIDTH,0]]; 
    pts_DOWN_ADD = addPoints(pts_DOWN,0.5);  // add points at spacing of second parameter
    BEAM_DOWN = beamFromNodes(pts_DOWN_ADD,t,w,true);  // creates the beam elements
    *color("purple") draw_points(pts_DOWN_ADD,dia=0.07);

    BEAM1 = concat(BEAM_UP,concat(BEAM_OVER,BEAM_DOWN));
    NumberBeams = len(BEAM1);
    echo(BEAM1=BEAM1," n=", len(BEAM1));
        
    Fx = 1;
    Mz = 0; // Fx*HGT/2;
    //echo(Mz=Mz);
    //LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,0,Mz]]);
    LOADS1 = concat ( concat (
    concat([for (i=[1:(NumberBeams/2)-1]) [0,0,0]],[[Fx,0,Fx*HGT/2]]), // first leg & top load
        [for (j=[1:(NumberBeams/2)]) [0,0,0]]), // second leg
            [[-Fx*0.515,-0.04,Fx*HGT*.245]]); // << last load is back solved to hold
       
    //echo(LOADS1=LOADS1," n=", len(LOADS1));

    translate(ORIGIN) draw_beam_undeformed(BEAM1); 
    
    // ~Stress level at which the part will fail (PSI)
    FAILURE_STRESS = 10000;
    // This could be tensile failure, compression failure, bending, etc.
    // material density (lb per inch^3)
    DENSITY = 0.043;

    Do_Analysis(BEAM1,LOADS1,force_scale,Display_steps,FAILURE_STRESS,E_PSI,DENSITY,ORIGIN,steps=4);
        
    // FUNCTION CALLS TO GET FINAL NODES:
    StartingNodes = getNodesFromBeams(BEAM1,ORIGIN[0],ORIGIN[1]);  // DOES NOT MATCH NEW POINTS
    initial_loads = spread_ext_loads(LOADS1); // Spread Loads
    beam_angles = global_angles(BEAM1); // Generate GLOBAL Beam ANGLES, undeformed
    FinalNodes = GetFinalNodes(BEAM1,FAILURE_STRESS,E_PSI,DENSITY,initial_loads, ORIGIN, STEPS=4);
    NODE_NUM = NumberBeams-12;
    THING(StartingNodes,NODE_NUM,LEN=1);
    TranslateChildren(StartingNodes,FinalNodes,NODE_NUM) THING(StartingNodes,NODE_NUM,LEN=1);
} else if (ACTIVE_BEAM == 11) { // PARALLEL FLEXTURE SYSTEM, PAPER AIRPLANE LAUNCHER
    t=.06;  // individual beam thickness, minimum
    w=0.5;  // width of beam (3d printing thickness
    L=5;  // total length of beams
    LN = L/NumberBeams;
    Fx=0;
    Fy=-1.0; // lbs
    Mz = -Fy*L/2;
    START_ANG = 15;
    
    //BEAM1 = [[Qbeam,LN,t,w,ang_fixed], for (i=[1:NumberBeams-1]) [Qbeam,LN,t,w,0]];
    
    BEAM1 = [[11111, 0.3125, t*1.0, w, START_ANG], 
    [11111, 0.3125, t*1.0, w, 0], 
    [11111, 0.3125, t, w, 0], 
    [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], [11111, 0.3125, t, w, 0], 
    [11111, 0.3125, t, w, 0], 
    [11111, 0.3125, t*1.0, w, 0], 
    [11111, 0.3125, t*1.0, w, 0]] ;
    
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,Fy,Mz]]);
   
    BEAM_SPACE = 1.5;
    START_Y = L*sin(START_ANG)/2;
    START_X_ANG = L*cos(START_ANG) - L;
    
    CAP_LEN = 4.0;
    CAP_X = 0.7; 
    CAP_Y = -CAP_LEN + BEAM_SPACE + .2;
    
    LATCH_H = 0.3;
    
    ORIGIN = [0,0,0]; // [0,-START_Y,0];

    // Three flex beams
    FLEX_BEAM_FILLETED(BEAM1,ORIGIN,L,BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=0.15);
    FLEX_BEAM_FILLETED(BEAM1,[0,BEAM_SPACE/2-START_Y,0],L,BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=0.15);
    FLEX_BEAM_FILLETED(BEAM1,[0,BEAM_SPACE-START_Y,0],L,BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=0.15);
    
    translate([-CAP_X,CAP_Y-START_Y-.1,-w/2]) cube([CAP_X,CAP_LEN+.1,w]); // left cap

    translate([L+START_X_ANG,CAP_Y+START_Y,-w/2]) launcher(CAP_X,CAP_LEN,w,LATCH_H);
    
    translate([L-0.4,-2.2,-w/2]) {
        rotate([0,0,180]) LATCH(LATCH_H,LATCH_H,w); // fixed latch
        translate([-0.06,-0.3,0]) cylinder(w,r=0.06,center=false,$fn=32);
        translate([-LATCH_H-.06,-1.45,0]) cube([0.06,1.4,w]); // flexure for fixed latch
        translate([-LATCH_H-1.06,-0.25,0]) cube([1,0.2,w]); // trigger for fixed latch
    }
    
    translate([-.7,CAP_Y-CAP_X-START_Y,-w/2]) 
        cube([L-.05,CAP_X-.1,w]); // upright for fixed latch

    // // ANALYSIS, DON'T INCLUDE ANYTHING BELOW THIS LINE IN PRINT 
    Load_Steps = 6;
    translate(ORIGIN) Do_Analysis(BEAM1,LOADS1,force_scale,true,Failure_Stress,E_PSI,density,steps=Load_Steps) {launcher(CAP_X,CAP_LEN,w,LATCH_H);} ;
    
    // THREE FUNCTION CALLS TO GET FINAL NODES:
    initial_loads = spread_ext_loads(LOADS1); // Spread Loads
    beam_angles = global_angles(BEAM1); // Generate GLOBAL Beam ANGLES, undeformed
    FinalNodes = GetFinalNodes(BEAM1,Failure_Stress,E_PSI,density,initial_loads, ORIGIN, STEPS=Load_Steps);
    
    StartingNodes = getNodesFromBeams(BEAM1,ORIGIN[0],ORIGIN[1]);

    TranslateChildren(StartingNodes,FinalNodes,16) {
        translate([L+START_X_ANG,CAP_Y+START_Y,-w/2]) 
            color("yellow",0.5) launcher(CAP_X,CAP_LEN,w,LATCH_H); };
//
} else if (ACTIVE_BEAM == 12) {  // CROSS FRAME OF BEAMS from points
    t=.06;  
    w=0.5;
    HGT = 6;  // height of overall frame
    WIDTH = 2; // width of frame
    ORIGIN = [0,0,0];

    pts_UP=[[0,0],[WIDTH,HGT]]; 
    pts_UP_ADD = addPoints(pts_UP,0.5);  // add points at spacing of second parameter
    BEAM_UP = beamFromNodes(pts_UP_ADD,t,w,true);  // creates the beam elements
    *color("blue") draw_points(pts_UP_ADD,dia=0.1);
    *draw_beam_undeformed(BEAM_UP); 

    CROSS_ANG = atan2(WIDTH,HGT) + 90;
    pts_OVER =[[0,0],[WIDTH*cos(CROSS_ANG),WIDTH*sin(CROSS_ANG)]];  // HAVE TO ROTATE 
    pts_OVER_ADD = addPoints(pts_OVER,0.5);  // add points at spacing of second parameter
    BEAM_OVER = beamFromNodes(pts_OVER_ADD,t*4,w,false);  // creates the beam elements
    *color("yellow") draw_points(pts_OVER_ADD,dia=0.08);
    *draw_beam_undeformed(BEAM_OVER); 

    pts_DOWN=[[0,0],[-WIDTH,HGT]]; 
    pts_DOWN_ADD = addPoints(pts_DOWN,0.5);  // add points at spacing of second parameter
    BEAM_DOWN = beamFromNodes(pts_DOWN_ADD,t,w,true);  // creates the beam elements
    *color("purple") draw_points(pts_DOWN_ADD,dia=0.07);

    BEAM1 = concat(BEAM_UP,concat(BEAM_OVER,BEAM_DOWN));
    NumberBeams = len(BEAM1);
    echo(BEAM1=BEAM1," n=", len(BEAM1));
        
    Fx = 0.5;
    Mz = 0; // Fx*HGT/2;
    //echo(Mz=Mz);
    //LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,0,Mz]]);
    LOADS1 = concat ( concat (
    concat([for (i=[1:(NumberBeams/2)-1]) [0,0,0]],[[Fx,0,0]]), // first leg & top load
        [for (j=[1:(NumberBeams/2)]) [0,0,0]]), // second leg
            [[-Fx*.245,Fx*.47,0.88]]); // << last load is back solved to hold
       
    //echo(LOADS1=LOADS1," n=", len(LOADS1));

    translate(ORIGIN) draw_beam_undeformed(BEAM1); 
    
    // ~Stress level at which the part will fail (PSI)
    FAILURE_STRESS = 10000;
    // This could be tensile failure, compression failure, bending, etc.
    // material density (lb per inch^3)
    DENSITY = 0.043;

    Do_Analysis(BEAM1,LOADS1,force_scale,Display_steps,FAILURE_STRESS,E_PSI,DENSITY,ORIGIN,steps=4);
        
    // FUNCTION CALLS TO GET FINAL NODES AND TRANSFORM THING:
    StartingNodes = getNodesFromBeams(BEAM1,ORIGIN[0],ORIGIN[1]);  // DOES NOT MATCH NEW POINTS
    initial_loads = spread_ext_loads(LOADS1); // Spread Loads
    beam_angles = global_angles(BEAM1); // Generate GLOBAL Beam ANGLES, undeformed
    FinalNodes = GetFinalNodes(BEAM1,FAILURE_STRESS,E_PSI,DENSITY,initial_loads, ORIGIN, STEPS=4);
    NODE_NUM = NumberBeams-17;
    THING(StartingNodes,NODE_NUM,LEN=1);
    TranslateChildren(StartingNodes,FinalNodes,NODE_NUM) THING(StartingNodes,NODE_NUM,LEN=1);  //
} else if (ACTIVE_BEAM == 13) { // Compression leaf spring segment:
    L=220; // mm
    Y = 15; // mm, initial offset of wave
    t=1.4;  // mm, individual beam thickness, minimum
    w=15.0;  // mm, width of beam (3d printing z-direction)
    NumberBeams = 20;
    LN = L/NumberBeams;
    Fx=-1.7; // Newtons
    
    pts=[for (i=[0:NumberBeams]) [L*(i/NumberBeams),Y*sin(360*(i/NumberBeams)-90) + Y] ];
    //draw_points(pts,dia=2);
    
    BEAM1 = beamFromNodes(pts,t,w,THICKEN_ENDS=true);

    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,0,-18.630*Fx]]);
   
    draw_beam_undeformed(BEAM1);

    Do_Analysis(BEAM1,LOADS1,force_scale*0.5,Display_steps,FAILURE_STRESS_PETG_METRIC,E_PETG_NSMM,DENSITY_PETG_METRIC,steps=10);
} else if (ACTIVE_BEAM == 99 ) {  // Junk case
    LDB_DEF = 5;
    echo("JUNK TEST CASE");
    Do_Analysis(LDB_DEF,force_scale,Display_steps,Failure_Stress,E_PSI,density);
} 

module launcher(CAP_X=0.7,CAP_LEN=4,w=0.5,LATCH_H=0.3) {
    difference() {
        translate([0,0,0]) 
            cube([CAP_X,CAP_LEN,w]);    // right cap
        translate([.1,.1,w/2])
            VEE_SLOT(CAP_X-.075,w,CAP_LEN); // slot in right cap. Cut exit end out!
    }
    translate([-LATCH_H,1.3,0]) 
        LATCH(LATCH_H,LATCH_H,w); // moving latch
}
//launcher();

module VEE_SLOT(X=1,Y=1,LEN=10) {
    X_TIP = X/14;
    translate([X/1.5,LEN,-Y/2]) rotate([90,0,-6]) linear_extrude(LEN,convexity=10) {
        polygon([[0,Y/2-X_TIP],[X,0],[X,Y],[0,Y/2+X_TIP]]);
    }
    
    //translate([X,0,-Y]) rotate([0,0,6]) cube([X,LEN*1.5,2*Y]);
};
//VEE_SLOT();

module LATCH(X=1,Y=1,LEN=2) {
    translate([0,0,0]) linear_extrude(LEN,convexity=10) {
        polygon([[0,Y-Y/10],[X,Y/10],[X,Y*.9],[0,Y]]);
    }
};
//LATCH();
module FLEX_BEAM_FILLETED(BEAM,ORG,LEN,BEAM_T,BEAM_W,BEAM_ANG,R) {
    translate(ORG) draw_beam_undeformed(BEAM);
    translate(ORG) BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R);
    X_END = LEN*cos(BEAM_ANG);
    Y_END = LEN*sin(BEAM_ANG);
    translate(ORG) translate([X_END,Y_END,0]) rotate([0,0,180]) BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R);
}

module BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R) {
    Y_FILR = R + BEAM_T/2;
    Y_ANG = R*sin(BEAM_ANG);
    X_FILR = R;
    TRAPIZOID = [[X_FILR,Y_FILR+Y_ANG] , [-X_FILR,Y_FILR-Y_ANG] , [-X_FILR,-Y_FILR-Y_ANG] , [X_FILR,-Y_FILR+Y_ANG]];
    
    translate([0,0,-BEAM_W/2]) difference() {
        linear_extrude(BEAM_W,convexity=10)
            polygon(TRAPIZOID);
        translate([X_FILR,Y_FILR+Y_ANG,BEAM_W/2]) cylinder(h=2*BEAM_W,r=R,center=true,$fn=32);
        translate([X_FILR,-Y_FILR+Y_ANG,BEAM_W/2]) cylinder(h=2*BEAM_W,r=R,center=true,$fn=32);
    }
}
//translate([-2,0,0]) BEAM_FILLETS(BEAM_T=0.1,BEAM_W=1,BEAM_ANG=15,R=0.1);