// Paper Airplane Launcher Model
include <LDB_Indexes.scad>
use <LDB_Modules.scad>
use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>

// Scale of Force & Moment Display
force_scale = 0.5; // [0.05:.05:2.0]
// MATERIAL PROPERTIES. 
// Modulus of Elasticity (PSI), PLA=340000,PETG=300000,Polycar=320000
E_PSI = 320000; // MUST MODIFY IN MODULES 
// ~Stress level at which the part will fail (PSI)
Failure_Stress = 10000;
// This could be tensile failure, compression failure, bending, etc.
// material density (lb per inch^3)
density = 0.045;

// Number of Beams
//NumberBeams=16; // [2:1:40]

module AIRPLANE_LAUNCHER () {
    // PARALLEL FLEXTURE SYSTEM, PAPER AIRPLANE LAUNCHER
    t=.06;  // individual beam thickness, minimum
    w=0.6;  // width of beam (3d printing thickness
    L=5.8;  // total length of beams
    //LN = L/NumberBeams;
    Fx=0;
    Fy=-1.0; // lbs
    Mz = -Fy*L*.95/2;
    START_ANG = 0;
    ORIGIN = [0,0,0]; // [0,-START_Y,0];

    new_pts =   [[0, 0, 0], [0.305121, 0.00779123, 2.87013], [0.609343, 0.0322647, 3.38469], [0.911355, 0.0761489, 3.92757], [1.20931, 0.142091, 4.46482], [1.50183, 0.229107, 3.71097], [1.78882, 0.333017, 2.96755], [2.07082, 0.449846, 2.23425], [2.34886, 0.575826, 1.50965], [2.62433, 0.707366, 0.791531], [2.8988, 0.840991, 0.0771391], [3.17391, 0.973275, -0.636576], [3.45128, 1.10076, -1.35271], [3.73232, 1.2199, -2.07418], [4.01816, 1.32696, -2.80347], [4.30946, 1.41803, -3.54236], [4.60628, 1.48899, -4.29171], [4.90739, 1.53877, -3.79455], [5.21102, 1.56981, -3.28308], [5.51588, 1.58467, -2.79303]];
    
    //color("blue") draw_points(new_pts,dia=0.03);
    
    BEAM1 =  [[11111, 0.30522, 0.07986, 0.6, 1.46272], [11111, 0.305205, 0.0726, 0.6, 3.13659], [11111, 0.305184, 0.066, 0.6, 3.66825], [11111, 0.305165, 0.06, 0.6, 4.21173], [11111, 0.305188, 0.06, 0.6, 4.08688], [11111, 0.305222, 0.06, 0.6, 3.33754], [11111, 0.305243, 0.06, 0.6, 2.59988], [11111, 0.305249, 0.06, 0.6, 1.87169], [11111, 0.305265, 0.06, 0.6, 1.1497], [11111, 0.305269, 0.06, 0.6, 0.434017], [11111, 0.305261, 0.06, 0.6, -0.278886], [11111, 0.305265, 0.06, 0.6, -0.995635], [11111, 0.30525, 0.06, 0.6, -1.71117], [11111, 0.305232, 0.06, 0.6, -2.44013], [11111, 0.305204, 0.06, 0.6, -3.17223], [11111, 0.305184, 0.06, 0.6, -3.91577], [11111, 0.305197, 0.066, 0.6, -4.05788], [11111, 0.305212, 0.0726, 0.6, -3.55027], [11111, 0.305222, 0.07986, 0.6, -3.04645]];

    StartingNodes = getNodesFromBeams(BEAM1,ORIGIN[0],ORIGIN[1]);  // DOES NOT MATCH NEW POINTS
    //color("red") draw_points(StartingNodes,dia=0.03);

    NumberBeams = len(BEAM1);
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,Fy,Mz]]);

    BEAM_SPACE = 0.5;
    START_Y = L*sin(START_ANG)/2;
    START_X_ANG = L*cos(START_ANG) - L;

    CAP_LEN = 4.0;
    CAP_X = 0.7; 
    CAP_Y = -CAP_LEN + BEAM_SPACE*2 + .2;

    LATCH_H = 0.3;


    // ANALYSIS, DON'T INCLUDE IN PRINT 
    Load_Steps = 6;
    translate(ORIGIN) Do_Analysis(BEAM1,LOADS1,force_scale,true,Failure_Stress,E_PSI,density,steps=Load_Steps);

    // THREE FUNCTION CALLS TO GET FINAL NODES:
    initial_loads = spread_ext_loads(LOADS1); // Spread Loads
    beam_angles = global_angles(BEAM1); // Generate GLOBAL Beam ANGLES, undeformed
    FinalNodes = getFinalNodes(BEAM1,Failure_Stress,E_PSI,density,initial_loads,beam_angles,beam_angles, ORIGIN, STEPS=Load_Steps,index=Load_Steps);


    end_point = [StartingNodes[NumberBeams][Nx],StartingNodes[NumberBeams][Ny],0];

    // Launcher, Deformed, DON'T INCLUDE IN PRINT 
    TranslateChildren(StartingNodes,FinalNodes,NumberBeams) 
        translate(end_point)  color("yellow",0.5) LAUNCHER(CAP_X,CAP_LEN,w,LATCH_H); 

    // Three flex beams, not deformed
    Rfillet = 0.12;
    FLEX_BEAM_FILLETED(BEAM1,ORIGIN,BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=Rfillet, NODES=new_pts, SPACE = BEAM_SPACE,CYL=true);
    FLEX_BEAM_FILLETED(BEAM1,[0,BEAM_SPACE,0],BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=Rfillet, NODES=new_pts, SPACE = BEAM_SPACE,CYL=true);
    FLEX_BEAM_FILLETED(BEAM1,[0,BEAM_SPACE*2,0],BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=Rfillet, NODES=new_pts, SPACE = BEAM_SPACE);

    // Launcher, Not deformed
    translate([StartingNodes[NumberBeams][Nx],StartingNodes[NumberBeams][Ny],0]) 
               LAUNCHER(CAP_X,CAP_LEN,w,LATCH_H);

    // Base
    translate([-CAP_X,-1,-w/2]) cube([CAP_X,2.2,w]); // left cap 
    translate([2,-1,-w/2]) rotate([0,0,180]) 
        curved_beam(rot1 = 90, radOfCurve = 2, t1 = CAP_X, t2 = w,$fn=60); // arc
    translate([2,-3-CAP_X,-w/2]) cube([3,CAP_X,w]); // upright for latch

    translate([L-0.5,-1.8,-w/2]) Trigger(LATCH_H,w);
    // END OF MAIN
    //
}
AIRPLANE_LAUNCHER();

module Trigger(LATCH_H,W) {
    rotate([0,0,180]) LATCH(LATCH_H,LATCH_H,W); // fixed latch
    translate([-0.06,-0.3,0]) cylinder(W,r=0.06,center=false,$fn=32);
    translate([-LATCH_H-.056,-1.3,0]) cube([0.056,1.2,W]); // flexure for fixed latch
    translate([-LATCH_H-1.5,-0.23,0]) cube([1.5,0.2,W]); // trigger for fixed latch
}
//Trigger(LATCH_H,w);

module LAUNCHER(CAP_X=0.7,CAP_LEN=4,w=0.5,LATCH_H=0.3) {
    translate([0,-CAP_LEN+1.2,-w/2]) {
        difference() {
            translate([0,0,0]) 
                cube([CAP_X,CAP_LEN,w]);    // right cap
            translate([-0.3,.1,w/2])
                VEE_SLOT(CAP_X-.075,w,CAP_LEN); // slot in right cap. 
            translate([0.7,0,-w/2]) 
                rotate([0,0,6]) cube([CAP_X,CAP_LEN*2,w*2]);  // remove top angle
        }
        translate([-LATCH_H,2.0,0]) 
            LATCH(LATCH_H,LATCH_H,w); // moving latch
    }
}
//LAUNCHER();

module VEE_SLOT(X=1,Y=1,LEN=10) {
    X_TIP = X/14;
    translate([X/1.5,LEN,-Y/2]) rotate([90,0,0]) linear_extrude(LEN,convexity=10) {
        polygon([[0,Y/2-X_TIP],[X,0],[X,Y],[0,Y/2+X_TIP]]);
    }
    
    //translate([X,0,-Y]) rotate([0,0,6]) cube([X,LEN*1.5,2*Y]);
};
//VEE_SLOT();

module LATCH(X=1,Y=1,LEN=2) {
    translate([0,0,0]) linear_extrude(LEN,convexity=10) {
        polygon([[0,Y-Y*.2],[X,Y/10],[X,Y*.9],[0,Y]]);
    }
};
//LATCH();
module FLEX_BEAM_FILLETED(BEAM,ORG,BEAM_T,BEAM_W,BEAM_ANG,R,NODES,SPACE,CYL=false) {
    NB = len(BEAM);  // number of beams
    
    translate(ORG) draw_beam_undeformed(BEAM);
    
    translate(ORG) BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R); // Fillet at start
    
    // Fillet at end
    end_point = [NODES[NB][Nx],NODES[NB][Ny],0];
    translate(ORG) translate(end_point) 
        rotate([0,0,180]) BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R);

    NEW_D = SPACE-BEAM_T;    
    if (CYL) {
        // Circle in middle
        Nmid = floor(NB/2);
        mid_point = [NODES[Nmid][Nx],NODES[Nmid][Ny],0];
        Rmid = atan2(NODES[Nmid][Ny]-NODES[Nmid-1][Ny],NODES[Nmid][Nx]-NODES[Nmid-1][Nx]);
        translate(ORG) translate(mid_point) rotate([0,0,Rmid]) translate([0,SPACE/2-BEAM_T,0]) HOOP();

        Nless = floor(NB/2)-5;
        less_point = [NODES[Nless][Nx],NODES[Nless][Ny],0];
        Rless = atan2(NODES[Nless][Ny]-NODES[Nless-1][Ny],NODES[Nless][Nx]-NODES[Nless-1][Nx]);
        translate(ORG) translate(less_point) rotate([0,0,Rless]) translate([0,SPACE/2-BEAM_T,0]) HOOP();

        Nmore = floor(NB/2)+5;
        more_point = [NODES[Nmore][Nx],NODES[Nmore][Ny],0];
        Rmore = atan2(NODES[Nmore][Ny]-NODES[Nmore-1][Ny],NODES[Nmore][Nx]-NODES[Nmore-1][Nx]);
        translate(ORG) translate(more_point)rotate([0,0,Rmore]) translate([0,SPACE/2-BEAM_T,0]) HOOP();

        //cylinder(d=SPACE,h=BEAM_W,center=true,$fn=16);
    }
    module HOOP() {
        difference() {
            cylinder(d=NEW_D,h=BEAM_W,center=true,$fn=64);
            cylinder(d=NEW_D-2*BEAM_T,h=2*BEAM_W,center=true,$fn=64);
        }
    }
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