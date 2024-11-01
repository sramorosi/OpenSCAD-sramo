// Paper Airplane Launcher Model, by SrAmo 2024
// One piece.  No rubber bands.
// Can be printed on a Mini 3d printer (180x180 mm bed)
//    The build volume of a Prusa MINI printer is 180x180x180 mm
// Note: Import .STL into MS 3D Builder, REPAIR, AND ROTATE IN YAW 84 DEGREES, before slicing
// Designed for PLA or PETG filament.  
// For this design, when latched:
// PETG has greater margin to failure than PLA
// PLA has greater potential energy (35 N-mm) than PETG (30.4 N-mm)
include <NEW_LDB_Indexes.scad>
use <NEW_LDB_Modules.scad>
use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>

// Scale of Force & Moment Display
force_scale = 0.1; // [0.05:.05:2.0]

// Units
UNITS = "METRIC, LENGTH = MM, FORCE = NEWTONS";

TITLE = "AIRPLANE LAUNCHER July 24"; // UPDATE DATE BEFORE EXPORTING

// MATERIAL PROPERTIES
MATERIAL = "PLA";
//MATERIAL = "PETG";

echo(str("UNITS ARE ",UNITS,", MATERIAL IS ",MATERIAL));

E_NSMM = 2344;  // Modulus of Elasticity (NEWTONS PER mm^2), PLA = 2344
//E_NSMM = 2068;  // Modulus of Elasticity (NEWTONS PER mm^2), PETG = 2068

FAILURE_STRESS_METRIC = 50;  // ~Stress level at which PLA will fail (NEWTONS per mm^2)
//FAILURE_STRESS_METRIC = 60;  // ~Stress level at which PETG will fail (NEWTONS per mm^2)
// This could be tensile failure, compression failure, bending, etc.

DENSITY_METRIC = 0.0012318;  // material density (gram per mm^3) PLA
//DENSITY_METRIC = 0.0012733;  // material density (gram per mm^3) PETG

// PARALLEL FLEXTURE SYSTEM, PAPER AIRPLANE LAUNCHER
t=1.5;  // individual beam thickness, minimum
w=15.0;  // width of beam (3d printing z-direction)
L=150;  // total length of beams

Fx=0;
// Force to pull launcher to trigger for PETG is 19.2 Newtons, for PLA is 22 N
Fy=-5.5; // Newtons.  For PETG use -5.5, for PLA use -4.8
// measured 24 (/4) = 6 Newtons per beam
Mz = -Fy*L*.95/2;
ORIGIN = [0,0,0];  // USED IF BEAM DOES NOT START AT 0,0,0

START_ANG=0;  // USED IF BEAM ENDS ARE NOT TO BE HORIZONTAL

Load_Steps = 6;

LATCH_H = 7.6; // mm,  HEIGHT OF THE LATCH
FLEX_LEN = 22; // mm, Length of flex beam  for the latch
BEAM_SPACE = 8; // mm, DISTANCE BETWEEN THE FLEX BEAMS
Rfillet = 3.0; // mm
//START_Y = L*sin(START_ANG)/2;
//START_X_ANG = L*cos(START_ANG) - L;

CAP_LEN = 120; // mm, Length of rigid beams, USED IN MANY PLACES
CAP_X = 18; // mm, Thickness of rigid beams, USED IN MANY PLACES

LAUNCHER_Y_SHIFT = 0.76;  // Adjust based on CAP_LEN

module SOLVE_STARTING_SHAPE () {
    // STEP 1, CREATE STARTING SHAPE
    pts=[[0,0],[L*cos(START_ANG),L*sin(START_ANG)]];  // starting shape = line
    new_pts = addPoints(pts,5.0);  // divide pts into more pts, second value is max pt distance
    //echo(new_pts=new_pts);
    color("blue") draw_points(new_pts,dia=1);

    BEAM1 = beamFromNodes(new_pts,t,w,true); 
    NumberBeams = len(BEAM1);
    //echo(BEAM0=BEAM0," n=", NumberBeams);
    Fy_HALF = -Fy/2;
    Mz_HALF = -Fy_HALF*L*.95/2;
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[0,Fy_HALF,Mz_HALF]]);
       
    //echo(LOADS1=LOADS1," n=", len(LOADS1));

    draw_beam_undeformed(BEAM1); 
    
    Do_Analysis(BEAM1,LOADS1,force_scale,false,FAILURE_STRESS_METRIC,E_NSMM,DENSITY_METRIC,ORIGIN,steps=Load_Steps);

    FinalNodes = GetFinalNodes(BEAM1 , FAILURE_STRESS_METRIC , E_NSMM , DENSITY_METRIC , LOADS1 , ORIGIN, STEPS=Load_Steps);
    
    echo(FinalNodes=FinalNodes);  // COPY AND PASTE THESE POINTS FROM LOG WINDOW INTO STEP_1_PTS BELOW
}
*SOLVE_STARTING_SHAPE();  // Not for exporting/printing

module AIRPLANE_LAUNCHER (PRINT = false) {
    // STEP 2, USE STEP 1 RESULTS (paste FinalNodes below) AND CODE BUILDS THE REST OF THE MODEL
    // Set PRINT = true when doing F6 Render and F7 Export STL for 3D printing

    STEP_1_PTS =    [[0, 0, 1.17029], [4.99851, 0.102111, 2.3128], [9.98873, 0.407483, 2.34769], [14.9621, 0.91808, 2.36953], [19.9101, 1.63426, 2.37589], [24.8242, 2.55458, 2.36409], [29.6965, 3.67556, 2.3311], [34.5198, 4.99152, 2.27356], [39.2883, 6.49439, 2.18768], [43.9976, 8.1735, 2.06919], [48.6457, 10.0154, 1.91325], [53.2358, 11.9977, 1.56877], [57.7763, 14.0915, 1.22733], [62.2774, 16.2687, 0.888432], [66.7508, 18.5023, 0.551494], [71.2092, 20.7658, 0.215854], [75.6657, 23.033, -0.119193], [80.1335, 25.278, -0.45437], [84.6254, 27.4742, -0.790396], [89.1533, 29.595, -1.12796], [93.7281, 31.6126, -1.4677], [98.3586, 33.4986, -1.81018], [103.051, 35.2231, -2.15584], [107.808, 36.7641, -2.29235], [112.623, 38.1091, -2.39299], [117.491, 39.2488, -2.46243], [122.404, 40.1764, -2.50478], [127.352, 40.8874, -2.52371], [132.328, 41.3797, -2.52249], [137.32, 41.6532, -2.50405], [142.319, 41.7097, -2.47102]];
    
    //color("blue") draw_points(STEP_1_PTS,dia=0.03);
    
    BEAM1 = BEAM_FROM_NODES(nodes=STEP_1_PTS,TBEAMS=t,TENDS=t,w=w,THICKEN_ENDS=true, T_MID=false);  // CREATES THE BEAM ELEMENTS FROM POINTS BEAM_FROM_NODES(nodes,TBEAMS,TENDS,w,THICKEN_ENDS=false,T_MID=false,TUP = 1.03,S=9,index=0,prior_ang=0)

    NumberBeams = len(BEAM1)-2;
    LOADS1 = concat([for (i=[1:NumberBeams]) [0,0,0]],[[Fx,Fy,Mz]]);

    // Set the beam thickness to be a Minimum MS of:
    //MSFLOOR = 0.44;
    //BEAM1 = SetBeamMinMS(BEAM0 ,LOADS1 , MSFLOOR, FAILURE_STRESS_METRIC , E_NSMM ,DENSITY_METRIC , ORIGIN, STEPS=Load_Steps);
    
    StartingNodes = getNodesFromLDB(LDB=BEAM1,NB = NumberBeams);  // SHOULD  BE THE SAME AS STEP_1_PTS ?ANGLES DON'T MATCH
    //echo(StartingNodes=StartingNodes);
    //color("red") draw_points(StartingNodes,dia=0.03);
    
    end_point = [STEP_1_PTS[NumberBeams][Nx],STEP_1_PTS[NumberBeams][Ny],0];

    // ANALYSIS, DON'T INCLUDE IN PRINT 
    if(!PRINT) {
        translate(ORIGIN) Do_Analysis(BEAM1,LOADS1,force_scale,false,FAILURE_STRESS_METRIC,E_NSMM,DENSITY_METRIC,steps=Load_Steps);

        // GET FINAL NODES:
        FinalNodes = GetFinalNodes(BEAM1,FAILURE_STRESS_METRIC,E_NSMM,DENSITY_METRIC,LOADS1, ORIGIN, STEPS=Load_Steps);


            // Launcher, Deformed, DON'T INCLUDE IN PRINT 
            TranslateChildren(StartingNodes,FinalNodes,NumberBeams) 
            translate(end_point)  color("yellow",0.5) {
                *LAUNCHER(CAP_X,CAP_LEN,w,LATCH_H,-CAP_LEN*LAUNCHER_Y_SHIFT); 
                THIN_LAUNCHER(3,V_LEN,w,LATCH_H,-CAP_LEN*LAUNCHER_Y_SHIFT);
            }
        }

    // Four flex beams, not deformed
    FLEX_BEAM_FILLETED(BEAM1,ORIGIN,BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=Rfillet, NODES=STEP_1_PTS, SPACE = BEAM_SPACE,CYL=true);
    
    FLEX_BEAM_FILLETED(BEAM1,[0,BEAM_SPACE,0],BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=Rfillet, NODES=STEP_1_PTS, SPACE = BEAM_SPACE,CYL=true);
    
    FLEX_BEAM_FILLETED(BEAM1,[0,BEAM_SPACE*2,0],BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=Rfillet, NODES=STEP_1_PTS, SPACE = BEAM_SPACE,CYL=true);
    
    FLEX_BEAM_FILLETED(BEAM1,[0,BEAM_SPACE*3,0],BEAM_T=t,BEAM_W=w,BEAM_ANG=START_ANG,R=Rfillet, NODES=STEP_1_PTS, SPACE = BEAM_SPACE,CYL=false);

    // Launcher, Not deformed
    translate(end_point) {
        *LAUNCHER(CAP_X,CAP_LEN,w,LATCH_H,-CAP_LEN*LAUNCHER_Y_SHIFT);
        THIN_LAUNCHER(3,V_LEN,w,LATCH_H,-CAP_LEN*LAUNCHER_Y_SHIFT);
    }
    
    Base();

    // END OF MAIN
    //
}
AIRPLANE_LAUNCHER(PRINT = true);

// Prusa Mini Print Bed (DON'T PRINT)
*color("yellow") 
    translate([-18,-100,-10]) 
        cube([180,180,2]);

module Base() {
    // Base constants
    R_ARC = 25; // inside radius

    // ADJUST TRIGGER X Y TO GET GOOD ENGAGEMENT (TO YELLOW PART)
    X=129;
    Y=-80;

    translate([X,Y,-w/2]) 
        Trigger(LATCH_H,w,t*1.3,FLEX_LEN);

    // left cap
    translate([-CAP_X,Y+R_ARC,-w/2]) 
        cube([CAP_X,4*BEAM_SPACE + (-Y-R_ARC),w]);  
    
    // arc
    translate([R_ARC,Y+R_ARC,-w/2]) 
        rotate([0,0,180]) 
        curved_beam(rot1 = 90, radOfCurve = R_ARC, t1 = CAP_X, t2 = w,$fn=60); 
        
    // upright for latch
    translate([R_ARC,Y-CAP_X,-w/2]) 
        difference() {
            UP_X = X+t-R_ARC;
            cube([X+t-R_ARC,CAP_X,w]); 
            translate([UP_X*.8,-UP_X*.42,0]) cylinder(h=3*w,d=UP_X*.9,center=true,$fn=100);
        }
        
    // ADD TEXT 
    color("yellow") {
    SZ = 5.4;  // Text height
    translate([0,3.8*BEAM_SPACE,w/2]) 
        rotate([0,0,-85])
        linear_extrude(1) {
            translate([0,-SZ-9,0]) 
                text(str(TITLE),size=SZ,font="Liberation Sans:style=Bold Italic");
        }
        *translate([0,0,w/2]) linear_extrude(1) {
            translate([20,-90,0]) 
                text("for Ilvy",size=6,font="Liberation Sans:style=Bold Italic");
        }

    }
}
*Base();

module Trigger(HLATCH,W,T,FLEXL) {
    translate([HLATCH,FLEXL+HLATCH,0]) {
        // Trapizoidal latch
        rotate([0,0,180]) translate([0,0.3,0])
            LATCH(HLATCH,HLATCH,W); 
        translate([-W/10,-HLATCH,0]) 
            cylinder(W,r=W/10,center=false,$fn=32); // Knob on end
        
        // flexure for fixed latch
        translate([-HLATCH-1.5*T,-FLEXL-HLATCH,0]) cube([T,FLEXL,W]);
        translate([-HLATCH-T,-HLATCH,W/2]) rotate([0,0,-90]) 
            BEAM_FILLETS(BEAM_T=T,BEAM_W=W,BEAM_ANG=0,R=T);
        translate([-HLATCH-T,-HLATCH-FLEXL,W/2]) rotate([0,0,90]) 
            BEAM_FILLETS(BEAM_T=T,BEAM_W=W,BEAM_ANG=0,R=T);
        
        // lever for latch (arc + arc)
        R = 18;
        ANG = 50;
        translate([-HLATCH,-R,0]) {
            rotate([0,0,ANG]) { 
                rotate([0,0,-90]) 
                curved_beam(rot1 = -ANG, radOfCurve = -R, t1 = HLATCH, t2 = w,$fn=60); 

            translate([0,2*R,0]) 
                rotate([0,0,-90]) 
                    curved_beam(rot1 = -110, radOfCurve = R, t1 = HLATCH, t2 = w,$fn=60); 
            }
        }
    }
}
*Trigger(LATCH_H,w,t*1.3,FLEX_LEN);

T_X = 120; // mm, Length of Thin launcher
T_Y = 5; // mm, width of Thin launcher
T_Z = 17; // mm, z height of launcher portion

module THIN_PAPER_HOLDER(X=4,Y=0.5,Z=0.7,WALL=1) { 
   translate([0,WALL,0])
    color("orange") {
        difference() {
            translate([0.1,0.1,-0.1])
                cube([X+WALL,Y+WALL,Z+WALL],center=false); 
            translate([-2*WALL,0,3*WALL])
                cube([X,Y,Z],center=false); // cube2
            
            //scale([1.01,1.01,1.01])
            THIN_OUTER(X=T_X,Y=T_Y,Z=T_Z,WALL=1.01,CORNER=true); 

        }
    }

}
*THIN_PAPER_HOLDER(Z=T_Z,X=T_X,Y=T_Y); // not for print

module THIN_OUTER(X=10,Y=2,Z=3,WALL=1,CORNER=true) {
    translate([0,0,WALL]) color("green") {
        difference() {
            cube([X,Y,Z],center=false); 
            
            translate([-WALL,WALL,WALL])
                cube([X,Y,Z],center=false); // cube2
        }
        // corner is for fit
        // leave corner off for printing
        if(CORNER)
        translate([X-WALL,Y/2,WALL]) rotate([0,45,0])
            cube([WALL,Y,WALL],center=true);
    }
}
translate([10,-10,-CAP_X/2+1.5]) rotate([90,0,0]) 
    THIN_OUTER(X=T_X,Y=T_Y,Z=T_Z,WALL=1,CORNER=false);  // for printing (REQUIRED)

module VEE_SLOT2(X=1,Y=1,LEN=10) {
    // x is vee depth, y is vee width
    X_TIP = .7;
    translate([0,-Y/2,0])
    linear_extrude(LEN,convexity=10) {
        polygon([[0,Y/2-X_TIP],[X,0],[X,Y],[0,Y/2+X_TIP]]);
    }
};


V_LEN = 120; // mm, Length of rigid beams, USED IN MANY PLACES
//CAP_X = 18; // mm, Thickness of rigid beams, USED IN MANY PLACES

module THIN_LAUNCHER(CAPX=0.7,LEN=4,W=0.5,HLATCH=0.3,YSHIFT=0) {
    translate([0,YSHIFT,-W/2]) {
            cube([CAPX,LEN+1,W]);  // right cap
            translate([-HLATCH,LEN/2+12,0]) 
                LATCH(HLATCH,HLATCH,W); // moving latch
        }
    translate([2,29.9,-0.4]) rotate([-90,0,-90]) 
        THIN_PAPER_HOLDER(Z=T_Z,X=T_X,Y=T_Y); 
}
*THIN_LAUNCHER(3,V_LEN,w,LATCH_H,-CAP_LEN*LAUNCHER_Y_SHIFT); // NOT FOR PRINT

// May 2 2024 paper holder launcher
module LAUNCHER(CAPX=0.7,LEN=4,W=0.5,HLATCH=0.3,YSHIFT=0) {
    translate([0,YSHIFT,-W/2]) {
        difference() {
            cube([CAPX,LEN,W]);  // right cap
            translate([-0.6*CAPX,LEN*0.01,W/2]) 
                VEE_SLOT(CAPX-.075,W,LEN); // slot in right cap. 
            translate([CAPX,0,-W/2]) rotate([0,0,7.0]) 
                cube([CAPX,LEN*2,W*2]);  // remove top angle
        }
        translate([-HLATCH,LEN/2+12,0]) 
            LATCH(HLATCH,HLATCH,W); // moving latch
    }
}
*LAUNCHER(CAP_X,V_LEN,w,LATCH_H,-CAP_LEN*LAUNCHER_Y_SHIFT); // NOT FOR PRINT

module VEE_SLOT(X=1,Y=1,LEN=10) {
    X_TIP = X/30;
    translate([X/1.5,LEN,-Y/2]) rotate([90,0,0]) linear_extrude(LEN,convexity=10) {
        polygon([[0,Y/2-X_TIP],[X,0],[X,Y],[0,Y/2+X_TIP]]);
    }
    
    //translate([X,0,-Y]) rotate([0,0,6]) cube([X,LEN*1.5,2*Y]);
};
*VEE_SLOT(CAP_X-.075,w,CAP_LEN);

module LATCH(X=1,Y=1,LEN=2) {
    translate([0,0,0]) linear_extrude(LEN,convexity=10) {
        polygon([[X,0],[X,Y*.9],[0.1*X,Y],[0,Y*0.9],[0,Y-Y*.3]]);
    }
};
*LATCH();

module FLEX_BEAM_FILLETED(BEAM,ORG,BEAM_T,BEAM_W,BEAM_ANG,R,NODES,SPACE,CYL=false) {
    NB = len(BEAM)-2;  // number of beams
    
    translate(ORG) MAKE_BEAM_UNDEFORMED(BEAM,BEAM_W);
    
    translate(ORG) BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R); // Fillet at start
    
    // Fillet at end
    end_point = [NODES[NB][Nx],NODES[NB][Ny],0];
    //echo(end_point=end_point,ORG=ORG);
    translate(ORG) translate(end_point) 
        rotate([0,0,180]) BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R);

    NEW_D = SPACE-BEAM_T*0.9;    
    if (CYL) {
       RING(floor(2*NB/12),NEW_D*1.13);
       RING(floor(3.2*NB/12),NEW_D*1.1);
       RING(floor(4.5*NB/12),NEW_D*1.03);
       //RING(floor(4*NB/6),NEW_D*1.06);
       //RING(floor(5*NB/6),NEW_D*1.14);
    }
    module RING(NODE_NUM,DIA_RING) {
        T_RING = 0.8;  // mm (2 x nozzle dia)
        POINT = [NODES[NODE_NUM][Nx],NODES[NODE_NUM][Ny],0];
        ROTANG = atan2(NODES[NODE_NUM][Ny]-NODES[NODE_NUM-1][Ny],NODES[NODE_NUM][Nx]-NODES[NODE_NUM-1][Nx]);
        translate(ORG) 
            translate(POINT) 
            rotate([0,0,ROTANG]) 
            translate([0,SPACE/2-BEAM_T*.7,0]) 
            difference() {
                cylinder(d=DIA_RING,h=BEAM_W,center=true,$fn=64);
                cylinder(d=DIA_RING-2*T_RING,h=2*BEAM_W,center=true,$fn=64);
        }
    }
}

module BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R) {
    Y_FILR = R + BEAM_T/2;
    Y_ANG = R*sin(BEAM_ANG);
    X_FILR = R;
    TRAPIZOID = [[X_FILR,Y_FILR+Y_ANG] , [-X_FILR/3,Y_FILR-Y_ANG] , [-X_FILR/3,-Y_FILR-Y_ANG] , [X_FILR,-Y_FILR+Y_ANG]];
    
    translate([0,0,-BEAM_W/2]) difference() {
        linear_extrude(BEAM_W,convexity=10)
            polygon(TRAPIZOID);
        translate([X_FILR,Y_FILR+Y_ANG,BEAM_W/2]) cylinder(h=2*BEAM_W,r=R,center=true,$fn=48);
        translate([X_FILR,-Y_FILR+Y_ANG,BEAM_W/2]) cylinder(h=2*BEAM_W,r=R,center=true,$fn=48);
    }
}
*translate([-2,0,0]) BEAM_FILLETS(BEAM_T=0.1,BEAM_W=1,BEAM_ANG=15,R=0.1);