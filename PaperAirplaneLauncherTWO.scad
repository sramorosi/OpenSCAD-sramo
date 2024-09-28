// Paper Airplane Launcher parallel, by SrAmo 2024
// One piece.  No rubber bands.
// Note: Import .STL into MS 3D Builder, REPAIR, AND ROTATE IN YAW 40 DEGREES, before slicing
// Designed for PLA or PETG filament.  
// For this design, when latched:
// PETG has greater margin to failure than PLA
// PLA has greater potential energy (xx N-mm) than PETG (xx N-mm)
include <LDB_Indexes.scad>
use <LDB_Modules.scad>
use <../MAKE3-Arm/openSCAD-code/Robot_Arm_Parts_lib.scad>

// Scale of Force & Moment Display
force_scale = 0.1; // [0.05:.05:2.0]

// Units
UNITS = "METRIC, LENGTH = MM, FORCE = NEWTONS";

TITLE = "AIRPLANE LAUNCHER May 2 24"; // UPDATE DATE BEFORE EXPORTING

// MATERIAL PROPERTIES
//MATERIAL = "PLA";
MATERIAL = "PETG";

echo(str("UNITS ARE ",UNITS,", MATERIAL IS ",MATERIAL));

//E_NSMM = 2344;  // Modulus of Elasticity (NEWTONS PER mm^2), PLA = 2344
E_NSMM = 2068;  // Modulus of Elasticity (NEWTONS PER mm^2), PETG = 2068

//FAILURE_STRESS_METRIC = 50;  // ~Stress level at which PLA will fail (NEWTONS per mm^2)
FAILURE_STRESS_METRIC = 60;  // ~Stress level at which PETG will fail (NEWTONS per mm^2)
// This could be tensile failure, compression failure, bending, etc.

//DENSITY_METRIC = 0.0012318;  // material density (gram per mm^3) PLA
DENSITY_METRIC = 0.0012733;  // material density (gram per mm^3) PETG

Fx=-1.7; // Newtons, for running simulation

L=220; // mm, of flex beam
Y = 15; // mm, initial offset of beam wave
t=1.2;  // mm, individual beam thickness, minimum
Z_BEAM=20.0;  // mm, z height of beam (3d printing z-direction) WAS 15
NumberBeams = 45;  // was 30
LN = L/NumberBeams;

BEAM_SPACE = 20; // mm, space between two beams

// Support blocks on the ends of the beams
Rfillet = 8.0; // mm
END_T = 17;

// Tube support "square" size
W_SQR = 8; // mm,  8 mm square around 5 mm tube

V_LEN = 100; // mm, Length of V launcher
V_W = 5; // mm, width of V launcher
Z_LAUNCH = 45; // mm, z height of launcher portion
Z_VEE = 44; // mm, z height of vee in launcher

module AIRPLANELAUNCHERTWO() {
    intersection() { // make sure print fits on Mini printer
        
        rotate([0,0,45]) SHOOTER_SUB2();  // rotate 45!
            
        translate([0,0,-Z_BEAM]) 
            cube([180,180,2*Z_LAUNCH]); // Prusa Mini Bed Size

    };
}

AIRPLANELAUNCHERTWO();

*rotate([0,0,45]) translate([L-3.5,0,0]) // NOT FOR PRINT, TRANSFORMATIONS FOR ASSY
  TRIGGER(Z_TRIG=30,Z_LAUNCH=Z_BEAM,W_TRIG=22); // FOR PRINT

module FLEX_BEAM_FILLETED(BEAM,ORG,BEAM_T,BEAM_W,BEAM_ANG,R,NODES) {
    NB = len(BEAM);  // number of beams
    
    MAKE_BEAM_UNDEFORMED(BEAM,Z_BEAM);
    
    BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R); // Fillet at start
    
    // Fillet at end
    end_point = [NODES[NB][Nx],NODES[NB][Ny]+0.5,0]; 
    translate(end_point) 
        rotate([0,0,180]) BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R);
}

module BEAM_FILLETS(BEAM_T,BEAM_W,BEAM_ANG,R) {
    Y_FILR = R + BEAM_T/2;
    Y_ANG = R*sin(BEAM_ANG) +0.2; // NOT SURE WHY THE +0.2 IS NEEDED
    X_FILR = R;
    TRAPIZOID = [[X_FILR,Y_FILR+Y_ANG] , [-X_FILR/3,Y_FILR-Y_ANG] , [-X_FILR/3,-Y_FILR-Y_ANG] , [X_FILR,-Y_FILR+Y_ANG]];
    
    translate([0,0,-BEAM_W/2]) difference() {
        linear_extrude(BEAM_W,convexity=10)
            polygon(TRAPIZOID);
        translate([X_FILR,Y_FILR+Y_ANG,BEAM_W/2]) cylinder(h=2*BEAM_W,r=R,center=true,$fn=48);
        translate([X_FILR,-Y_FILR+Y_ANG,BEAM_W/2]) cylinder(h=2*BEAM_W,r=R,center=true,$fn=48);
    }
}
module SHOOTER_SUB1() {
    pts=[for (i=[0:NumberBeams]) [L*(i/NumberBeams),Y*sin(360*(i/NumberBeams)-90) + Y] ];
    //draw_points(pts,dia=2);
    
    // make beam model from points
    BEAM1 = beamFromNodes(nodes=pts,t=t,w=Z_BEAM,THICKEN_ENDS=true,
            T_MID=true,TUP=1.10,S=5,index=0,prior_ang=0); 
    echo(BEAM1=BEAM1);
        
    //draw_beam_undeformed(BEAM1);
    StartingNodes = getNodesFromBeams(BEAM1,0,0);  

    BEAM_ANG = 0;
    FLEX_BEAM_FILLETED(BEAM1,BEAM_T=t,BEAM_W=Z_BEAM,BEAM_ANG=BEAM_ANG,R=Rfillet, NODES=StartingNodes);

    translate([0,-BEAM_SPACE,0]) rotate([180,0,0]) 
        FLEX_BEAM_FILLETED(BEAM1,BEAM_T=t,BEAM_W=Z_BEAM,BEAM_ANG=BEAM_ANG,R=Rfillet, NODES=StartingNodes);
    
    translate([-END_T/2,-BEAM_SPACE/2,0]) cube([END_T,BEAM_SPACE*2,Z_BEAM],center=true);
    translate([L+END_T/2,-BEAM_SPACE/2,0]) cube([END_T,BEAM_SPACE*2,Z_BEAM],center=true);
}
*SHOOTER_SUB1();

module SHOOTER_SUB2() {
    MJX = 3.5;
    difference() {
        union() {
            
            translate([END_T,BEAM_SPACE/2,0]) 
                SHOOTER_SUB1();
            
            translate([V_LEN,0,-Z_BEAM/2]) 
                rotate([0,-90,90]) 
                    LAUNCHER(CAPX=Z_LAUNCH,CAPLEN=V_LEN,W=V_W);
        };
        // remove hole ON SLIDING END for tube
        rotate([0,90,0]) translate([Z_BEAM/MJX,0,0]) {
            BIG_D = 5.8;
            TOP_D = BIG_D*0.5;
            cylinder(h=3*V_LEN,d=BIG_D,center=true,$fn=100);
            
            // ADD HAT ON CYLINDER, TO MINIMIZE OVERHANG WHILE PRINTING
            translate([-Z_BEAM/MJX+BIG_D/1.6,0,0]) 
                rotate([0,0,45]) 
                    cube([TOP_D,TOP_D,3*V_LEN],center=true);
        }
        
        // remove hole on FIXED END (SMALLER)
        // DON'T CARE ABOUT SOME OVERHANGE HERE, AS WE WANT A TIGHT FIT
        rotate([0,90,0]) translate([Z_BEAM/MJX,0,L]) 
            cylinder(h=L,d=5.1,center=true,$fn=100);
        
        Z_TRANS = Z_LAUNCH-Z_VEE;
        
        translate([V_LEN-2,0,Z_TRANS]) rotate([0,-88,0]) 
            VEE_SLOT(Z_VEE,V_W-0.5,V_LEN);
        
        translate([L+22,0,0]) SCREW_HOLES(DIA=2.5);

    }

}
*SHOOTER_SUB2();

module LAUNCHER(CAPX=0.7,CAPLEN=4,W=0.5) { 
    union() {
        
        translate([0,0,-W/2])
            cube([CAPX,CAPLEN,W],center=false); // cube for airplane
        
        translate([0,0,-W_SQR/2])
            cube([W_SQR,CAPLEN,W_SQR],center=false); // cube for tube
        
        // LATCH addition
        translate([0,W,-0.75*W_SQR]) 
            cube([6,W/2,1.5*W_SQR],center=false);
    }
}
*LAUNCHER(CAPX=Z_LAUNCH,CAPLEN=V_LEN,W=V_W); 

module VEE_SLOT(X=1,Y=1,LEN=10) {
    // x is vee depth, y is vee width
    X_TIP = .7;
    translate([0,-Y/2,0])
    linear_extrude(LEN,convexity=10) {
        polygon([[0,Y/2-X_TIP],[X,0],[X,Y],[0,Y/2+X_TIP]]);
    }
};
*VEE_SLOT(Z_VEE,V_W,V_LEN);

module TRIGGER(Z_TRIG=10,Z_LAUNCH=5,W_TRIG=20) {
    FLEX_Z = 2;
    color("blue") {
        difference() {
            union() {
                translate([0,0,-Z_TRIG/2-4]) 
                    cube([V_W,W_TRIG,Z_TRIG],center=true); // trigger
                
                translate([0,-W_TRIG/2,-Z_LAUNCH/2-FLEX_Z]) 
                    cube([33,W_TRIG,FLEX_Z]); // flex beam
                
                translate([22,-W_TRIG/2,-Z_LAUNCH/2-FLEX_Z-2]) 
                    cube([10,W_TRIG,FLEX_Z]); // flex beam add at holes
                
                translate([30,-W_TRIG/2,-40-Z_LAUNCH/2]) 
                    cube([10,W_TRIG,40]); // palm upright

            }
            cube([10,W_SQR*1.15,Z_LAUNCH],center=true); // remove notch
            
            translate([-5,0,0]) 
                rotate([0,-30,0]) 
                    cube([10,W_TRIG*2,Z_LAUNCH],center=true); // remove chamfer
           
            translate([25,0,0]) SCREW_HOLES(DIA=3.3);  // remove screw holes
            
            translate([40,-W_TRIG/2,-30]) rotate([0,0,45])
                    cube([4,4,50],center=true); // remove palm upright chamfer
            translate([40,W_TRIG/2,-30]) rotate([0,0,45])
                    cube([4,4,50],center=true); // remove palm upright chamfer

        }
        translate([2,-W_TRIG/2,-13.5]) 
            rotate([0,-45,0]) 
                cube([6,W_TRIG,2.5]); // fillet on flex beam, trigger side
        
        translate([28,-W_TRIG/2,-11]) //-9.5]) 
            rotate([0,45,0]) 
                cube([6,W_TRIG,2]); // fillet on flex beam, palm side
    }
}

module SCREW_HOLES(DIA=10) {
    translate([0,5,0]) cylinder(h=30,d=DIA,center=true,$fn=50);
    translate([0,-5,0]) cylinder(h=30,d=DIA,center=true,$fn=50);
}