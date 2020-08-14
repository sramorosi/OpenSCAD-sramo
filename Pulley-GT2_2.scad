// Parametric Pulley for GT2-2 belt profiles
// by droftarts January 2012
// Modified to parametric GT2-2 module by SrAmo May-Aug 2020
// Based on pulleys by:
// http://www.thingiverse.com/thing:11256 by me!
// https://github.com/prusajr/PrusaMendel by Josef Prusa
// http://www.thingiverse.com/thing:3104 by GilesBathgate
// http://www.thingiverse.com/thing:2079 by nophead

// dxf tooth data from http://oem.cadregister.com/asp/PPOW_Entry.asp?company=915217&elementID=07807803/METRIC/URETH/WV0025/F
// pulley diameter checked and modelled from data at http://www.sdp-si.com/D265/HTML/D265T016.html

// number of teeth for display
display_teeth = 30;
// pulley
display_pulley = true;
// belt
display_belt = true;
// section cut
perform_section_cut = false;

// tuneable constants
//retainer = 1;		// Belt retainer above teeth, 0 = No, 1 = Yes
retainer_ht = 1.;	// height of retainer flange, part of pulley_t_ht
//idler = 1;			// Belt retainer below teeth, 0 = No, 1 = Yes
idler_ht = 1.;		// height of idler flange, part of pulley_t_ht

//	********************************
//	** Scaling tooth for good fit **
//	********************************
/*	To improve fit of belt to pulley, set the following constant. Decrease or increase by 0.1mm at a time. We are modelling the *BELT* tooth here, not the tooth on the pulley. Increasing the number will *decrease* the pulley tooth size. Increasing the tooth width will also scale proportionately the tooth depth, to maintain the shape of the tooth, and increase how far into the pulley the tooth is indented. Can be negative */

//difference () {
    
if (display_pulley) {
    difference () {
        pulley_gt2_2 ( teeth = display_teeth , pulley_t_ht = 10 ,motor_shaft=10 );

        if (perform_section_cut)
            translate ([0,0,55]) cube (100,center=true);
        }
    }

if (display_belt) {
    difference () {
        translate ([0,0,2]) 
            belt_circle(teeth=display_teeth,ht=6,thk=1.1);
        if (perform_section_cut)
            translate ([0,0,55.5]) cube (100,center=true);
        }
    }

// Functions

function tooth_spaceing_curvefit (teeth,b,c,d)
	= ((c * pow(teeth,d)) / (b + pow(teeth,d))) * teeth ;

function tooth_spacing(teeth,tooth_pitch,pitch_line_offset)
	= (2*((teeth*tooth_pitch)/(3.14159265*2)-pitch_line_offset)) ;

module belt_circle ( teeth=100, ht=5, thk = 1,tooth_depth=0.764 ,tooth_width=1.494){
    
    // for pulley 0.2, for belt -0.1 mm
    additional_tooth_width = -0.1; //mm
    
    //	If you need more tooth depth than this provides, adjust the following constant. However, this will cause the shape of the tooth to change.
    // for pulley 0, for belt 0 mm
    additional_tooth_depth = 0.0; //mm
        
    diam_mm = teeth*2/PI;
    echo(diam_mm=diam_mm);
    diam_inch = diam_mm/25.4;
    echo(diam_inch=diam_inch);
    
    pulley_OD = tooth_spacing (teeth,2,0.254);
    
	tooth_distance_from_centre = sqrt( pow(pulley_OD/2,2) - pow((tooth_width+additional_tooth_width)/2,2));
    
	tooth_width_scale = (tooth_width + additional_tooth_width ) / tooth_width;
	tooth_depth_scale = ((tooth_depth + additional_tooth_depth ) / tooth_depth) ;

    difference () {
        cylinder(h=ht,d=(diam_mm+2*thk),center=false,$fa=2);
        translate([0,0,-ht/4])
		linear_extrude(ht*2)
        difference()
			{
			//outside diameter of pulley
			translate([0,0,0]) 
			rotate ([0,0,360/(teeth*4)]) 
			//cylinder(r=pulley_OD/2,h=ht*2, $fa=2);
			circle(r=pulley_OD/2,$fa=2);
	
			//teeth - cut out of cylinder
			for(i=[1:teeth]) 
			rotate([0,0,i*(360/teeth)])
			translate([0,-tooth_distance_from_centre,-1]) 
			scale ([ tooth_width_scale , tooth_depth_scale , 1 ])  
                GT2_2mm(pulley_t_ht=ht*2);
			}

    }
}
// Main Module

module pulley_gt2_2( teeth = 40 , pulley_t_ht = 10, motor_shaft = 5.2, tooth_depth=0.764 , tooth_width=1.494 ,retainer=1,idler=1)
	{

    // for pulley 0.2, for belt -0.2 mm
    additional_tooth_width = 0.1; //mm
    
    //	If you need more tooth depth than this provides, adjust the following constant. However, this will cause the shape of the tooth to change.
    // for pulley 0, for belt 0 mm
    additional_tooth_depth = 0; //mm
        
    // calculated constants
    
    // The following set the pulley diameter for a given number of teeth
    pulley_OD = tooth_spacing (teeth,2,0.254);

	tooth_distance_from_centre = sqrt( pow(pulley_OD/2,2) - pow((tooth_width+additional_tooth_width)/2,2));
	tooth_width_scale = (tooth_width + additional_tooth_width ) / tooth_width;
	tooth_depth_scale = ((tooth_depth + additional_tooth_depth ) / tooth_depth) ;

	difference()
	 {	 
		union()
		{
			linear_extrude(pulley_t_ht)
            difference()
			{
			//shaft - diameter is outside diameter of pulley
			
			translate([0,0,0]) 
			rotate ([0,0,360/(teeth*4)]) 
            circle(r=pulley_OD/2);
			//cylinder(r=pulley_OD/2,h=pulley_t_ht, $fa=2);
	
			//teeth - cut out of shaft
		
			for(i=[1:teeth]) 
			rotate([0,0,i*(360/teeth)])
			translate([0,-tooth_distance_from_centre,-1]) 
			scale ([ tooth_width_scale , tooth_depth_scale , 1 ])  
                GT2_2mm(pulley_t_ht=pulley_t_ht);
			}
			
		//belt retainer / idler
		if ( retainer > 0 ) {translate ([0,0, pulley_t_ht-retainer_ht ]) 
		rotate_extrude($fa=2)  
		//polygon([[0,0],[pulley_OD/2,0],[pulley_OD/2 + retainer_ht , retainer_ht],[0 , retainer_ht],[0,0]]);}
		polygon([[0,-retainer_ht],[pulley_OD/2-retainer_ht,-retainer_ht],[pulley_OD/2 + retainer_ht , retainer_ht],[0 , retainer_ht],[0,-retainer_ht]]);}		
        
		if ( idler > 0 ) {translate ([0,0, 0 ]) 
		rotate_extrude($fa=2)  
		//polygon([[0,0],[pulley_OD/2 + idler_ht,0],[pulley_OD/2 , idler_ht],[0 , idler_ht],[0,0]]);}
        polygon([[0,0],[pulley_OD/2 + idler_ht,0],[pulley_OD/2-idler_ht , 2*idler_ht],[0 , 2*idler_ht],[0,0]]);}
		}
	   
		//hole for motor shaft
		translate([0,0,-1])
            cylinder(r=motor_shaft/2,h= pulley_t_ht + retainer_ht + 2,$fa=1,$fs=1);
				
	 }
	   
	}


// Tooth profile modules

module GT2_2mm(pulley_t_ht=6)
	{
	/*linear_extrude(height=pulley_t_ht+2) */ polygon([[0.747183,-0.5],[0.747183,0],[0.647876,0.037218],[0.598311,0.130528],[0.578556,0.238423],[0.547158,0.343077],[0.504649,0.443762],[0.451556,0.53975],[0.358229,0.636924],[0.2484,0.707276],[0.127259,0.750044],[0,0.76447],[-0.127259,0.750044],[-0.2484,0.707276],[-0.358229,0.636924],[-0.451556,0.53975],[-0.504797,0.443762],[-0.547291,0.343077],[-0.578605,0.238423],[-0.598311,0.130528],[-0.648009,0.037218],[-0.747183,0],[-0.747183,-0.5]]);
	}

