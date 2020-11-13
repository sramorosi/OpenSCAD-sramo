// Large Displacement Beam, Index Constants

// Indexes into the DNA array start with Z
Ztype = 0;     // first element of each list 
    // All Q values should numbers that don't get used for angles or what not
    Qnode = 0;  // NOT IMPLEMENTED. Way to specify origin.
    Qbeam = 11111;  
    Qresult = 10000;  // beam analysis results
    Qfork = 22222;
    Qload = 33333;  
    Qdisp = 4;  // NOT IMPLEMENTED.
    Qskip = 99999;  
// USED WITH Qbeam:
Zlen = 1;   // beam length
Zthk = 2;   // beam thickness
Zw = 3;     // beam thickness
Zang = 4;   // Z rotation of beam relative to prior beam
// USED WITH Qload:
Zfx = 1;    // external force in global X
Zfy = 2;    // external force in global Y
Zm = 3;     // external moment about global Z 
// USED WITH dx dy
Zdx = 1;
Zdy = 2;
// USED WITH Qresult:
Zbeamtype = 1;
Zrad = 2;  // characteristic radius
//Zconst = 2; // constant used in determining spring rate
ZK = 3;     // Spring Rate
Ztheta = 4; // Spring angle
//Zend = 5;   // constant used to determine end angle from spring angle
Zthetaend = 5;
Za = 6;     // beam x end position
Zb = 7;     // beam y end position
Zstressmax = 8;
Zstressmin = 9;
Zenergy = 10;
Zweight = 11;   // beam segment weight
Zms = 12;       // margin of safety
// USED WITH BEAM TYPE
Zvertforce = 12; // use A.1.2 vertical force PRBM constants
Zendmoment = 15; // use A.1.5 end moment PRBM constants
