// Large Displacement Beam, Index Constants
//
// LIST DEFINITIONS
// LDBdef (LDB definition)

// Indexes into the Beam array start with Z
Ztype = 0;     // first element of each list 
    // All Q values should numbers that don't get used for angles or what not
    Qbeam = 11111;  
    Qstart = 88888;  // invisible beam  at the start
    Qend = 99999;    // invisible beam at the end

// BEAM ARRAY: [Qbeam,Blen,Bthk,Bw,Bang]  INITIALLY DEFINED, NOT CHANGED
Blen = 1;   // beam length
Bthk = 2;   // beam thickness
Bw = 3;     // beam width (normal to plane, or printer bed)
Bang = 4;   // Unloaded Z rotation of beam relative to prior beam (local)
  // Bang is needed for AUTO-GENERATED BEAMS (i.e. from points)

// INITIAL Load AND ANGLES Array: [Ifx,Ify,Im]
// Array size = Beams + 1  (Nodes)
Ifx = 0;    // external force in global X
Ify = 1;    // external force in global Y
Im = 2;     // external moment about global Z 

// USED WITH NODES ARRAY [Nx,Ny,Nang]
Nx = 0;
Ny = 1;
Nang = 2;

// USED WITH Result Array:
Ztheta = 0; // Spring angle, at the start of the beam
Za = 1;     // beam-local x end position
Zb = 2;     // beam-local y end position
Zms = 3;       // margin of safety
Zstressmin = 4;
Zstressmax = 5;
Zenergy = 6;
ZNewThk = 7;
ZReactx = 8;
ZReacty = 9;
ZReactm = 10;
