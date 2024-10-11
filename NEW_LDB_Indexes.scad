// Large Displacement Beam, Index Constants
//
// LIST DEFINITIONS
// LDBdef (LDB definition)

// Indexes into the DNA array start with Z
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
//Ztargetx = 4;  // to specify a target x value with load
//Ztargety = 5;  // to specify a target y value with load

// USED WITH QDisp:
// Zfx = 1;    // Estimated external force in global X to meet displacement
// Zfy = 2;    // Estimated external force in global Y to meet displacement
//Ztarget = 3; // Target value for the displacement (X or Y)
//ZdFdx = 4; // Estimate spring rate
//Zgreater = 5;
//Zless = 6;

// USED WITH NODES ARRAY [Nx,Ny,Nang]
Nx = 0;
Ny = 1;
Nang = 2;

// USED WITH Result Array:
// Matches length of beam array
// [theta,theta_end,a,b,K,CR,ms,   stressmax,stressmin,energy,weight,ms,fx,fy,m_total] : 
Ztheta = 0; // Spring angle, at the start of the beam
Za = 1;     // beam-local x end position
Zb = 2;     // beam-local y end position
//ZK = 3;     // Spring Rate
//Zrad = 4;  // characteristic radius
Zms = 3;       // margin of safety
Zstressmin = 4;
Zstressmax = 5;
Zenergy = 6;
ZNewThk = 7;
ZReactx = 8;
ZReacty = 9;
ZReactm = 10;

//ZPx = 13;      // X load on beam end, local
//ZPy = 14;      // Y load on beam end, local
//ZPm = 15;      // Total Moment on beam end, local
//ZMerror = 16;  // Difference between ZPm[i] and Zm[i+1] (from Qload)