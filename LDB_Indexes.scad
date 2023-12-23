// Large Displacement Beam, Index Constants
//
// LIST DEFINITIONS
// LDBdef (LDB definition)
// [[beam, len, thk, width, angle]

// Indexes into the DNA array start with Z
Ztype = 0;     // first element of each list 
    // All Q values should numbers that don't get used for angles or what not
    Qnode = 0;  // NOT IMPLEMENTED. Way to specify origin.
    Qbeam = 11111;  
    Qresult = 10000;  // beam analysis results
    Qfork = 22222;
    Qload = 33333;  
    QdispX = 44441;  // X displacement contraint on a node
    QdispY = 44442;  // Y displacement contraint on a node
    Qskip = 99999;  

// USED WITH Qbeam: [Qbeam,Zlen,Zthk,Zw,Zang,ZangLoad]
Zlen = 1;   // beam length
Zthk = 2;   // beam thickness
Zw = 3;     // beam thickness
Zang = 4;   // Unloaded Z rotation of beam relative to prior beam (local)
ZangLoad = 5; // Estimated deflection angle with load (helps with convergence)

// USED WITH Qload: [Qload,Zfx,Zfy,Zm]
Zfx = 1;    // external force in global X
Zfy = 2;    // external force in global Y
Zm = 3;     // external moment about global Z 
//Ztargetx = 4;  // to specify a target x value with load
//Ztargety = 5;  // to specify a target y value with load

// USED WITH QDisp:
// Zfx = 1;    // Estimated external force in global X to meet displacement
// Zfy = 2;    // Estimated external force in global Y to meet displacement
Ztarget = 3; // Target value for the displacement (X or Y)
ZdFdx = 4; // Estimate spring rate
Zgreater = 5;
Zless = 6;

// USED WITH dx dy ??
Zdx = 1;
Zdy = 2;

// USED WITH Qresult:
// [Qresult,bt,cr,K,theta,theta_end,a,b,stressmax,stressmin,energy,weight,ms,fx,fy,m_total] : 
Zbeamtype = 1;  // see below
Zrad = 2;  // characteristic radius
ZK = 3;     // Spring Rate
Ztheta = 4; // Spring angle
Zthetaend = 5; // angle of the end of the beam
Za = 6;     // beam x end position
Zb = 7;     // beam y end position
Zstressmax = 8;
Zstressmin = 9;
Zenergy = 10;
Zweight = 11;   // beam segment weight
Zms = 12;       // margin of safety
ZPx = 13;      // X load on beam end, local
ZPy = 14;      // Y load on beam end, local
ZPm = 15;      // Total Moment on beam end, local
ZMerror = 16;  // Difference between ZPm[i] and Zm[i+1] (from Qload)


// USED WITH BEAM TYPE
Zvertforce = 12; // use A.1.2 vertical force PRBM constants
Zendmoment = 15; // use A.1.5 end moment PRBM constants
