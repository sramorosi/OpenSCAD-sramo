include <LDB_Indexes.scad>
Results = [[23.969, 36.3466, 2.18552, 0.696326, 0.7346, 1.65944], [15.9793, 24.2311, 2.2671, 0.471866, 0.7346, 2.52309], [4.5045, 5.58558, 2.32721, 0.155766, 0.85, 4.24152]];
origin = [0,0];


// Define an array of points (x, y, z)
points = [
    [0, 0, 0],
    [1, 1, 1],
    [2, 2, 2],
    [3, 3, 3]
];

// Function to process points recursively (NOTE: last 4 parameters are hidden-don't supply)
// Modified example from ChatGPT
function processPoints(pointsArray, index=0,x_past=0,y_past=0,ang_past=0) = 
    index < len(pointsArray) ?
        let (x = pointsArray[index][Za])
        let (y = pointsArray[index][Zb])
        let (ang = (index==0 ? 0 : pointsArray[index-1][Zthetaend]))
        let (sum_ang = ang + ang_past) // add up to get new angle
        let (x_new = rot_x(x,y,sum_ang) + x_past) // add up to get new x
        let (y_new = rot_y(x,y,sum_ang) + y_past) // add up to get new y
        //echo(index=index,x=x,y=y,y_past=y_past,sum_ang=sum_ang)
        concat([ [x_new , y_new ] ],
        // Recursive call to process the next point
        processPoints(pointsArray, index + 1, x_new , y_new , sum_ang) ) 
    :  [] ;  // Return nothing when all points are processed
     
function rot_x (x,y,a) = x*cos(a)-y*sin(a);

function rot_y (x,y,a) = x*sin(a)+y*cos(a);
test=rot_y(2.32721,0.155766,60.5777);
echo(test=test);
// Call the function with initial values
NewPoints = concat([origin],processPoints(Results));
echo(NewPoints=NewPoints);
draw_points(NewPoints);

module draw_points(pts,dia=0.1) {
    numPts = len(pts);
    for (i=[0:numPts-1]) translate([pts[i][0],pts[i][1],1]) color("black") circle(dia,$fn=8);
}