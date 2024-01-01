// Tetrahedral honeycomb module
module tetrahedron(x, y, z, r) {
  // r is the circumradius of the tetrahedron
  // x, y, z are the coordinates of the center of the tetrahedron
  // The vertices of the tetrahedron are calculated using trigonometry
  v1 = [x, y + r * sqrt(3) / 3, z - r / sqrt(6)];
  v2 = [x + r * sqrt(3) / 3, y - r * sqrt(3) / 6, z - r / sqrt(6)];
  v3 = [x - r * sqrt(3) / 3, y - r * sqrt(3) / 6, z - r / sqrt(6)];
  v4 = [x, y, z + r * sqrt(2) / sqrt(3)];
  // The faces of the tetrahedron are defined by the indices of the vertices
  f1 = [0, 1, 2];
  f2 = [0, 1, 3];
  f3 = [0, 2, 3];
  f4 = [1, 2, 3];
  // The polyhedron function creates the 3D shape from the vertices and faces
  polyhedron(points = [v1, v2, v3, v4], faces = [f1, f2, f3, f4]);
}

// Parameters for the honeycomb
r = 10; // Circumradius of the tetrahedron
n = 2; // Number of tetrahedra along each axis
s = r;// * sqrt(2); // Distance between the centers of adjacent tetrahedra

// Loop to create the honeycomb
for (i = [0 : n - 1]) {
  for (j = [0 : n - 1]) {
    for (k = [0 : n - 1]) {
      // The offset is used to alternate the positions of the tetrahedra
      offset = 0;//(i + j + k) % 2 * r / sqrt(6);
      // The center of each tetrahedron is calculated using the index and the distance
      x = i * s;
      y = j * s;
      z = k * s + offset;
      // The tetrahedron module is called with the center and the radius
      tetrahedron(x, y, z, r);
    }
  }
}
// Alternated cubic honeycomb module
module honeycomb(x, y, z, r) {
  // r is the edge length of the truncated octahedron
  // x, y, z are the coordinates of the center of the truncated octahedron
  // The vertices of the truncated octahedron are calculated using trigonometry
  v1 = [x + r / sqrt(2), y, z];
  v2 = [x - r / sqrt(2), y, z];
  v3 = [x, y + r / sqrt(2), z];
  v4 = [x, y - r / sqrt(2), z];
  v5 = [x, y, z + r / sqrt(2)];
  v6 = [x, y, z - r / sqrt(2)];
  v7 = [x + r / 2, y + r / 2, z + r / 2];
  v8 = [x + r / 2, y + r / 2, z - r / 2];
  v9 = [x + r / 2, y - r / 2, z + r / 2];
  v10 = [x + r / 2, y - r / 2, z - r / 2];
  v11 = [x - r / 2, y + r / 2, z + r / 2];
  v12 = [x - r / 2, y + r / 2, z - r / 2];
  v13 = [x - r / 2, y - r / 2, z + r / 2];
  v14 = [x - r / 2, y - r / 2, z - r / 2];
  // The faces of the truncated octahedron are defined by the indices of the vertices
  f1 = [0, 6, 8, 2];
  f2 = [0, 2, 10, 4];
  f3 = [0, 4, 12, 6];
  f4 = [1, 7, 9, 3];
  f5 = [1, 3, 11, 5];
  f6 = [1, 5, 13, 7];
  f7 = [2, 8, 9, 3];
  f8 = [3, 9, 7, 11];
  f9 = [4, 10, 11, 5];
  f10 = [5, 11, 7, 13];
  f11 = [6, 12, 13, 7];
  f12 = [7, 13, 12, 6];
  f13 = [8, 6, 12, 10];
  f14 = [9, 8, 10, 11];
  // The polyhedron function creates the 3D shape from the vertices and faces
  polyhedron(points = [v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14], faces = [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14],convexity=20);

  // The offset is used to alternate the positions of the tetrahedra
  offset = r / sqrt(6);
  // The centers of the tetrahedra are calculated using the center and the edge length of the truncated octahedron
  x1 = x + r / sqrt(2) + r / 2;
  x2 = x - r / sqrt(2) - r / 2;
  y1 = y + r / sqrt(2) + r / 2;
  y2 = y - r / sqrt(2) - r / 2;
  z1 = z + r / sqrt(2) + r / 2;
  z2 = z - r / sqrt(2) - r / 2;
  // The tetrahedron module is called with the centers and the edge length
  /*tetrahedron(x1, y1, z1 + offset, r);
  tetrahedron(x1, y1, z2 - offset, r);
  tetrahedron(x1, y2, z1 - offset, r);
  tetrahedron(x1, y2, z2 + offset, r);
  tetrahedron(x2, y1, z1 - offset, r);
  tetrahedron(x2, y1, z2 + offset, r);
  tetrahedron(x2, y2, z1 + offset, r);
  tetrahedron(x2, y2, z2 - offset, r); */
}

// Parameters for the honeycomb
honeycomb(40, 0, 0, 10);

/* Loop to create the honeycomb
for (i = [0 : n - 1]) {
  for (j = [0 : n - 1]) {
    for (k = [0 : n - 1]) {
      // The center of each truncated octahedron is calculated using the index and the distance
      x = i * s;
      y = j * s;
      z = k * s;
      // The honeycomb module is called with the center and the edge length
      honeycomb(x, y, z, r);
    }
  }
} */
