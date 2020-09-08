import math
import turtle

import numpy as np

pen = turtle.Pen()
pen.pencolor('blue')
pen.hideturtle()
pen.speed('fastest')

# Modulus of Elasticity (PSI)
E = 340000
# Tensile Ultimate Strength (PSI)
Ftu = 7000
# material density (lb per cubic inch)
rho = 0.05

# MADE-UP FACTOR TO DETERMINE IF FORCE OR MOMENT
Q = 1
# CANTELIVER BEAM WITH FORCE, 6 SEGMENT
# beam thickness
t = 0.1
# beam width
w = .4
ang = 0
dna = [[.5, t, w, ang], [.5, t, w, ang], [.5, t, w, ang], [.5, t, w, ang], [.5, t, w, ang], [.5, t, w, ang]]
F_ext = [[0, 0], [0, 0], [0, 0], [0, 0], [3, 0], [3, 4]]
M_ext = [0, 0, 0, 0, 0, 0]

# CHECK number of beam segments
n = len(dna)
# print(dna,n)

# n_sub = [ for (i=[0:1:n-1]) len(dna[i]) ]
# if ( sumv(n_sub,n-1) != n*4 ) { echo (n_sub=n_sub," NOT EQUAL ",n=n);}
n_F = len(F_ext)
if n_F != n:
    print('F_ext length', n_F, ' NOT EQUAL ', n)
n_M = len(M_ext)
if n_M != n:
    print('M_ext length', n_M, ' NOT EQUAL ', n)


# recursive module that draws the undeformed beam.
def draw_beam_undeformed(dna_vector, idx=0):
    if idx < len(dna_vector):
        L = dna_vector[idx][0]
        t = dna_vector[idx][1]
        # w = dna_vector[idx][2]
        z_ang = dna_vector[idx][3]
        # print(idx,L,t,w,z_ang)

        # draw the beam segment
        pen.pensize(t * 100)
        pen.left(z_ang)  # rotation
        pen.forward(L * 100)  # translation

        # Recursive call generating the next beam
        # turtle remembers where it is
        draw_beam_undeformed(dna_vector, idx + 1)


# recursive module that calculates the undeformed beam nodes from the DNA
def nodes_undeformed(nodes, idx=0):
    if idx < n:
        L = dna[idx][0]
        z_ang = dna[idx][3]
        x = L * math.cos(math.radians(z_ang))
        y = L * math.sin(math.radians(z_ang))

        nodes.append([nodes[idx][0] + x, nodes[idx][1] + y])
        # Recursive call generating the next beam
        nodes_undeformed(nodes, idx + 1)


# segment area moment of inertia about Z axis
def Iz_func(w, t):    return ((w * t * t * t) / 12)


# compute beam deflections
def gen_beam_displacement(nodes):
    return nodes * 10


# recursion - find the sum of the values in a vector (array) by calling itself
# from the start (or s'th element) to the i'th element - remember elements are zero based
def sumv(v, i, s=0):
    if i == s:
        return v[i]
    else:
        return np.add(v[i], sumv(v, i - 1, s))


def rot_x(x, y, a):  return x * math.cos(math.radians(a)) - y * math.sin(math.radians(a))


def rot_y(x, y, a):  return x * math.sin(math.radians(a)) + y * math.cos(math.radians(a))


# recursive module that calculates the deformed beam nodes from the DNA
def nodes_deformed(n_in, n_out, idx=0):
    if idx < n:
        L = dna[idx][0]
        z_ang = dna[idx][3]
        tha = 10
        g = 1
        x = n_in[idx][0] + rot_x(L * (1 - g * (1 - math.cos(tha))), g * L * math.sin(tha), z_ang)
        y = n_in[idx][1] + rot_y(L * (1 - g * (1 - math.cos(tha))), g * L * math.sin(tha), z_ang)

        n_out.append([x, y])
        # Recursive call generating the next beam
        nodes_deformed(n_in, n_out, idx + 1)


# draw_beam_undeformed(dna)  # draw the beam

# generate moment of inertia
Iz = []
for i in range(n):    Iz.append(Iz_func(dna[i][2], dna[i][1]))

# generate cross section area
Area = []
for i in range(n):    Area.append(dna[i][1] * dna[i][2])

# generate undeformed nodes from DNA list
nodes_un = [[0, 0]]  # first node is zero,zero (for now)
nodes_undeformed(nodes_un)
# print(nodes_un)

# generate internal forces from external forces
F_int = []
for i in range(n):  F_int.append(sumv(F_ext, n - 1, i))
print(F_ext)
print(F_int)

# generate deformed nodes
nodes_def = [[0, 0]]
M_def = []
nodes_deformed(nodes_un, nodes_def)
print(nodes_def)

# turtle.done()
