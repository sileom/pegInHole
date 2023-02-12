import open3d as o3d
import numpy as np
import copy



cad = o3d.io.read_point_cloud("resources/input/CAD.ply")
 #Ti frame non allineati
#Ti = np.array([[-0.98913791, -0.12128099, -0.0830489,   0.01986135],
# [ 0.11000404, -0.98552047,  0.12902911,  0.47794764],
# [-0.09749517,  0.11849187,  0.98815706, -0.70390326],
# [ 0.,          0.,          0.,          1.        ]])

#Ti frame allineati
Ti = np.array([[-0.04363803, -0.9989895, 0.01075619, 0.4076106],
            [-0.87943271, 0.03330271, -0.47485686, 0.11997196],
            [0.47401881, -0.03018116,-0.87999731, 0.80221387],
            [0, 0, 0, 1,   ]])

cad_temp = copy.deepcopy(cad)
cad_temp.transform(Ti)
o3d.io.write_point_cloud('resources/input/CR.ply', cad_temp, write_ascii=True)
o3d.visualization.draw_geometries([cad, cad_temp])


'''

def get_matrix(normal):
    R = np.array([[-0.094378, 0.995485, -0.009164],
                [0.995355, 0.094186, -0.019453],
                [-0.018502, -0.010957, -0.999769]])
    v_ = R.dot(normal)
    x = np.cross(normal,v_)
    x = x/np.linalg.norm(x)
    y = np.cross(normal, x)
    y = y/np.linalg.norm(y)
    return np.array([x, y, normal]).T 

def get_R_string(R):
    s = str(R[0,0]) + " " + str(R[0,1]) + " " + str(R[0,2]) + "\n" + str(R[1,0]) + " " + str(R[1,1]) + " " + str(R[1,2]) + "\n" + str(R[2,0]) + " " + str(R[2,1]) + " " + str(R[2,2]) + "\n"
    return s

# Transformation of holes and normals
hn = np.loadtxt("resources/input/FORI.txt", comments="#", delimiter=" ", usecols=range(7))

#Ti frame non allineati
#Ti = np.array([[-0.98913791, -0.12128099, -0.0830489,   0.01986135],
# [ 0.11000404, -0.98552047,  0.12902911,  0.47794764],
# [-0.09749517,  0.11849187,  0.98815706, -0.70390326],
# [ 0.,          0.,          0.,          1.        ]])

#Ti frame allineati
Ti = np.array([[-0.98642931, -0.13497221, -0.09348644,  0.05137532],
 [ 0.12082837, -0.9822826 ,  0.14325289,  0.4700342 ],
 [-0.11116526,  0.13001304,  0.98526083, -0.70722571],
 [ 0.,          0.,          0.,          1.        ]])

T_CR_scocca = np.array([[ 9.99999942e-01, -2.41146378e-04, -2.40039100e-04, -1.98705845e-04],
 [ 2.40973338e-04,  9.99999711e-01, -7.20648896e-04, -5.48682830e-04],
 [ 2.40212813e-04,  7.20591011e-04,  9.99999712e-01,  4.01899521e-06],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#Matrice di correzione tra ricostruzione e camera
Tc = np.array([[-1, 0, 0,  0],
            [0,  1,  0,  0],
            [0,  0, -1, 0],
            [0., 0.,  0., 1.]])

Are = np.array([[-0.0087,    0.9992,   -0.0400,    0.0504],
                    [0.9997,    0.0096,    0.0216,   -0.0109],
                    [0.0220,   -0.0398,   -0.9990,   -0.0616],
                    [   0 ,        0,         0 ,   1.0000]])


Ae = np.loadtxt("resources/input/p_r.txt", comments="#", delimiter=" ", usecols=range(4))
print(Ae)
Ace = np.loadtxt("resources/input/Ace.txt", comments="#", delimiter=" ", usecols=range(4))

file2 = open(r"resources/output/FORI.txt","wt")
file2.write("6 10\n")
for i in range(len(hn)):
    p = hn[i,:4]
    #foro = T_CR_scocca.dot(Ti.dot(p))
    foro = Ae.dot(Are.dot(T_CR_scocca.dot(Ti.dot(p))))
    #foro = Ae.dot(Ace.dot(Tc.dot(T_CR_scocca.dot(Ti.dot(p)))))
    s_ = str(foro[0]) + " " + str(foro[1]) + " " + str(foro[2]) + "\n"
    file2.write(s_)
    file2.write("Rd\n")

    n = hn[i,4:]
    normale = Ae[:3,:3].dot(Ace[:3,:3].dot(( Ti[:3,:3].dot(n) )))
    matrix = get_matrix(normale)
    s_ = get_R_string(matrix)
    file2.write(s_)


file2.close()
'''