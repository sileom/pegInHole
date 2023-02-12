import open3d as o3d
import numpy as np
import copy
import sys
import time

attemps = 10

def filter_pointcloud_on_z(source, voxel_size):
    z_min = 0.02
    z_max = 0.80
    result = []
    for i in range(len(source.points)):
        _p = source.points[i]
        if (z_min <= abs(_p[2]) and abs(_p[2]) <= z_max):
            result.append(_p)
    
    result_pc = o3d.geometry.PointCloud()
    result_pc.points = o3d.utility.Vector3dVector(result)
    result_pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    #o3d.io.write_point_cloud("resources/output/vasca_clear.pcd", result_pc)
    #o3d.visualization.draw_geometries([result_pc])
    return result_pc


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    #o3d.io.write_point_cloud('resources/output/aligned_vasca.ply', target_temp, write_ascii=True)
    from utilsPython import saveTxt
    saveTxt(source_temp, "resources/output/aligned_cad_7.txt", 7)
    argvs=sys.argv
    savePath = argvs[1]
    file_name = savePath + "aligned_cad_7.txt"
    saveTxt(source_temp, file_name, 7)
    frame_s = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=np.array([0., 0., 0.]))
    o3d.visualization.draw_geometries([source_temp, target_temp, frame_s])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size):
    #print(":: Load two point clouds and disturb initial pose.")
    #source = o3d.io.read_point_cloud("resources/output/vasca.pcd")
    #source_ = o3d.io.read_point_cloud("resources/output/vasca.pcd")
    target = o3d.io.read_point_cloud("resources/output/vasca.pcd")
    source =  o3d.io.read_point_cloud("resources/input/CR.ply")
    #source_ =  o3d.io.read_point_cloud("resources/input/CAD.ply")
    #target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    #target = o3d.io.read_point_cloud("resources/input/cad.ply")

    #Pulizia pointcloud vasca
    #source = filter_pointcloud_on_z(source_, voxel_size)
    #frame_s = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.array([0., 0., 0.]))
    #o3d.visualization.draw_geometries([source, frame_s])
    #target = filter_pointcloud_on_z(target_, voxel_size)
    #o3d.visualization.draw_geometries([source, target, frame_s])

    #o3d.io.write_point_cloud("resources/output/vasca_ascii.ply", source, write_ascii=True)

    #trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                         [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    #source.transform(trans_init)
    #draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

voxel_size = 0.05  # means 5cm for this dataset

t = time.time()
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    voxel_size)
    
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    #print(":: RANSAC registration on downsampled point clouds.")
    #print("   Since the downsampling voxel size is %.3f," % voxel_size)
    #print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result
    
result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
#print(result_ransac)
#print(result_ransac.transformation)
#draw_registration_result(source_down, target_down, result_ransac.transformation)

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    #print(":: Point-to-plane ICP registration is applied on original point")
    #print("   clouds to refine the alignment. This time we use a strict")
    #print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result
    
result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size)
#print(result_icp)
#print(result_icp.transformation)

#draw_registration_result(source, target, result_icp.transformation)


# Controllo sulla bonta' e reiterazione del metodo
times = 1
while(result_icp.fitness <= 0.2 and times < attemps):
    result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
    #print(result_ransac)
    result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size)
    #print(result_icp)
    times = times + 1

elapsed = time.time() - t
print("MON - tempo per caricamento + ricerca della sovrapposizione + eventuale reiterazione")
print(elapsed)

draw_registration_result(source, target, result_icp.transformation)
print("Number of attempts: ", times)

argvs=sys.argv
savePath = argvs[1]
file_name = savePath + "Registration_results.txt"
file1 = open(file_name,"w")
file1.write("Fitness - Inlier_rmse - Correspondence_set\n")
linea = str(result_icp.fitness) + " " + str(result_icp.inlier_rmse) + " " + str(result_icp.correspondence_set)
file1.write(linea)
file1.write("\nMatrix\n")
linea = str(result_icp.transformation[0,0]) + " " + str(result_icp.transformation[0,1]) + " " + str(result_icp.transformation[0,2]) + " " + str(result_icp.transformation[0,3]) + "\n" \
    + str(result_icp.transformation[1,0]) + " " + str(result_icp.transformation[1,1]) + " " + str(result_icp.transformation[1,2]) + " " + str(result_icp.transformation[1,3]) + "\n" \
    + str(result_icp.transformation[2,0]) + " " + str(result_icp.transformation[2,1]) + " " + str(result_icp.transformation[2,2]) + " " + str(result_icp.transformation[2,3]) + "\n" \
    + "0 0 0 1\n"
file1.write(linea)
file1.close()


# Transformation of holes and normals
def get_matrix(normal, y_w):
    R = np.array([[-0.094378, 0.995485, -0.009164],
                [0.995355, 0.094186, -0.019453],
                [-0.018502, -0.010957, -0.999769]])
    v_ = R.dot(normal)
    x = np.cross(normal,y_w)
    #x = np.cross(np.array([1, 0, 0]), normal)
    x = x/np.linalg.norm(x)
    y = np.cross(normal, x)
    y = y/np.linalg.norm(y)
    return np.array([x, y, normal]).T 

def get_R_string(R):
    s = str(R[0,0]) + " " + str(R[0,1]) + " " + str(R[0,2]) + "\n" + str(R[1,0]) + " " + str(R[1,1]) + " " + str(R[1,2]) + "\n" + str(R[2,0]) + " " + str(R[2,1]) + " " + str(R[2,2]) + "\n"
    return s

def get_holes_and_normals_W(T_CR_scocca):
    # carica i fori e le normali in terna CAD (originale)
    hn = np.loadtxt("resources/input/fori_cad.txt", comments="#", delimiter=" ", usecols=range(7))
    # carico la matrice CAD - CAD_rif
    Ti = np.loadtxt("resources/input/Ti.txt", comments="#", delimiter=" ", usecols=range(4))
    # carico la matrice riferimento - EE
    Ace = np.loadtxt("resources/input/Ace.txt", comments="#", delimiter=" ", usecols=range(4))
    Are = np.loadtxt("resources/input/Are.txt", comments="#", delimiter=" ", usecols=range(4))
    # carico la matrice EE - W (che rappresenta la posizione di riferimento per la ricostruzione)
    Ae = np.loadtxt("resources/input/p_r.txt", comments="#", delimiter=" ", usecols=range(4))
    
    Tc = np.array([[-1, 0, 0,  0],
            [0,  1,  0,  0],
            [0,  0, -1, 0],
            [0., 0.,  0., 1.]])

    fileh = open(r"resources/output/fori_w.txt","wt")
    #fileh.write("6 10\n")
    s_2 = str(len(hn)) + " 10\n"
    fileh.write(s_2)
    for i in range(len(hn)):
        p = hn[i,:4]
        #foro = T_CR_scocca.dot(Ti.dot(p))
        foro = Ae.dot( Ace.dot(T_CR_scocca.dot(Ti.dot(p))) )
        #foro = Ae.dot(Ace.dot(Tc.dot(T_CR_scocca.dot(Ti.dot(p)))))
        s_ = str(foro[0]) + " " + str(foro[1]) + " " + str(foro[2]) + "\n"
        fileh.write(s_)
        fileh.write("Rd\n")

        n = hn[i,4:]
        normale = Ae[:3,:3].dot(Ace[:3,:3].dot( T_CR_scocca[:3,:3].dot((Ti[:3,:3].dot(n))) ) )
        y_w = Ae[:3,:3].dot(Ace[:3,:3].dot( T_CR_scocca[:3,:3].dot((Ti[:3,:3].dot([0, 1, 0]))) ) )
        #normale = Ae[:3,:3].dot(Ace[:3,:3].dot( Tc[:3,:3].dot(T_CR_scocca[:3,:3].dot((Ti[:3,:3].dot(n)))) ) )
        #normale = T_CR_scocca[:3,:3].dot((Ti[:3,:3].dot(n)))
        matrix = get_matrix(normale, y_w)
        #if(i < 5):
        #    matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1] ])
        s_ = get_R_string(matrix)
        fileh.write(s_)
    fileh.close()

    import shutil
    import os
    original = os.path.abspath("resources/output/fori_w.txt")
    target = savePath + "fori_world.txt"
    shutil.copyfile(original, target)

np.savetxt("resources/input/T_cr_s.txt", result_icp.transformation, fmt="%s", delimiter= " ")
get_holes_and_normals_W(result_icp.transformation)
print("DONE")




