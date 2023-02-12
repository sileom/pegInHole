import open3d as o3d
import numpy as np
import os
import sys
import glob
import time

def load_point_clouds(voxel_size=0.0):
    pcds_list = glob.glob("resources/ply_data/pointCloud_*.ply")
    print(pcds_list)
    pcds = []
    for i in range(len(pcds_list)):
        pcd = o3d.io.read_point_cloud("resources/ply_data/pointCloud_%d.ply" % i)
        #o3d.visualization.draw_geometries([pcd])
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcds.append(pcd_down)
    return pcds

def load_point_clouds2(voxel_size=0.0):
    pcds_list = glob.glob("resources/ply_data/pointCloud_*.ply")
    print(pcds_list)
    pcds = []
    for i in range(len(pcds_list)-2, -1, -1):
        pcd = o3d.io.read_point_cloud("resources/ply_data/pointCloud_%d.ply" % i)
        print("resources/ply_data/pointCloud_%d.ply" % i)
        #o3d.visualization.draw_geometries([pcd])
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        pcds.append(pcd_down)
    return pcds
    
voxel_size = 0.01
t = time.time()
pcds_down = load_point_clouds2(voxel_size)
#frame_s = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.array([0., 0., 0.]))
#o3d.visualization.draw_geometries(pcds_down)

def pairwise_registration(source, target):
    #print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp
    
def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        #print(source_id)
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            #print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph
    
#print("Full registration ...")
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5

pose_graph = full_registration(pcds_down,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)
                                   
#print("Optimizing PoseGraph ...")
option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=max_correspondence_distance_fine,
    edge_prune_threshold=0.25,
    reference_node=0)

o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
        
#print("Transform points and display")
#for point_id in range(len(pcds_down)):
#    print(pose_graph.nodes[point_id].pose)
#    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
#o3d.visualization.draw_geometries(pcds_down)

#pcds = load_point_clouds(voxel_size)
pcd_combined = o3d.geometry.PointCloud()
for point_id in range(len(pcds_down)):
    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    pcd_combined += pcds_down[point_id]
pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
o3d.io.write_point_cloud("resources/output/vasca_2.pcd", pcd_combined_down)

elapsed = time.time() - t
print("MON - tempo per caricamento + ricerca delle trasformazioni + generazione della vasca totale + salvataggio")
print(elapsed)

argvs=sys.argv
savePath = argvs[1]
file_name= savePath + "vasca_2.pcd"
o3d.io.write_point_cloud(file_name, pcd_combined_down)
#o3d.io.write_point_cloud("resources/output/vasca.ply", pcd_combined_down)
from utilsPython import saveTxt
saveTxt(pcd_combined_down, "resources/output/vasca_2.txt", 1)
file_name = savePath + "vasca_2.txt"
saveTxt(pcd_combined_down, file_name, 1)
frame_s = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.array([0., 0., 0.]))
#o3d.visualization.draw_geometries([pcd_combined_down, frame_s])

print("vasca_2.pcd generated")


