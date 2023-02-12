import open3d as o3d 


def saveTxt(o3d_pc, filepath, increment):
    points = o3d_pc.points
    file = open(filepath,"wt")
    for i in range(0, len(points), increment):
        p_ = points[i]
        s_ = str(p_[0]) + "," + str(p_[1]) + "," + str(p_[2]) + "\n"
        file.write(s_)    
    file.close()
    print("Saved")
