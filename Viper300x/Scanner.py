from ErrorAnalysis import *

def Scanner():
    
    success, clusters = pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
    pass_on = True
    x,y,z = cluster[0]["position"]
    X,Y = Error(x,y)
    
    if z < 0.04:
        pass_on = True
        x += X
        y += Y
    else:
        pass_on = False 
        
    if success == False:
        pass_on = False
    
    return pass_on, x, y