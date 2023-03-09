
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

def main():
    pcl = InterbotixPointCloudInterface()
    average = input("How many pics: ")
    while(1==1):
        check = input("Take pic (y/n): ")
        if check == "n":
            break
        elif check == "y":
            sum_x = 0
            sum_y = 0
            sum_z = 0
            for i in range(average):
                success, clusters = pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="y", reverse=False)
                
                for cluster in clusters:
                    x,y,z = cluster["position"]
                    sum_x += x
                    sum_y += y
                    sum_z += z
                    break
            average_x = sum_x/average
            average_y = sum_y/average
            average_z = sum_z/average
            
            print("X = ", average_x)
            print("Y = ", average_y)
            print("Z = ", average_z)
                
if __name__=='__main__':
    main()