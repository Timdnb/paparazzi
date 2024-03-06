import numpy as np
import pandas as pd


def Find_T_drone_org(roll, pitch, yaw, x, y, z):

    # Convert Euler angles to rotation matrices
    R_roll = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine individual rotation matrices to obtain the overall rotation matrix
    rotation_matrix = np.dot(np.dot(R_yaw, R_pitch), R_roll)


    # Translation vector representing displacement between the origins of the frames
    translation_vector = np.array([-x, -y, -z])  # Replace with actual translation values

    # Homogeneous transformation matrix
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = translation_vector
    
    T_drone_org = homogeneous_matrix
    
    return T_drone_org




def corners_object_to_drone(corners_object, T_drone_org):
    corners_drone = None   # calculate bounding box corners in drone frame
    
    corners_drone = np.empty((4,4))\
    
    for index, object in enumerate(corners_object):
        corners_drone[index] = np.dot(T_drone_org, object.T)
        
    assert corners_drone.shape == (4, 4)
    
    return corners_drone


def project_points(projection_matrix, corners_drone):
    """
    projection_matrix: The projection matrix based on the intrinsic camera calibration.
    points: Nx[x,y,z,1.0] np.array of the points that need to be projected to the camera image from drone frame.
    return: Nx[u,v] rounded coordinates of the points in the camera image as int data type.
    """
    assert corners_drone.shape[-1] == 4
    # points = points[points[:,2] > 0.0]
    uvs = None   # calculate points in image plane
    
    coordinates = np.dot(projection_matrix, corners_drone.T).T
    coordinates = coordinates[:, :2] / coordinates[:, 2:] #normalisation
    uvs = coordinates[:, :2]
    uvs = np.round(uvs).astype(int)
    
    
    return uvs



#-------MAIN--------
# projection matrix?
# corners coordinates?
# transaltion in a matrix?

#corner coordiantes in original frame
cor_coord_org = np.array([[1,1,1,1],
                          [2,3,4,1],
                          [6,7,8,1],
                          [9,10,11,1]])
projection_matrix  = 0


# Replace 'file_path.csv' with the actual path to your CSV file
file_path = 'debug_python_code/cyberzoo_poles_panels_mats/20190121-142943.csv'

# Read the CSV file into a DataFrame
df = pd.read_csv(file_path)

# Separate each column into individual variables
x_col = df.iloc[:, 1]  
y_col = df.iloc[:, 2]  
z_col = df.iloc[:, 3]

roll_col = df.iloc[:, 7] 
pitch_col = df.iloc[:, 8] 
yaw_col = df.iloc[:, 9] 




# # Euler angles representing orientation difference between the frames
# roll = 0.0  # Replace with actual roll angle
# pitch = 0.0  # Replace with actual pitch angle
# yaw = 0.0  # Replace with actual yaw angle
# x = 0
# y = 0
# z = 0

#definig final array for corner pixel coordinates
uvs = np.zeros((x_col.shape(0),2))


for i in range(0, x_col.shape(0)):

    #Find Transformation matrix from orginial frame to drone
    T_drone_org = Find_T_drone_org(roll_col[i], pitch_col[i], yaw_col[i], x_col[i], y_col[i], z_col[i])

    #Find corners coordinates in drone frame
    cor_coord_drone = corners_object_to_drone(cor_coord_org, T_drone_org)

    #Find corner pixel coordinates for the image 
    uvs_row = project_points(projection_matrix,cor_coord_drone)

    uvs[i] = uvs_row

