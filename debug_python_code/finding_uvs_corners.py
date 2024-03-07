import numpy as np
import pandas as pd


def Find_T_drone_org(roll, pitch, yaw, x, y, z):

    #because input is expressed as rotation and translation from original to drone we need to
    #add minus in front so that we actually know what should be the coordinates in the drone frame 
    #if the points were transformed from drone to original frame. Thus we can know that the drone
    #would see in its frame

    roll = -roll
    pitch = -pitch
    yaw = - yaw
    x = - x
    y = - y
    z = - z
 
 
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
    
    # rotation_matrix = np.dot(R_pitch, np.dot(R_yaw, R_roll))
    # print(rotation_matrix)

    # Translation vector representing displacement between the origins of the frames
    # so that the croners are expressed in coordinates of drone frame
    translation_vector = np.array([x, y, z])  

    # Homogeneous transformation matrix
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = translation_vector
    homogeneous_matrix[3, :] = np.array([0,0,0,1])
    
    T_drone_org = homogeneous_matrix

    # T_drone_org = np.eye(4)
    
    return T_drone_org


def Find_T_camera_drone():

    #minus because we go back to give coordinates in camera frame for points positioned as in 
    #drone frame

    # y_rot = - np.radians(90)
    z_rot = - np.radians(90)

    x_rot =  - np.radians(90)
    

    # Convert Euler angles to rotation matrices
    R_roll = np.array([[1, 0, 0],
                [0, np.cos(x_rot), -np.sin(x_rot)],
                [0, np.sin(x_rot), np.cos(x_rot)]])

    # R_pitch = np.array([[np.cos(y_rot), 0, np.sin(y_rot)],
    #                     [0, 1, 0],
    #                     [-np.sin(y_rot), 0, np.cos(y_rot)]])
    
    R_yaw = np.array([[np.cos(z_rot), -np.sin(z_rot), 0],
                    [np.sin(z_rot), np.cos(z_rot), 0],
                    [0, 0, 1]])
    
    #Find correct order for rotations
    rotation_matrix = np.dot( R_yaw , R_roll)
    

    #forming homogenous matrix, no translation

    homogeneous_matrix = np.zeros([4,4])
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[3, :] = np.array([0,0,0,1])

    T_camera_drone = homogeneous_matrix

    return T_camera_drone



def project_points(projection_matrix, corners_drone):
    """
    projection_matrix: The projection matrix based on the intrinsic camera calibration.
    points: Nx[x,y,z,1.0] np.array of the points that need to be projected to the camera image from drone frame.
    return: Nx[u,v] rounded coordinates of the points in the camera image as int data type.
    """
    assert corners_drone.shape[-1] == 4
    uvs = None   # calculate points in image plane
    
    coordinates = np.dot(projection_matrix, corners_drone.T).T
    coordinates = coordinates[:, :2] / coordinates[:, 2:] #normalisation

    #giving uvs shape [4,2]
    uvs = coordinates[:, :2]
    uvs = np.round(uvs).astype(int)
    
    return uvs



#---------------------------MAIN-----------------------------------------------------------------


#corner coordiantes in original frame
cor_coord_org = np.array([[4,4,0,1],
                          [-4,-4,0,1],
                          [4,-4,0,1],
                          [-4,4,0,1]])

#camera projection matrix 
projection_matrix  = np.array([[30,0,120,0],
                               [0,30,260,0],
                               [0,0,1,0],
                               [0,0,0,1]])

#this is experimental projecton matrix because the one above creates huge coordinates
# projection_matrix  = np.array([[30,0,64,0],
#                                [0,30,64,0],
#                                [0,0,1,0],
#                                [0,0,0,1]])

# Replace 'file_path.csv' with the actual path to your CSV file
file_path = './cyberzoo_poles_panels_mats/20190121-142943.csv'

# Read the CSV file into a DataFrame
df = pd.read_csv(file_path)

# Separate each column into individual variables
x_col = df.iloc[:, 1]  
y_col = df.iloc[:, 2]  
z_col = df.iloc[:, 3]

roll_col = df.iloc[:, 7] 
pitch_col = df.iloc[:, 8] 
yaw_col = df.iloc[:, 9] 


#definig final array for corner pixel coordinates (3D)
uvs = np.zeros((x_col.shape[0],4, 2))

#main loop for findinf all 4 corners for a datapoint
for i in range(0, x_col.shape[0]):

    #Find Transformation matrix from orginial frame to drone
    T_drone_org = Find_T_drone_org(roll_col[i], pitch_col[i], yaw_col[i], x_col[i], y_col[i], z_col[i])

    #Find corners coordinates in drone frame
    cor_coord_drone = np.dot(T_drone_org, cor_coord_org.T).T

    #Find 3D coordinates in camera frame
    T_camera_drone = Find_T_camera_drone()
    cor_coord_camera = np.dot(T_camera_drone, cor_coord_drone.T).T
    

    #Find corner pixel coordinates for the image 
    uvs_row = project_points(projection_matrix,cor_coord_camera)

    #fill a row in the final uvs matrix
    uvs[i,:,:] = uvs_row


    #for testing save every 12 datapoint (because per frame you have about 12 datapoints)
    uvs_testing = uvs[::12]
    # Save the array to the specified file path
    np.save("./saved_numpy.npy", uvs_testing)

print(uvs_testing[1])