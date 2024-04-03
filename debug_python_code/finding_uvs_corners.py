import numpy as np
import pandas as pd


def Find_T_drone_org(roll, pitch, yaw):
#finding homogenous transformation matrix from original cyberzoo frame to drone frame

    #---ANGLE VARIABLES---
    #because input is expressed as rotation and translation from original to drone frame we need to add minus in front of variables
    roll = -roll 
    pitch = -pitch 

    #hard_coded yaw for calibration at the starting postion
    yaw_hard_coded_calibration = np.radians(153)   
    yaw = -(yaw - yaw_hard_coded_calibration)

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

    # Translation is taken care outside the function
    translation_vector = np.array([0, 0, 0])  

    # Homogeneous transformation matrix
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = translation_vector
    homogeneous_matrix[3, :] = np.array([0,0,0,1])
    
    T_drone_org = homogeneous_matrix
    
    return T_drone_org


def Find_T_camera_drone():
#creating homogenous transofrmation matrix from drone frame to camera frame

    #adding minus for angle variables
    #the rotation makes sure the z dimension is in the image plane
    y_rot = - np.radians(90)
    x_rot =  - np.radians(0)

    # Convert Euler angles to rotation matrices
    R_roll = np.array([[1, 0, 0],
                [0, np.cos(x_rot), -np.sin(x_rot)],
                [0, np.sin(x_rot), np.cos(x_rot)]])

    R_pitch = np.array([[np.cos(y_rot), 0, np.sin(y_rot)],
                        [0, 1, 0],
                        [-np.sin(y_rot), 0, np.cos(y_rot)]])
    
    #order for rotations
    rotation_matrix = np.dot( R_pitch , R_roll)

    
    #forming homogenous matrix, no translation
    homogeneous_matrix = np.zeros([4,4])
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[3, :] = np.array([0,0,0,1])

    T_camera_drone = homogeneous_matrix

    return T_camera_drone



def project_points(projection_matrix, corners_drone):
#finding uv position in the image based on the coordinates in camera frame and projection matrix
    """
    projection_matrix: The projection matrix based on the intrinsic camera calibration.
    points: Nx[x,y,z,1.0] np.array of the points that need to be projected to the camera image from drone frame.
    return: Nx[u,v] rounded coordinates of the points in the camera image as int data type.
    """

    #-------------------------the corrected projection------------------
    assert corners_drone.shape[-1] == 4
    uvs = []
    # points = points[:,:3]
    # print(corners_drone)
    for row in corners_drone:
        projected_points_homogeneous = projection_matrix @ row.T

        # Normalize the homogeneous coordinates
        normalized_points = (projected_points_homogeneous[:2] / projected_points_homogeneous[2]).T

        # Round the coordinates to integers
        uvs.append(np.round(normalized_points).astype(int))
    uvs = np.array(uvs)
    # print(uvs)
    
    return uvs

def rel_3D_to_2D(point):
#find uvs from corner cooridnates in drone frame (after drone translation and rotation)
    T_camera_drone = Find_T_camera_drone()
    cor_coord_camera = np.dot(T_camera_drone, point.T).T
    uvs_row = project_points(projection_matrix, cor_coord_camera)
    return uvs_row



#---------------------------MAIN-----------------------------------------------------------------


#camera projection matrix 
projection_matrix  = np.array([[300,0,120,0],
                               [0,300,260,0],
                               [0,0,1,0],
                               [0,0,0,1]])

# Replace 'file_path.csv' with the actual path to the CSV file
# file_path = './cyberzoo_poles_panels_mats/20190121-142943.csv'
file_path = './sim_poles_panels_mats/20190121-161955.csv'

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


#main loop for finding all 4 corners for a datapoint
for i in range(0, x_col.shape[0]):

    #for now for every coordinate update we want original corner coordinates
    cor_coord_org = np.array([[4,4.,0.,1.],
                          [-4.,-4.,0.,1.],
                          [4.,-4.,0.,1.],
                          [-4.,4.,0.,1.]])
    if i == 0:
        print("org")
        print(cor_coord_org)
    #New corner coordinates in drone frame after translation from original frame
    cor_coord_org[:,0] = cor_coord_org[:,0] - x_col[i] 
    cor_coord_org[:,1] = cor_coord_org[:,1] - y_col[i] 
    cor_coord_org[:,2] = cor_coord_org[:,2] - z_col[i]

    if i == 0:
        print("org after translation")
        print(cor_coord_org)

    T_drone_org = Find_T_drone_org(roll_col[i], pitch_col[i], yaw_col[i])

    #new corner coordinates after rotation
    cor_coord_drone = np.dot(T_drone_org, cor_coord_org.T).T

    if i == 0:
        print("org after rotation")
        print(cor_coord_drone)

    #find uvs from corner cooridnates in drone frame (after drone translation and rotation)
    uvs_row = rel_3D_to_2D(cor_coord_drone)

    #fill a row in the final uvs matrix
    uvs[i,:,:] = uvs_row


    #for testing save every 12 datapoint (because per frame you have about 12 datapoints)
    uvs_testing = uvs#[::11]
    # Save the array to the specified file path
    np.save("./saved_numpy.npy", uvs_testing)

# print(uvs_testing[0])

# def draw_3D_to_2D(frame):
#     T_drone_org = np.eye(4)
#     cor_coord_drone = np.dot(T_drone_org, np.array([2, 1, 0]).T).T
#     T_camera_drone = Find_T_camera_drone()
#     cor_coord_camera = np.dot(T_camera_drone, cor_coord_drone.T).T
#     uv = project_points(projection_matrix,cor_coord_camera)[:2]
#     print(uv)
#     y, x = 240- int(uv[0]), int(uv[1])  # Assuming uvs contains (x, y) coordinates
#     # y, x = int(220), int(400)
#     cv2.circle(frame, (y, x), 5, (0, 0, 255), -1)  # Draw a circle with radius 5 and red color
#     return uvs