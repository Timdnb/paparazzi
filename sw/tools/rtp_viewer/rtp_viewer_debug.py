#! /usr/bin/python

import cv2
import sys
import argparse
from os import path, getenv
import os

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

# See the issue and solution here: https://github.com/opencv/opencv/issues/10328
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# See the issue and solution here: https://github.com/opencv/opencv/issues/10328
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'

class RtpViewer:
    running = False
    scale = 1
    rotate = 0
    frame = None
    mouse = dict()

    def __init__(self, src):
        # Create the video capture device
        self.cap = cv2.VideoCapture(src)

        # Start the ivy interface
        self.ivy = IvyMessagesInterface("RTPviewer", start_ivy=False)
        self.ivy.start()

        # Create a named window and add a mouse callback
        cv2.namedWindow('rtp')
        cv2.setMouseCallback('rtp', self.on_mouse)

    def run(self):
        self.running = True

        # Start an 'infinite' loop
        while self.running:
            # Read a frame from the video capture
            ret, self.frame = self.cap.read()

            # Quit if frame could not be retrieved
            if not ret:
                break
            
            self.frame = draw_3D_to_2D(self.frame)
            # Run the computer vision function
            self.cv()

            # Process key input
            self.on_key(cv2.waitKey(1) & 0xFF)

    def cv(self):
        # Rotate the image by increments of 90
        if self.rotate % 2:
            self.frame = cv2.transpose(self.frame)

        if self.rotate > 0:
            self.frame = cv2.flip(self.frame, [1, -1, 0][self.rotate - 1])

        # If a selection is happening
        if self.mouse.get('start'):
            # Draw a rectangle indicating the region of interest
            cv2.rectangle(self.frame, self.mouse['start'], self.mouse['now'], (0, 255, 0), 2)

        if self.scale != 1:
            h, w = self.frame.shape[:2]
            self.frame = cv2.resize(self.frame, (int(self.scale * w), int(self.scale * h)))

        # Show the image in a window
        cv2.imshow('rtp', self.frame)

    def on_key(self, key):
        if key == ord('q'):
            self.running = False

        if key == ord('r'):
            self.rotate = (self.rotate + 1) % 4
            self.mouse['start'] = None

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.rotate == 0:
            self.mouse['start'] = (x, y)

        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse['start'] = None

        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse['now'] = (x, y)

        if event == cv2.EVENT_LBUTTONUP:
            # If mouse start is defined, a region has been selected
            if not self.mouse.get('start'):
                return

            # Obtain mouse start coordinates
            sx, sy = self.mouse['start']

            # Create a new message
            msg = PprzMessage("datalink", "VIDEO_ROI")
            msg['ac_id'] = None
            msg['startx'] = sx
            msg['starty'] = sy
            msg['width'] = abs(x - sx)
            msg['height'] = abs(y - sy)
            msg['downsized_width'] = self.frame.shape[1]

            # Send message via the ivy interface
            self.ivy.send_raw_datalink(msg)

            # Reset mouse start
            self.mouse['start'] = None

    def cleanup(self):
        # Shutdown ivy interface
        self.ivy.shutdown()

import numpy as np
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

#camera projection matrix 
projection_matrix  = np.array([[30,0,120,0],
                               [0,30,260,0],
                               [0,0,1,0],
                               [0,0,0,1]])

def draw_3D_to_2D(frame):
    T_drone_org = np.eye(4)
    cor_coord_drone = np.dot(T_drone_org, np.array([2, 1, 0, 1]).T).T
    T_camera_drone = Find_T_camera_drone()
    cor_coord_camera = np.dot(T_camera_drone, cor_coord_drone.T).T
    uv= project_points(projection_matrix,cor_coord_camera)[:2]
    print(uv)
    y, x = 240- int(uv[0]), int(uv[1])  # Assuming uvs contains (x, y) coordinates
    # y, x = int(220), int(400)
    cv2.circle(frame, (y, x), 5, (0, 0, 255), -1)  # Draw a circle with radius 5 and red color
    return frame

if __name__ == '__main__':
    import sys
    import os
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=int, default=5000, help="The port number to open for the RTP stream (5000 or 6000)")
    parser.add_argument("-s", "--scale", type=float, default=1., help="The scaling factor to apply to the incoming video stream (default: 1)")
    parser.add_argument("-r", "--rotate", type=int, default=0, help="The number of clockwise 90deg rotations to apply to the stream [0-3] (default: 0)")
    
    args = parser.parse_args()

    filename = os.path.dirname(os.path.abspath(__file__)) + "/rtp_" + str(args.port) + ".sdp"

    viewer = RtpViewer(filename)
    viewer.scale = args.scale
    viewer.rotate = args.rotate

    if not viewer.cap.isOpened():
        viewer.cleanup()
        raise IOError("Can't open video stream")

    viewer.run()
    viewer.cleanup()