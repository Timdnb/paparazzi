import cv2
import matplotlib.pyplot as plt
import os
import numpy as np
from sklearn.linear_model import LinearRegression, RANSACRegressor, Ridge
import copy

# Load dataset
example_files = os.listdir('./cyberzoo_poles_panels_mats/20190121-142935')
example_files.sort()
example_files = [os.path.join('./cyberzoo_poles_panels_mats/20190121-142935', filename) for filename in example_files]

def remove_mess(image, window_size=15, thresh=.35):
    pixels_threshold = window_size**2 * thresh
    for y in range(0, image.shape[0] - window_size + 1, window_size):
        for x in range(0, image.shape[1] - window_size + 1, window_size):
            
            roi = image[y:y+window_size, x:x+window_size]
            
            # Count white pixels 
            white_pixels = np.sum(roi == 255)
            
            # If there are 20 or more white pixels, replace the entire ROI with black pixels
            if white_pixels >= pixels_threshold:
                image[y:y+window_size, x:x+window_size] = 0
                # print("Removed patch")
    return image

def perform_linear_regression_with_guess(binary_image, model, x_range):
    ACCEPT_REGION = 5
    """
    Given a portion of the image we try to find the horizon
    """
    new_x = []
    new_y = []
    counter = 0
    for x in x_range:
        distance = ACCEPT_REGION * 5 + 1
        best_y = -1
        prediction = int(model.predict([[x]])[0][0])
        for y in range(max(0, prediction - ACCEPT_REGION * 5), min(binary_image.shape[1], prediction + ACCEPT_REGION * 5)):
            # Maybe try to save from both direction if similar
            new_distance = abs(y - prediction)
            if new_distance < distance:
                best_y = y
                distance = new_distance
        if best_y != -1:
            counter += 1
            new_x.append(x)
            new_y.append(y)

    if counter < 2:
        print("No white pixels found in the image.")
        return model
    model = LinearRegression()
    model.fit(np.array(new_x).reshape(-1, 1), np.array(new_y).reshape(-1, 1))

    counter = 0
    old_x = new_x
    new_x = []
    new_y = []
    counter = 0
    for x in old_x:
        distance = ACCEPT_REGION + 1
        best_y = -1
        prediction = int(model.predict([[x]])[0][0])
        for y in range(max(0, prediction - ACCEPT_REGION), min(binary_image.shape[1], prediction + ACCEPT_REGION)):
            # Maybe try to save from both direction if similar
            new_distance = abs(y - prediction)
            if new_distance < distance:
                best_y = y
                distance = new_distance
        if best_y != -1:
            counter += 1
            new_x.append(x)
            new_y.append(y)

    if counter < 2:
        #print("No white pixels found in the image.")
        return model
    model = LinearRegression()
    model.fit(np.array(new_x).reshape(-1, 1), np.array(new_y).reshape(-1, 1))


    return model

def detect_horizons(frame):
    """
    Return the model for each orizon line up to two, and the x for which they meet
    If there is only one line x_separation is equal to frame.shape[1]
    """



exit = False
i = 195
j = 100
window_name = "Original vs Edge Detected"
cv2.namedWindow(window_name)

while not exit:
    file = example_files[i]
    frame  = cv2.imread(file)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    blurred = cv2.GaussianBlur(src=gray, ksize=(15, 15), sigmaX=0.9) 
    edges = cv2.Canny(blurred,15, 20) 
    edges = remove_mess(edges)
    colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    y_min = 120
    lines = cv2.HoughLinesP(edges[y_min:, :], 1, 1 * np.pi/180, 10, minLineLength=30, maxLineGap=10)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(colored, (x1, y1 + y_min), (x2, y2 + y_min), (0, 0, 255), 2)
    combined_image = cv2.hconcat([colored, frame])
    cv2.imshow(window_name, combined_image)
    cv2.setWindowTitle(window_name, f"Original vs Edge Detected - Index {i}")
    key = cv2.waitKey(0)
    if key == ord("q"):
        exit = True
    if key == 83:
        i += 1
    elif key == 81:
        i = max(0, i - 1)