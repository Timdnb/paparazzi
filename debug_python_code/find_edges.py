import cv2
import matplotlib.pyplot as plt
import os
import numpy as np
from sklearn.linear_model import LinearRegression, RANSACRegressor, Ridge
import copy
from utils import *

# Load dataset
example_files = os.listdir('./cyberzoo_poles_panels_mats/20190121-142935')
example_files.sort()
example_files = [os.path.join('./cyberzoo_poles_panels_mats/20190121-142935', filename) for filename in example_files]

def perform_linear_regression(binary_image):
    ACCEPT_REGION = 10
    """
    Given a portion of the image we try to find the horizon
    We use RANSACRegressor to try to be resiliant to the numerous outliers
    """
    white_pixels = np.argwhere(binary_image == 255)  # Get coordinates of white pixels
    num_pixels = white_pixels.shape[0]


    # Initialize arrays to store x and y coordinates of white pixels
    x_coords = white_pixels[:, 1]
    y_coords = white_pixels[:, 0]
    sorted_indices = np.argsort(x_coords)
    x_coords_sorted = x_coords[sorted_indices]
    y_coords_sorted = y_coords[sorted_indices]


    # Perform linear regression on the coordinates
    #base_estimator = Ridge(alpha=100)
    #model = RANSACRegressor(base_estimator=base_estimator)
    #model = RANSACRegressor()
    model = LinearRegression()
    if num_pixels < 2:
        print("No white pixels found in the image.")
        return None
    model.fit(x_coords.reshape(-1, 1), y_coords.reshape(-1, 1))
    new_x = []
    new_y = []
    possible_x = None
    for x, y in zip(x_coords, y_coords):
        if x == possible_x:
            list_y.append(y)
            continue
            
        if possible_x is not None:
            prediction = model.predict([[possible_x]])[0]
            closest_y = prediction + ACCEPT_REGION + 1
            for old_y in list_y:
                if abs(old_y - prediction) < abs(closest_y - prediction):
                    closest_y = old_y
            if abs(closest_y - prediction) <= ACCEPT_REGION:
                new_x.append(x)
                new_y.append(closest_y)

        possible_x = x
        list_y = [y]

    model = LinearRegression()
    if len(new_x) < 2:
        print("No white pixels found in the image.")
        return None
    model.fit(np.array(new_x).reshape(-1, 1), np.array(new_y).reshape(-1, 1))

    return model

def assign_colors_to_compatible_models(models, threshold = 20):
    """
    Just a quick clustering of the different predictions for the horizon
    """
    color_map = {}
    color_index = 0

    for i in range(len(models)):
        compatible = np.full(color_index, 0)
        for j in range(i):

            model1 = models[i]
            model2 = models[j]

            y_pred_1 = model1.predict([[300]])[0]
            y_pred_2 = model2.predict([[300]])[0]
            error = abs(y_pred_1 - y_pred_2)
            compatible[color_map[j]] = max(error, compatible[color_map[j]])
            
        if compatible.size == 0 or all(compatible >= threshold):
            color_map[i] = color_index
            color_index += 1
        else:
            color_map[i] = np.argmin(compatible)

    return color_map

def detect_horizons(frame):
    """
    Return the model for each orizon line up to two, and the x for which they meet
    If there is only one line x_separation is equal to frame.shape[1]
    """
    original = frame.copy()
    
    # This was an attempt to make the blue and black patches around the arena look the same (I wanted a gradient between the green and blue grass)
    #frame[:, :, 2] = 0
    #hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ## Define lower and upper bounds for blue color in HSV
    #lower_blue = np.array([90, 0, 0])
    #upper_blue = np.array([255, 255, 255])
    ## Create a mask to extract blue areas
    #blue_mask = cv2.inRange(frame, lower_blue, upper_blue)
    ## Print some information about the blue mask
    #print("Pixels detected as blue:", np.count_nonzero(blue_mask))
    #print("Shape of the blue mask:", blue_mask.shape)
    ## Darken the blue areas in the original image
    #darkened_image = frame.copy()
    ##darkened_image[blue_mask != 0, 0] = 80   # Darken blue areas by dividing pixel values by 2
    ##darkened_image[blue_mask != 0, 1] = 60
    #frame = darkened_image.copy()

    # Convert the frame to grayscale for edge detection 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    # Apply Gaussian blur to reduce noise and smoothen edges 
    blurred = cv2.GaussianBlur(src=gray, ksize=(15, 15), sigmaX=0.9) 
    # Perform Canny edge detection 
    edges = cv2.Canny(blurred,15, 20) 
    #edges = cv2.GaussianBlur(src=edges, ksize=(5, 5), sigmaX=0.9) 
    rgb_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    edges = remove_mess(rgb_edges)
    edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
    edges_debug = edges.copy()
    colored = cv2.cvtColor(edges_debug, cv2.COLOR_GRAY2BGR)
    
    
    # Split the image in regions where we try to interpolate the horizon line
    # Frame diveded in two to separate the two walls
    horizons = []
    for offset in [0, edges.shape[1] // 2]:
        # Should be based on the corner position
        x_min = offset
        x_max = offset+edges.shape[1] // 2
        wrong = []
        models = []
        #y_min = 80
        y_max = edges.shape[0] - 30
        y_min = 110
        #y_max = 200 - 40
        for y in range(y_min, y_max, 10):
            #sample = copy.deepcopy(edges[y:y+50, x_min:x_max])
            sample = edges[y:y+50, x_min:x_max]

            model = perform_linear_regression(sample)
            if model is None:
                continue
            #model.estimator_.intercept_[0] = model.predict([[- x_min]])[0] + y
            model.intercept_[0] = model.predict([[- x_min]])[0] + y
            #model.intercept_[0] += y
            models.append(model)

        color_map = assign_colors_to_compatible_models(models)
        #print(color_map)
        #print([list(color_map.values()).count(x) for x in color_map.values()])
        rgb_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255),
                      (255, 0, 255), (128, 128, 0), (128, 0, 128), (0, 128, 128), (128, 128, 128),
                      (255, 128, 0), (255, 0, 128), (0, 255, 128), (128, 255, 0), (0, 128, 255),
                      (128, 0, 255), (255, 128, 128), (128, 255, 128), (128, 128, 255), (255, 255, 128)]

        
        # We try to find the correct model by looking at how many points are on the line
        # If there are too many points around the line we supose it is the mat and not the horizon
        for model, color, y in zip(models, color_map.values(), range(y_min, y_max, 10)):
            #x1 = x_min
            #y1 = int(model.predict([[x1]])[0][0])
            #x2 = x_max - 1
            #y2 = int(model.predict([[x2]])[0][0])
            ##print((x1, y1), (x2, y2))
            ## Draw the regression line on the image
            #line_thickness = 1
            #cv2.line(colored, (x1, y1), (x2, y2), (0, 0, 128), line_thickness)
            # if x_min == 0:
            #     cv2.imshow(f"sample: {y}", sample)
            #assert (sample != edges[y:y+50, x_min:x_max]).any()
            model.wrong = []
            for x in range(x_min, x_max, 5):
                y_predicted = int(model.predict([[x]]).flatten()[0])
                ys_actual_low = np.where(edges[y_predicted - 3:y_predicted + 3, x] == 255)[0]
                ys_actual_high = np.where(edges[y_predicted - 5:y_predicted + 10, x] == 255)[0]
                if len(ys_actual_low) == 0 or len(ys_actual_high) > 2:
                    model.wrong.append((x, int(y_predicted)))
        
            #print(y, len(model.wrong))
        horizons.append(min(models, key = lambda model: len(model.wrong)))
    return horizons, edges.shape[1] // 2, colored


exit = False
i = 250
j = 100
window_name = "Original vs Edge Detected"
cv2.namedWindow(window_name)

while not exit:
    file = example_files[i]
    frame  = cv2.imread(file)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    models, x_separation, colored = detect_horizons(frame)
    
    assert len(models) == 2 if x_separation < frame.shape[1] else len(models) == 1

    for k, model in enumerate(models):
        for x in range([0, x_separation][k], [frame.shape[1] // 2 - x_separation, frame.shape[1] // 2][k], 5):
            y_predicted = int(model.predict([[x]]).flatten()[0])
            ys_actual_low = np.where(edges[y_predicted - 3:y_predicted + 3, x] == 255)[0]
            ys_actual_high = np.where(edges[y_predicted - 5:y_predicted + 10, x] == 255)[0]
            if len(ys_actual_low) == 0 or len(ys_actual_high) > 2:
                wrong.append((x, int(y_predicted)))
        
        
        for x0, y0 in model.wrong:
            cv2.circle(colored, (x0, y0), 2, (0, 0, 255), -1)

    # I don't have the model anymore
    #cv2.imshow("sample", sample)
    #line_color = (255)  # Red color in BGR format
    #line_thickness = 2  # Adjust thickness as needed
    #cv2.line(edges, (0, 110+model.intercept_), (edges.shape[1], j), line_color, line_thickness)

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