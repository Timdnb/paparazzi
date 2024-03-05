import cv2
import matplotlib.pyplot as plt
import os
import numpy as np

example_files = os.listdir('./cyberzoo_poles_panels_mats/20190121-142935')
example_files.sort()
example_files = [os.path.join('./cyberzoo_poles_panels_mats/20190121-142935', filename) for filename in example_files]
from sklearn.linear_model import LinearRegression, RANSACRegressor, Ridge

def perform_linear_regression(binary_image):
    white_pixels = np.argwhere(binary_image == 255)  # Get coordinates of white pixels
    num_pixels = white_pixels.shape[0]


    # Initialize arrays to store x and y coordinates of white pixels
    x_coords = white_pixels[:, 1]
    y_coords = white_pixels[:, 0]

    # Perform linear regression on the coordinates
    #base_estimator = Ridge(alpha=100)
    #model = RANSACRegressor(base_estimator=base_estimator)
    model = RANSACRegressor()
    #model = LinearRegression()
    if num_pixels < 2:
        print("No white pixels found in the image.")
        model.fit([[0], [1]], [[0], [0]])
    else:
        model.fit(x_coords.reshape(-1, 1), y_coords.reshape(-1, 1))
    return model

def assign_colors_to_compatible_models(models, threshold = 20):
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


exit = False
i = 341
j = 100
window_name = "Original vs Edge Detected"
cv2.namedWindow(window_name)

while not exit:
    file = example_files[i]
    frame  = cv2.imread(file)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    original = frame.copy()
    #frame[:, :, 2] = 0
    
    #hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for blue color in HSV
    lower_blue = np.array([90, 0, 0])
    upper_blue = np.array([255, 255, 255])
    
    # Create a mask to extract blue areas
    blue_mask = cv2.inRange(frame, lower_blue, upper_blue)

    # Print some information about the blue mask
    print("Pixels detected as blue:", np.count_nonzero(blue_mask))
    print("Shape of the blue mask:", blue_mask.shape)

    
    # Darken the blue areas in the original image
    darkened_image = frame.copy()
    #darkened_image[blue_mask != 0, 0] = 80   # Darken blue areas by dividing pixel values by 2
    #darkened_image[blue_mask != 0, 1] = 60
    
    frame = darkened_image.copy()

    # Convert the frame to grayscale for edge detection 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        
    # Apply Gaussian blur to reduce noise and smoothen edges 
    blurred = cv2.GaussianBlur(src=gray, ksize=(101, 101), sigmaX=0.5) 
    
    # Perform Canny edge detection 
    edges = cv2.Canny(blurred,15, 20) 
    #edges = cv2.GaussianBlur(src=edges, ksize=(5, 5), sigmaX=0.25) 
    edges_debug = edges.copy()
    colored = cv2.cvtColor(edges_debug, cv2.COLOR_GRAY2BGR)

    for offset in [0, edges.shape[1] // 2]:
        # Should be based on the corner position
        x_min = offset
        x_max = offset+edges.shape[1] // 2
        wrong = {}
        models = []
        y_min = 80
        y_max = edges.shape[0] - 40
        for y in range(y_min, y_max, 10):
            wrong[y] = []
            sample = edges[y:y+50, x_min:x_max].copy()

            model = perform_linear_regression(sample)
            model.estimator_.intercept_[0] += y
            #model.intercept_[0] += y
            models.append(model)

        color_map = assign_colors_to_compatible_models(models)
        #print(color_map)
        #print([list(color_map.values()).count(x) for x in color_map.values()])
        rgb_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255),
                      (255, 0, 255), (128, 128, 0), (128, 0, 128), (0, 128, 128), (128, 128, 128),
                      (255, 128, 0), (255, 0, 128), (0, 255, 128), (128, 255, 0), (0, 128, 255),
                      (128, 0, 255), (255, 128, 128), (128, 255, 128), (128, 128, 255), (255, 255, 128)]

        
        for model, color, y in zip(models, color_map.values(), range(y_min, y_max, 10)):
            #x1 = 0
            #y1 = int(model.predict([[x1]])[0][0])
            #x2 = x_max - 1
            #y2 = int(model.predict([[x2]])[0][0])
            # Draw the regression line on the image
            #line_thickness = 2
            #cv2.line(colored, (x1, y1), (x2, y2), rgb_colors[color], line_thickness)
            #cv2.imshow(f"sample: {y}", sample)
            
            for x in range(x_min, x_max, 5):
                y_predicted = int(model.predict([[x]]).flatten()[0])
                ys_actual_low = np.where(edges[y_predicted - 3:y_predicted + 3, x] == 255)[0]
                ys_actual_high = np.where(edges[y_predicted - 5:y_predicted + 10, x] == 255)[0]
                if len(ys_actual_low) == 0 or len(ys_actual_high) > 2:
                    wrong[y].append((x, int(y_predicted)))
        
        
        y = min(wrong, key = lambda k: len(wrong[k]))
        sorted_items = sorted(wrong.items(), key=lambda x: len(x[1]))
        for key, value in sorted_items:
            pass
            #print(f"Key: {key}, Length: {len(value)}")
        for x0, y0 in wrong[y]:
            cv2.circle(colored, (x0, y0), 2, (0, 0, 255), -1)

    # I don't have the model anymore

    #cv2.imshow("sample", sample)
    #line_color = (255)  # Red color in BGR format
    #line_thickness = 2  # Adjust thickness as needed
    #cv2.line(edges, (0, 110+model.intercept_), (edges.shape[1], j), line_color, line_thickness)

    combined_image = cv2.hconcat([original, colored])
    cv2.imshow(window_name, combined_image)
    cv2.setWindowTitle(window_name, f"Original vs Edge Detected - Index {i}")
    key = cv2.waitKey(0)
    if key == ord("q"):
        exit = True
    if key == 83:
        i += 1
    elif key == 81:
        i = max(0, i - 1)