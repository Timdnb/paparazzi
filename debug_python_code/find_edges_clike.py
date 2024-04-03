### Quick file for prototyping of the openCV approach
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

def remove_mess(image, window_size=10, thresh=.4):
    pixels_threshold = window_size**2 * thresh
    for y in range(0, image.shape[0] - window_size + 1, window_size):
        for x in range(0, image.shape[1] - window_size + 1, window_size):
            
            roi = image[y:y+window_size, x:x+window_size]
            
            # Count white pixels 
            white_pixels = np.sum(roi == 255) * 3
            
            # If there are 20 or more white pixels, replace the entire ROI with black pixels
            if white_pixels >= pixels_threshold:
                image[y:y+window_size, x:x+window_size] = 0
                # print("Removed patch")
    return image

def perform_linear_regression_with_guess(binary_image, model, x_range):
    ACCEPT_REGION = 4
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
        for y in range(max(0, prediction - ACCEPT_REGION * 4), min(binary_image.shape[0] - 1, prediction + ACCEPT_REGION * 4)):
            if not binary_image[y][x]:
                continue
            # Maybe try to save from both direction if similar
            new_distance = abs(y - prediction)
            if new_distance < distance:
                best_y = y
                distance = new_distance
        if best_y != -1:
            counter += 1
            new_x.append(x)
            new_y.append(y)

    print(new_x, new_y)
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
        for y in range(max(0, prediction - ACCEPT_REGION), min(binary_image.shape[0] - 1, prediction + ACCEPT_REGION)):
            if not binary_image[y][x]:
                continue
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
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    blurred = cv2.GaussianBlur(src=gray, ksize=(15, 15), sigmaX=0.9) 
    edges = cv2.Canny(blurred,15, 20) 
    edges = remove_mess(edges)
    colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    
    # Split the image in regions where we try to interpolate the horizon line
    # Frame diveded in two to separate the two walls
    horizons = []
    for offset in [0, edges.shape[1] // 2]:
        # Should be based on the corner position
        x_min = offset
        x_max = offset+edges.shape[1] // 2 - 1
        wrong = []
        models = []
        #y_min = 80
        y_min = 130
        y_max = 210
        if offset == 0:
            angle = -2/3
        else:
            angle = 1/4
        for y in range(y_min, y_max, 30):
            #sample = copy.deepcopy(edges[y:y+50, x_min:x_max])
            guess = LinearRegression()
            if x_min == 0:
                y_x_min = y - angle * (x_max - x_min)
                y_x_max = y
            else:
                y_x_min = y
                y_x_max = y + angle * (x_max - x_min)
            guess.fit([[x_min], [x_max]], [[y_x_min], [y_x_min]])
            model = perform_linear_regression_with_guess(edges, guess, range(x_min, x_max))
            print(y, x_min, x_max, *model.predict([[x_min], [x_max]]))
            #sample = edges[y:y+50, x_min:x_max]

            #model = perform_linear_regression(sample)
            if model is None:
                continue
            #model.estimator_.intercept_[0] = model.predict([[- x_min]])[0] + y
            #model.intercept_[0] = model.predict([[- x_min]])[0][0]# + y
            models.append(model)

        # We try to find the correct model by looking at how many points are on the line
        # If there are too many points around the line we supose it is the mat and not the horizon
        for model, y in zip(models, range(y_min, y_max, 10)):
            model.wrong = []
            for x in range(x_min, x_max, 5):
                y_predicted = int(model.predict([[x]]).flatten()[0])
                if y_predicted > edges.shape[0]:
                    break
                ys_actual_low = np.where(edges[y_predicted - 3:y_predicted + 3, x] == 255)[0]
                ys_actual_high = np.where(edges[y_predicted - 5:y_predicted + 10, x] == 255)[0]
                if len(ys_actual_low) == 0 or len(ys_actual_high) > 2:
                    model.wrong.append((x, int(y_predicted)))
        
            #print(y, len(model.wrong))
        old_model = min(models, key = lambda model: len(model.wrong))
        old_model.color = (0, 0, 100)
        guess = old_model
        #guess.intercept_[0] = old_model.predict([[x_min]])[0][0]
        model = perform_linear_regression_with_guess(edges, guess, range(x_min, x_max))
        #model.intercept_[0] = model.predict([[- x_min]])[0][0]
        model.wrong = []
        model.color = model.color = (0, 255, 0)
        for x in range(x_min, x_max, 2):
            y_predicted = int(model.predict([[x]]).flatten()[0])
            if y_predicted > edges.shape[0]:
                    continue
            ys_actual_low = np.where(edges[y_predicted - 5:y_predicted + 5, x] == 255)[0]
            ys_actual_high = np.where(edges[y_predicted - 10:y_predicted + 10, x] == 255)[0]
            if len(ys_actual_low) == 0 or len(ys_actual_high) > 2:
                model.wrong.append((x, int(y_predicted)))
        horizons.append([old_model, model])
    return horizons, edges.shape[1] // 2, colored


exit = False
i = 195
j = 100
window_name = "Original vs Edge Detected"
cv2.namedWindow(window_name)

while not exit:
    file = example_files[i]
    frame  = cv2.imread(file)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    models, x_separation, colored = detect_horizons(frame)
    
    #assert len(models) == 2 if x_separation < frame.shape[1] else len(models) == 1

    for _models in models:
        #    model = _models[1]
        for model in _models:
            for x0, y0 in model.wrong:
                cv2.circle(colored, (x0, y0), 2, model.color, -1)

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