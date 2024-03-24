import numpy as np
import cv2
import os
import math

# Load the saved array from the file
uvs = np.load("./saved_numpy.npy")[104:]

# print(uvs.shape)


# Input and output folder paths
# input_folder = './cyberzoo_poles_panels_mats/20190121-142935'
input_folder = './sim_poles_panels_mats/20190121-161931'


# List all files in the folder
files = os.listdir(input_folder)

# Sort the files to ensure consistent order (optional)
files.sort()

output_folder = './images_with_corners/'
os.makedirs(output_folder, exist_ok=True)

FPS = 10.3
# Iterate through all indices
for photo_index in range(min(int(uvs.shape[0] / FPS), len(files))):
    # Construct the full path of the image using the current index
    image_path = os.path.join(input_folder, files[photo_index])
    image = cv2.imread(image_path)

    # Iterate over coordinates from uvs and draw circles on the image
    for uv in uvs[math.ceil(photo_index * FPS)]:
        x, y = 240 - int(uv[0]), int(uv[1]) # Assuming uvs contains (x, y) coordinates
        #, y = 230,500
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)  # Draw a circle with radius 5 and red color

    # Save the modified image with circles to the output folder
    output_image_path = os.path.join(output_folder, f"image_{photo_index}.jpg")
    cv2.imwrite(output_image_path, image)


# # Display the image with circles
# cv2.imshow('Image with Circles', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()