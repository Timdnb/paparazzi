import numpy as np
import cv2
import os


# Load the saved array from the file
uvs = np.load("./saved_numpy.npy")

# print(uvs.shape)


# Input and output folder paths
input_folder = './cyberzoo_poles_panels_mats/20190121-142935'


# List all files in the folder
files = os.listdir(input_folder)

# Sort the files to ensure consistent order (optional)
files.sort()

# Specify the index of the photo you want to open
photo_index = 120

# Construct the full path of the image using the specified index
image_path = os.path.join(input_folder, files[photo_index])

image = cv2.imread(image_path)

# Iterate over coordinates from uvs and draw circles on the image

for uv in uvs[photo_index]:
    print(uv)
    y, x = 240- int(uv[0]), int(uv[1])  # Assuming uvs contains (x, y) coordinates
    # y, x = int(220), int(400)
    cv2.circle(image, (y, x), 5, (0, 0, 255), -1)  # Draw a circle with radius 5 and red color


# Display the image with circles
cv2.imshow('Image with Circles', image)
cv2.waitKey(0)
cv2.destroyAllWindows()