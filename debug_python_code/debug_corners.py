import numpy as np
import cv2 as cv
from finding_uvs_corners import rel_3D_to_2D

# Create a 240x520 black image
image = np.zeros((520, 240, 3), dtype=np.uint8)

# Starting point
point = np.array([[7, -1, 2, 1]])

# Function to display the image
def display_image():
    img = image.copy()
    print(point)
    uv = rel_3D_to_2D(point)[0]
    print(point, uv)
    cv.circle(img, (uv[0], uv[1]), 10, (0, 255, 0), -1)  # Draw a green circle at the current point
    cv.imshow('Image', img)
    cv.waitKey(1)

# Display the initial image
display_image()

# Event loop to handle key presses
while True:
    key = cv.waitKey(0)
    print(chr(key))
    if key == ord('a'):
        point[0][1] -= 0.5
    elif key == ord('d'):
        point[0][1] += 0.5
    elif key == ord('s'):
        point[0][0] -= 0.5
    elif key == ord('w'):
        point[0][0] += 0.5
    elif key == ord("q"):
        break

    # Display updated image
    display_image()

cv.destroyAllWindows()

