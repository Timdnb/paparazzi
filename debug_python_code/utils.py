def remove_mess(image, window_size=10, thresh=.4):
    pixels_threshold = window_size**2 * thresh
    for y in range(0, image.shape[0] - window_size + 1, window_size):
        for x in range(0, image.shape[1] - window_size + 1, window_size):
            
            roi = image[y:y+window_size, x:x+window_size]
            
            # Count white pixels 
            white_pixels = np.sum(roi == [255, 255, 255])
            
            # If there are 20 or more white pixels, replace the entire ROI with black pixels
            if white_pixels >= pixels_threshold:
                image[y:y+window_size, x:x+window_size] = [0, 0, 0]
                # print("Removed patch")
    return image