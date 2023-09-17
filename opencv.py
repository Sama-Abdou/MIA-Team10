
import cv2
import numpy as np

def detect_balls(img):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range of colors for the red and blue balls
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Apply a Gaussian blur to the red and blue masks
    blur_red = cv2.GaussianBlur(mask_red, (3, 3), 0)
    blur_blue = cv2.GaussianBlur(mask_blue, (5, 5), 0)

    # Use the HoughCircles function to detect circles in the image
    circles_red = cv2.HoughCircles(blur_red, cv2.HOUGH_GRADIENT_ALT, 1, 25, param1=300, param2=0.68, minRadius=0, maxRadius=0)
    circles_blue = cv2.HoughCircles(blur_blue, cv2.HOUGH_GRADIENT_ALT, 1, 30, param1=300, param2=.5, minRadius=0, maxRadius=0)

    # Draw a circle or bounding box around the detected circles
    if circles_red is not None:
        circles_red = np.round(circles_red[0, :]).astype("int")
        for (x, y, r) in circles_red:
            cv2.circle(img, (x, y), r, (0, 0, 255), 2)

    if circles_blue is not None:
        circles_blue = np.round(circles_blue[0, :]).astype("int")
        for (x, y, r) in circles_blue:
            cv2.circle(img, (x, y), r, (255, 0, 0), 2)
            
    # Save the image to a file
    cv2.imwrite(f"C:\\Users\\Lenovo\\Documents\\detected_balls\\detected_balls_{i}.jpg", img)
            
    return img


# Load and process 20 images
for i in range(1, 21):
    image = cv2.imread(f"C:\\Users\\Lenovo\\Downloads\\task8.1_dataset\\image{i}.jpg")
    
    while True:
        cv2.imshow(f"Detected Balls - Image {i}", image)
        key = cv2.waitKey(0)
        
        if key == 13:  # Enter key
            result = detect_balls(image)
            cv2.imshow(f"Detected Balls - Image {i}", result)
            break
    
cv2.destroyAllWindows()