import cv2
import numpy as np;

#Read image
im = cv2.imread("photo1.jpg")

out = im.copy()

hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([255, 255, 50]))
print(type(mask)) 


#res = cv2.bitwise_and(im, im, mask = mask)

cv2.imshow('res', mask)
k = cv2.waitKey(0)
cv2.destroyAllWindows()
