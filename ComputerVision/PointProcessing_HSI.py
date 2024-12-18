import numpy as np
import cv2

"""
A BGR-image to HSI-image converter.
Input BGR-images must consist of floating point elements in range [0;1]. 
Using floating point elements improves the precision of done point processing. 
The output matrix also consists of floating point elements.
"""
def BGRtoHSI(img):
    assert len(img.shape) == 3, """
                Use a BGR-image as input 
                in range [0;1]
                """
    assert type(img[0,0,0]) == np.float_, """
                Use floating point elements 
                in range [0;1]
                """
    assert np.max(img) <= 1, """
                Please convert the values into range [0;1] 
                by e.g. dividing 8-bit pixels by 255
                """

    result = np.empty(img.shape, np.float64)

    for x, rows in enumerate(img):
        for y, pixel in enumerate(rows):
            R = pixel[2] * 255
            G = pixel[1] * 255
            B = pixel[0] * 255

            V = sum([B, G, R]) / 3

            S = 0
            if sum([B, G, R]) != 0:
                S = 1 - 3 * min([B, G, R]) / sum([B, G, R])
            
            H = np.arccos(1/2 * ((R - G) + (R - B)) / 
                          np.sqrt((R - G)**2 + (R - B) * (G - B)))

            if np.isnan(H):
                H = 0
            else:
                H *= 180/np.pi
                if G < B:
                    H = 360 - H
            
            result[x, y] = [H/360, S, V/255]
    return result

"""
A HSI-image to BGR-image converter.
Works as the opposing operation to that of the BGRtoHSI(img) function.
Using the img_hsi=BGRtoHSI(img) function followed by img2=HSItoBGR(img_hsv) ...
provides the original BGR-image. 
"""
def HSItoBGR(img):
    assert len(img.shape) == 3, """
                Use a BGR-image as input 
                in range [0;1]
                """
    assert type(img[0,0,0]) == np.float_, """
                Use floating point elements 
                in range [0;1]
                """
    assert np.max(img) <= 1, """
                Please convert the values into range [0;1] 
                by e.g. dividing 8-bit pixels by 255
                """

    result = np.empty(img.shape, np.float64)

    for x, rows in enumerate(img):
        for y, pixel in enumerate(rows):
            I = pixel[2]
            S = pixel[1]
            H = pixel[0] * 360

            if H >= 0 and H <= 120:
                H_n = np.deg2rad(H)
            elif H > 120 and H <= 240:
                H_n = np.deg2rad(H - 120)
            elif H > 240 and H < 360:
                H_n = np.deg2rad(H - 240)

            val_a = I * (1 + (S * np.cos(H_n)) / np.cos(np.deg2rad(60) - H_n) )
            val_b = I - I * S

            if H >= 0 and H <= 120:
                R = val_a
                B = val_b
                G = 3 * I - R - B
            elif H > 120 and H <= 240:
                G = val_a
                R = val_b
                B = 3 * I - R - G
            elif H > 240 and H < 360:
                B = val_a
                G = val_b
                R = 3 * I - G - B
            
            result[x, y] = [B, G, R]
    return result

img = cv2.imread("figures/av_testset/av1.jpg") / 255
img = cv2.resize(img, (1000, 750))
img_hsi = BGRtoHSI(img)
img2 = HSItoBGR(img_hsi)

print(np.max(np.abs(img-img2)))

cv2.imshow("RGB", img)
cv2.imshow("HSI", img_hsi)
cv2.imshow("RGB2", img2)
cv2.imshow("Error", np.abs(img-img2))

cv2.waitKey(0)