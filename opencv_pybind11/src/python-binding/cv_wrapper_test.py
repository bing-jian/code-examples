import cv2
import matplotlib.pyplot as plt
import numpy as np

import cv_wrapper

# https://www.jianshu.com/p/be16847b0b74
def main():
    image_rgb = cv2.imread('lena_color.jpg', cv2.IMREAD_UNCHANGED)
    image_gray = cv2.imread('lena_gray.jpg', cv2.IMREAD_UNCHANGED)

    var1 = cv_wrapper.test_rgb_to_gray(image_rgb)
    print(var1.shape)
    plt.figure('rgb-gray')
    plt.imshow(var1, cmap=plt.gray())

    var2 = cv_wrapper.test_gray_canny(image_gray)
    plt.figure('canny')
    plt.imshow(var2, cmap=plt.gray())

    var3 = cv_wrapper.test_pyramid_image(image_gray)
    var3 = var3[1:]
    plt.figure('pyramid_demo')
    for i, image in enumerate(var3, 1):
        plt.subplot(2, 2, i)
        plt.axis('off')
        plt.imshow(image, cmap=plt.gray())

    plt.show()


if __name__ == '__main__':
    main()
