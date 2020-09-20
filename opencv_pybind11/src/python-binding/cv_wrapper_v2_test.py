# https://github.com/edmBernard/pybind11_opencv_numpy

import numpy as np
import copy
import cv_wrapper_v2

import cv2
import matplotlib.pyplot as plt


def simple_tests(imgfile):
    # Read from c++
    a = cv_wrapper_v2.read_image(imgfile)
    print('init a: 0x%x' % id(a))
    cv_wrapper_v2.show_image(a)  # work

    # Check continuous problem from old version
    b = a[:, :, 0]
    cv_wrapper_v2.show_image(b)  # work no more continous problem
    print('diff b: 0x%x' % id(b))

    c = copy.deepcopy(b)
    cv_wrapper_v2.show_image(c)  # still works
    print('diff c: 0x%x' % id(c))


    # Proves that it's still the same thing
    d = cv_wrapper_v2.passthru(a)
    print('same d: 0x%x' % id(b))

    # Make a copy
    e = cv_wrapper_v2.clone(d)
    print('diff e: 0x%x' % id(e))


    # different allocator
    f = np.zeros(shape=(100, 100), dtype=np.uint8)
    print('\ninit e: 0x%x' % id(f))

    g = cv_wrapper_v2.passthru(f)
    print('same f: 0x%x' % id(g))


    # example of class
    my_class = cv_wrapper_v2.AddClass(1)
    h = my_class.add(f)
    print(f[0, 0])  # expected 0
    print(h[0, 0])  # expected 1


def holefilling_test(imgfile, radius, num_iters):
    depth = cv2.imread(imgfile, cv2.IMREAD_UNCHANGED)
    depth = depth.astype(np.uint16)
    depth_orig = copy.deepcopy(depth)
    for i in range(num_iters):
        filled_cnt = cv_wrapper_v2.fill_hole(depth, radius)
        print("filled %d holes at iter #%d"%(filled_cnt, i+1))
    plt.subplot(1, 2, 1)
    plt.axis('off')
    plt.imshow(depth_orig, cmap=plt.gray())
    plt.subplot(1, 2, 2)
    plt.axis('off')
    plt.imshow(depth, cmap=plt.gray())
    plt.show()
