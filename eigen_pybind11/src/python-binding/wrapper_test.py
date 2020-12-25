import matplotlib.pyplot as plt
import numpy as np
import wrapper


def sample_spherical(npoints, ndim=3):
    vec = np.random.randn(npoints, ndim)
    vec /= np.linalg.norm(vec, axis=1)[:, np.newaxis]
    vec[:, 0] *= 3.0
    vec[:, 1] *= 2.0
    return vec


def dt3d_tests():
    pts = sample_spherical(10000)
    x_min = pts[:, 0].min() - 1
    y_min = pts[:, 1].min() - 1
    z_min = pts[:, 2].min() - 1

    z_max = pts[:, 2].max() + 1
    y_max = pts[:, 1].max() + 1
    x_max = pts[:, 0].max() + 1

    o = wrapper.distance_transform_3d(pts, x_min, y_min, z_min, x_max, y_max,
                                      z_max, 0.1, 50)
    z_dim, y_dim, x_dim = o.shape
    z_slice = o[z_dim // 2, :, :]
    y_slice = o[:, y_dim // 2, :]
    x_slice = o[:, :, x_dim // 2]

    fig = plt.figure()
    fig.suptitle("Distance field represented as numpy array of shape {}".format(
        o.shape))
    ax1 = fig.add_subplot(1, 3, 1)
    ax1.title.set_text('Z = %d' % (z_dim // 2))
    ax1.imshow(z_slice, cmap=plt.gray())

    ax2 = fig.add_subplot(1, 3, 2)
    ax2.title.set_text('Y = %d' % (y_dim // 2))
    ax2.imshow(y_slice, cmap=plt.gray())

    ax3 = fig.add_subplot(1, 3, 3)
    ax3.title.set_text('X = %d' % (x_dim // 2))
    ax3.imshow(x_slice, cmap=plt.gray())

    plt.show()


dt3d_tests()
