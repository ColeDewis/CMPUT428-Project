import cv2
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    img = cv2.imread("ruler_img.jpg")
    fig = plt.figure()
    plt.imshow(img)
    plt.title("define the 4 points for the plane")
    pts = plt.ginput(4) 
    s1, s2, s3, s4 = pts
    s1 = np.array([s1[0], s1[1], 1])
    s2 = np.array([s2[0], s2[1], 1])
    s3 = np.array([s3[0], s3[1], 1])
    s4 = np.array([s4[0], s4[1], 1])
    l1 = np.cross(s1, s4)
    l2 = np.cross(s2, s3)
    
    # now we have an image point of what is a vanishing point in reality.
    v1 = np.cross(l1, l2)
    v1 = v1 / v1[2]
    print(v1)
    
    
    plt.title("select the 2 arbitrary pts on line s2s3")
    pts = plt.ginput(2)
    a1, a2 = pts
    a1 = np.array([a1[0], a1[1], 1])
    a2 = np.array([a2[0], a2[1], 1])
    
    
    # determine cr1
    d_s2_s3 = np.linalg.norm(s2[:2] - s3[:2])
    d_a1_v1 = np.linalg.norm(a1[:2] - v1[:2])
    d_s2_a1 = np.linalg.norm(s2[:2] - a1[:2])
    d_s3_v1 = np.linalg.norm(s3[:2] - v1[:2])
    cr1 = (d_s2_s3 * d_a1_v1) / (d_s2_a1 * d_s3_v1)
    
    print("s2-s3", d_s2_s3)
    print("a1_v1", d_a1_v1)
    print("s3-a1", d_s2_a1)
    print("s2-v1", d_s3_v1)
    print("CR1", cr1)
    
    # determine cr2
    d_a1_s2 = np.linalg.norm(a1[:2] - s2[:2])
    d_a2_v1 = np.linalg.norm(a2[:2] - v1[:2])
    d_a1_a2 = np.linalg.norm(a1[:2] - a2[:2])
    d_s2_v1 = np.linalg.norm(s2[:2] - v1[:2])
    
    cr2 = (d_a1_s2 * d_a2_v1) / (d_a1_a2 * d_s2_v1)
    print("CR2", cr2)
    
    D = 2
    
    # dist = known distane / cr1 / cr2
    distance = (D / cr1) / cr2
    
    # Seems to be fairly accurate (as much as clicking on image points can be really)
    print(f"Distance between your two points is: {distance}")
    
    d_star = 2
    D_star = (d_star * d_s2_s3) / ((D / cr1) * d_s3_v1)
    
    # for our purposes, it seems convenient to define a = 0, v = d_a_v
    p = (d_a2_v1 * D_star) / (D_star + 1)
    
    # get the unit vector between the point "a" and point "v"
    unit_vec = [(v1[0] - a2[0])/d_a2_v1, (v1[1] - a2[1])/d_a2_v1]
    print("unit vec test: ", unit_vec, np.linalg.norm(unit_vec))
    print("P:", p)
    p_img = [a2[0] + unit_vec[0] * p, a2[1] + unit_vec[1] * p]
    print("P_img", p_img)
    
    plt.close()
    img = cv2.line(img, (int(a2[0]), int(a2[1])), (int(v1[0]), int(v1[1])), color=(255, 0, 0), thickness=5)
    img = cv2.circle(img, (int(p_img[0]), int(p_img[1])), 0, color=(0, 255, 0), thickness=10)
    cv2.imshow("your point:", img)
    cv2.waitKey()
    