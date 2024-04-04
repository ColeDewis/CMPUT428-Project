import cv2
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    img = cv2.imread("ruler_img.jpg")
    fig = plt.figure()
    plt.imshow(img)
    plt.title("define the 4 points for the plane")
    pts = plt.ginput(4) 
    p1, p2, p3, p4 = pts
    
    p1 = np.array([p1[0], p1[1], 1])
    p2 = np.array([p2[0], p2[1], 1])
    p3 = np.array([p3[0], p3[1], 1])
    p4 = np.array([p4[0], p4[1], 1])
    
    # calculate vanishing point 1
    l1 = np.cross(p2, p3)
    l2 = np.cross(p1, p4)
    v1 = np.cross(l1, l2) 
    v1 = v1 / v1[2]
    
    # calculate vanishing point 2
    l3 = np.cross(p1, p2)
    l4 = np.cross(p3, p4)
    v2 = np.cross(l3, l4)
    v2 = v2 / v2[2]
    
    # infinite line
    vl = np.cross(v1, v2)
    
    plt.title("select the 2 arbitrary pts parallel to line s2s3")
    pts = plt.ginput(2)
    a1, a2 = pts
    a1 = np.array([a1[0], a1[1], 1])
    a2 = np.array([a2[0], a2[1], 1])
    
    search_line = np.cross(a1, v1)
    v3_finder = np.cross(p2, a1)
    v3 = np.cross(v3_finder, vl)
    pt_finder = np.cross(p3, v3)
    
    p_t = np.cross(search_line, pt_finder)
    p_t = p_t / p_t[2]
    
    # known dist is a1-p_t
    # 4 points: a1, a1+epsilon, p_t, a2, v1; these are s3, a11, s2, a12, v1 respectively.
    epsilon = 0.01
    
    # determine cr1
    d_a1_pt = np.linalg.norm(a1[:2] - p_t[:2])
    d_aeps_v1 = np.linalg.norm(a1[:2] - v1[:2]) + epsilon # increase distance by epsilon to avoid zeros
    d_pt_aeps = np.linalg.norm(a1[:2] - p_t[:2]) + epsilon
    d_a1_v1 = np.linalg.norm(a1[:2] - v1[:2])
    cr1 = (d_a1_pt * d_aeps_v1) / (d_pt_aeps * d_a1_v1)

    
    # determine cr2
    d_aeps_s2 = np.linalg.norm(a1[:2] - p_t[:2]) + epsilon
    d_a2_v1 = np.linalg.norm(a2[:2] - v1[:2])
    d_aeps_a2 = np.linalg.norm(a1[:2] - a2[:2]) + epsilon
    d_pt_v1 = np.linalg.norm(p_t[:2] - v1[:2])
    cr2 = (d_aeps_s2 * d_a2_v1) / (d_aeps_a2 * d_pt_v1)
    
    
    D = 2
    distance = (D / cr1) / cr2
    print(f"Distance between your two points is: {distance}")
    
    d_star = 2
    D_star = (d_star * d_aeps_s2) / ((D / cr1) * d_pt_v1)
    p = (d_a1_v1 * D_star) / (D_star + 1)
    
    # get the unit vector between the point "a" and point "v"
    unit_vec = [(v1[0] - a1[0])/d_a1_v1, (v1[1] - a1[1])/d_a1_v1]
    print("P:", p)
    print("Unit vec:", unit_vec)
    p_img = [a1[0] + unit_vec[0] * p, a1[1] + unit_vec[1] * p]
    print("P_img", p_img)
    
    plt.close()
    img = cv2.line(img, (int(p2[0]), int(p2[1])), (int(v1[0]), int(v1[1])), color=(0, 0, 255), thickness=5)
    img = cv2.line(img, (int(p1[0]), int(p1[1])), (int(v1[0]), int(v1[1])), color=(0, 0, 255), thickness=5)
    img = cv2.line(img, (int(a1[0]), int(a1[1])), (int(v1[0]), int(v1[1])), color=(255, 0, 0), thickness=5)
    img = cv2.circle(img, (int(p_img[0]), int(p_img[1])), 0, color=(0, 255, 0), thickness=10)
    cv2.imshow("your point:", img)
    cv2.waitKey()
    
    
    exit()
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
    