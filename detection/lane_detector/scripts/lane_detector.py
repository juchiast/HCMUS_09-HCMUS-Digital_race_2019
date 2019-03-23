import numpy as np
import cv2 as cv
import sys
import matplotlib.pyplot as plt
import random
import numpy.polynomial.polynomial as poly
from sklearn.cluster.dbscan_ import DBSCAN

skyLine = 85
heightLayer = 10
BIRDVIEW_WIDTH = 240
BIRDVIEW_HEIGHT = 320

plot_raw = 131
plot_binary = 132
plot_lane = 133

left_lane = np.array([])
right_lane = np.array([])

def scan(points: np.array, visualization: np.ndarray):
    dbscan = DBSCAN(eps=heightLayer, min_samples=2)
    db = dbscan.fit(points)

    # clustering
    clusters = get_clusters(points, db.labels_)

    for cluster in clusters:
        color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))
        for point in cluster:
            cv.circle(visualization, (point[1], point[0]), 2, color, -1)

    max_1_index = -1
    max_1_len = 0
    for i in range(len(clusters)):
        if len(clusters[i]) > max_1_len:
            max_1_len = len(clusters[i])
            max_1_index = i

    max_2_index = -1
    max_2_len = 0
    for i in range(len(clusters)):
        if i != max_1_index and max_1_len >= len(clusters[i]) > max_2_len:
            max_2_len = len(clusters[i])
            max_2_index = i


    cluster_max1 = clusters[max_1_index]
    cluster_max2 = clusters[max_2_index]

    # print(cluster_max1)
    # print(cluster_max2)


    cluster_max1_org = np.mean(cluster_max1, axis=0).astype(np.int64)
    cluster_max2_org = np.mean(cluster_max2, axis=0).astype(np.int64)

    # print(cluster_max1_org, cluster_max2_org)

    if cluster_max1_org[1] < cluster_max2_org[1]:
        left = clusters[0]
        right = clusters[1]
        left_org = cluster_max1_org
        right_org = cluster_max2_org
    elif cluster_max2_org[1] < cluster_max1_org[1]:
        left = clusters[1]
        right = clusters[0]
        left_org = cluster_max2_org
        right_org = cluster_max1_org
    else:
        print("!!!!!!!")
        return
    
    color_left = (0,0,255) # BGR
    color_right = (255,0,0)

    cv.circle(visualization, (left_org[1], left_org[0]), 4, color_left, -1)
    cv.circle(visualization, (right_org[1], right_org[0]), 4, color_right, -1)


    # for point in left:
    #     cv.circle(visualization, (point[1], point[0]), 2, color_left, -1)
    
    # for point in right:
    #     cv.circle(visualization, (point[1], point[0]), 2, color_right, -1)

    left_x = np.array([point[1] for point in left])
    left_y = np.array([point[0] for point in left])

    left_coeff = poly.polyfit(left_x, left_y, 3)
    print(left_coeff)

    new_rows = np.linspace(0, visualization.shape[0], 10)
    new_cols = poly.polyval(new_rows, left_coeff)

    plt.subplot(plot_lane)
    plt.plot(new_rows, new_cols)
    # plt.imshow(cv.cvtColor(visualization, cv.COLOR_BGR2RGB))


def get_clusters(points, labels):
    clusters = []
    for label in set(labels):
        if label != -1:
            cluster = np.array([points[i] for i in range(len(points)) if labels[i] == label])
            clusters.append(cluster)
    return np.array(clusters)

def fillLane(src: np.ndarray):
    lines = cv.HoughLinesP(src, 1, np.pi / 180, 1)
    for line in lines:
        x_start,y_start,x_end,y_end = line[0]
        cv.line(src, (x_start, y_start), (x_end, y_end), 255, 3, cv.LINE_AA)

def birdViewTranform(src: np.ndarray):

    print(src.shape)
    width = src.shape[1]
    height = src.shape[0]

    src_vertices = np.float32([
        [0, skyLine],
        [width, skyLine],
        [width, height],
        [0, height]
    ])

    dst_vertices = np.float32([
        [0, 0],
        [BIRDVIEW_WIDTH, 0],
        [BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT],
        [105, BIRDVIEW_HEIGHT]
    ])

    M = cv.getPerspectiveTransform(src_vertices, dst_vertices)
    dst = cv.warpPerspective(src, M, (BIRDVIEW_WIDTH, BIRDVIEW_HEIGHT))

    return dst


def splitLayer(binary: np.ndarray):
    res = []
    i = 0
    while i <= binary.shape[0] - heightLayer:
        res.append(binary[i: heightLayer*(i+1), 0:binary.shape[1]])
        i += heightLayer

    print(len(res))

    return res


def centerRoadSide(binary: np.ndarray, layers: list):
    contourImage = np.ndarray(binary.shape, np.uint8)

    res = []
    for i in range(len(layers)):
        cnts, _ = cv.findContours(layers[i], cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if (len(cnts) == 0):
            continue
            
        layer_start_row = i * heightLayer
        layer_end_row = (i+1) * heightLayer
        layer_start_col = 0
        layer_end_col = binary.shape[1]

        roi_layer = contourImage[layer_start_row:layer_end_row,layer_start_col:layer_end_col]
        # cv.imshow("ROI", roi_layer)


        cv.drawContours(roi_layer, cnts, -1, (255,), cv.FILLED)         
        # cv.imshow("Contours", contourImage)

        for j in range(len(cnts)):
            # area = cv.contourArea(cnts[j], False)
            M1 = cv.moments(cnts[j], False)

            if M1["m00"] != 0:
                cx = int(M1["m10"] / M1["m00"])
                cy = int(M1["m01"] / M1["m00"])
                cv.circle(roi_layer, (cx, cy), 2, (127), -1)

                cy = cy + i * heightLayer

                if cx > 0 and cy > 0:
                    res.append(np.array([cy, cx]))

    return np.array(res)

def preprocessing(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    binary = cv.inRange(hsv, (0, 0, 180), (179, 65, 255))

    binary = birdViewTranform(binary)
    fillLane(binary)

    layers = splitLayer(binary)
    centroidsPoints = centerRoadSide(binary, layers)

    # visualize
    visualization = cv.cvtColor(binary, cv.COLOR_GRAY2BGR)

    for point in centroidsPoints:
        cv.circle(visualization, (point[1], point[0]), 2, (127, 127, 127), -1)
    plt.subplot(plot_binary)
    plt.imshow(visualization, cmap='gray')


    scan(centroidsPoints, visualization)

    plt.imshow(cv.cvtColor(visualization, cv.COLOR_BGR2RGB))
    plt.show()

    return binary

def main(argv):
    filename = argv[1]
    print("File " + filename)

    image = cv.imread(filename)
    plt.subplot(plot_raw)
    plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))

    image = preprocessing(image)

    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
