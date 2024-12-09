import cv2
import math
# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
# RUCO_MIP_36h12

# aruco_dict_name = "DICT_4X4_100"
image = cv2.imread('test-aruco.png')

for aruco_dict_name in ARUCO_DICT.keys():
  aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dict_name])



  arucoParams = cv2.aruco.DetectorParameters_create()
  (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict,
    parameters=arucoParams)
  if (ids):
    my_corners = corners
    print(ids)

# print(image.size)
print(my_corners)
center = [0, 0]
for corner_coordinates in my_corners[0][0]:
  x = corner_coordinates[0]
  y = corner_coordinates[1]

  center[0] += x
  center[1] += y 

center[0] = center[0]//4
center[1] = center[1]//4

my_corners = my_corners[0][0]
r = ((center[0] - my_corners[0][0])*2 + (center[1] - my_corners[0][1])2)*0.5
image = cv2.circle(image,(int(center[0]), int(center[1])), radius=math.ceil(r), color=(0, 0, 255), thickness=5)


cv2.imshow('image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
