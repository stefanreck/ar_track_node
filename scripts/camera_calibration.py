import argparse
import cv2
import glob
import numpy as np
from os import chdir
from ruamel.yaml import YAML
from pprint import pprint

yaml = YAML()
yaml.default_flow_style = True

parser = argparse.ArgumentParser(description='Extract frames from a video and save them to disk.')
parser.add_argument('filepath', nargs='?', type=str,
                    help='Path to the folder containing the .jpg files to be processed')
parser.add_argument('--distortion-model', type=str, default="plumb_bob", help='Distortion model to be applied')
parser.add_argument('--camera-name', type=str, default="camera", help='Camera name to be used in YAML')
parser.add_argument('--width', type=int, default=1280, help='Width of the video in pixels')
parser.add_argument('--height', type=int, default=720, help='Height of the video in pixels')
parser.add_argument('--chessboard-rows', type=int, default=9, help='Number of the chessboards rows')
parser.add_argument('--chessboard-columns', type=int, default=6, help='Number of the chessboards columns')
parser.add_argument('--outfile', type=str, default="calibration.yaml", help='name of the yaml file to be written')
args = parser.parse_args()

ROWS = args.chessboard_rows
COLUMNS = args.chessboard_columns

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((COLUMNS * ROWS, 3), np.float32)
objp[:, :2] = np.mgrid[0:ROWS, 0:COLUMNS].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

chdir(args.filepath)
images = glob.glob('*.jpg')
am = len(images)
count = 0

for fname in images:
    count += 1
    print(f"{count}/{am}: {fname}")
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (ROWS, COLUMNS), None)

    # If found, add object points, image points (after refining them)
    if ret:
        print(">> found corners")
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (ROWS, COLUMNS), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(200)

print("Performing camera calibration..")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

c = 0
proj = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
R = None
P = None

# Build the projection matrix
print("Calculating projection matrix..")
for r in rvecs:
    rotation_mat = np.zeros(shape=(3, 3))
    R = cv2.Rodrigues(r, rotation_mat)[0]
    P = np.column_stack((np.matmul(mtx, R), tvecs[c]))
    p_list = np.asarray(P).tolist()
    ind1 = 0
    ind2 = 0
    for ll in p_list:
        for pp in ll:
            proj[ind1][ind2] += pp
            ind2 = (ind2 + 1) % 4
        ind1 = (ind1 + 1) % 3
    c += 1

proj = np.true_divide(proj, len(rvecs))
pprint(proj)

# transform the matrix and distortion coefficients to writable lists
data = {'image_width': args.width,
        'image_height': args.height,
        'camera_name': args.camera_name,
        'camera_matrix':
            {'rows': 3,
             'cols': 3,
             'data': np.asarray(mtx).tolist()
             },
        'distortion_model': args.distortion_model,
        'distortion_coefficients':
            {'rows': 1,
             'cols': 5,
             'data': np.asarray(dist).tolist()
             },
        'rectification_matrix':
            {'rows': 3,
             'cols': 3,
             'data': np.asarray(R).tolist()
             },
        'projection_matrix':
            {'rows': 3,
             'cols': 4,
             'data': np.asarray(proj).tolist()
             },
        }

# Write to yaml file
with open(args.outfile, "w") as f:
    yaml.dump(data, f)
print(f"Result written to {args.outfile}")
cv2.destroyAllWindows()
