from os import chdir
from pathlib import Path
import cv2
import argparse

VID = r'C:\Users\reck-\PycharmProjects\cam-calib\res\calibration.mp4'
OUTPATH = "/img"

parser = argparse.ArgumentParser(description='Extract frames from a video and save them to disk.')
parser.add_argument('filename', nargs='?', type=str, help='Path to the videofile to be processed')
parser.add_argument('--outpath', type=str, default="/img", help='Folder to store the frames in')
args = parser.parse_args()

vidcap = cv2.VideoCapture(VID)
success, image = vidcap.read()

path = Path(args.outpath)
path.mkdir(parents=True, exist_ok=True)
chdir(args.outpath)
count = 0

while success:
    cv2.imwrite(f"frame{count}.jpg", image)  # save frame as JPEG file
    success, image = vidcap.read()
    count += 1
