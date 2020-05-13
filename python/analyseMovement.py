import argparse
from collections import namedtuple
import csv
from datetime import datetime
import glob
import os.path


from ImageAnalysisFuncs.target_detection_otsu import targetCoordinates 
from vfr.conf import POS_REP_TARGET_DETECTION_OTSU_PARS


IMAGE_EXTENTSION = ".bmp"

TIME_FORMAT = '%Y-%m-%d_%H-%M-%S-%f'

FIBRE_RATIO = 2 # ratio of the size of the small target > large target vector and small target > large target

movementData = namedtuple("movementData", #name of tuple
                        " x" # needed data
                        " y"  # needed data
                        " time" # needed data
                        " file" # debugging data
                        " sx" # debugging data
                        " sy" # debugging data
                        " lx" # debugging data
                        " ly") # debugging data

def analyseMovent(image_folder):
    """ Read all images in the image_folder, analysis the images for blob pairs and then determine the fibre center. returns a list  of named tuples containing x postion, y position, time, image name.
    """
    
    dataList = []
    glob_path = os.path.join(image_folder,"*{}".format(IMAGE_EXTENTSION) )
    for image in glob.glob(glob_path):
    
        sx, sy, _, lx, ly, _ = targetCoordinates(image, POS_REP_TARGET_DETECTION_OTSU_PARS)
        
        dx = lx - sx
        dy = ly - sy
        
        fx = dx * FIBRE_RATIO
        fy = dy * FIBRE_RATIO
        
        _,filename = os.path.split(image)
        
        timestamp = datetime.strptime(filename, TIME_FORMAT )
        
        # First image has time of zero
        if not dataList:
            time = timestamp - timestamp
            timeStart = timestamp
        else:
            time = timestamp - timeStart
        
        dataList.append(movementData(fx,fy,time,image,sx,sy,lx,ly))

def saveToFile(dataList, filename):
    # write data structure to a file.
    with open(filename, 'w') as openfile:
        csvwriter = csv.writer(openfile, fieldnames=movementData._fields)
        csvwriter.writeheader()
        csvwriter.writerows(dataList)

def get_parser():
    parser = argparse.ArgumentParser(description='Analyse a series of images denoting the movement of a single FPU')
    parser.add_argument('-i', '--image-folder',
                       help='folder containing timestamped images from a movement')
                       
    return parser

if __name__ == "__main__":
    parser = get_parser()
    parser.parse_args()
    print("Analysing movement")