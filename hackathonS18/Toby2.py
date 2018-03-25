
import cv2
import sys
import numpy as np
import os
import sys
import time
import tensorflow as tf
from imutils.video import FPS
from scripts import label_image
from datetime import datetime
from utils import label_map_util
from utils import visualization_utils_color as vis_util


#####################################################################


keep_processing = True
# camera_to_use = 0; # 0 if you have no built in webcam, 1 otherwise
# camera_to_use = cv2.CAP_XIAPI; # for the Xiema cameras (opencv built with driver)
camera_to_use = 1
distToTape = 200  # mm

PATH_TO_CKPT = './model/frozen_inference_graph_face.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = './protos/face_label_map.pbtxt'

NUM_CLASSES = 2

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
Q = np.genfromtxt('Q.csv', delimiter=',')
mapL1 = np.genfromtxt('mapL1.csv', delimiter=',')
mapL2 = np.genfromtxt('mapL2.csv', delimiter=',')
mapR1 = np.genfromtxt('mapR1.csv', delimiter=',')
mapR2 = np.genfromtxt('mapR2.csv', delimiter=',')
mapL1, mapL2 = cv2.convertMaps(mapL1.astype(np.float32), mapL2.astype(np.float32), cv2.CV_16SC2)
mapR1, mapR2 = cv2.convertMaps(mapR1.astype(np.float32), mapR2.astype(np.float32), cv2.CV_16SC2)

#####################################################################

# STAGE 1 - open 2 connected cameras
camL = cv2.VideoCapture(1)
camR = cv2.VideoCapture(2)

fps=FPS().start()
class TensoflowFaceDector(object):
    def __init__(self, PATH_TO_CKPT):
        """Tensorflow detector
        """
        #print "where am i?"
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        with self.detection_graph.as_default():
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            with tf.Session(graph=self.detection_graph, config=config) as self.sess:
                
                self.windowNotSet = True

    def run(self, image):
        """image: bgr image
        return (boxes, scores, classes, num_detections)
        """

        image_np = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        # Actual detection.
        start_time = time.time()
        (boxes, scores, classes, num_detections) = self.sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})
        elapsed_time = time.time() - start_time

        return (boxes, scores, classes, num_detections)




def classify(name):

    safe = False
    impath = 'tf_files/' + name
    glass_no_glass_graph = 'tf_files/trains/glasses_no_glasses/retrained_graph.pb'
    glass_no_glass_labels = 'tf_files/trains/glasses_no_glasses/retrained_labels.txt'
    labels, results = label_image.label(impath, glass_no_glass_graph, glass_no_glass_labels)
    if results[1] > .7 or results[0] < .3:
        print("WARNING: YOU ARE NOT WEARING SAFETY GLASSES")
        os.rename(impath, os.path.join('tf_files/classified_no_glasses', name))
    else:
        safe_v_reg_graph = 'tf_files/trains/safety_vs_regular_glasses/retrained_graph.pb'
        safe_v_reg_labels = 'tf_files/trains/safety_vs_regular_glasses/retrained_labels.txt'
        labels, results = label_image.label(impath, safe_v_reg_graph, safe_v_reg_labels)
        if results[0] > .7 or results[1] < .3:
            print("WARNING: THOSE GLASSES WILL NOT PROTECT YOU")
            os.rename(impath, os.path.join('tf_files/classified_regular_glasses', name))
        elif results[1] > .7:
            print("Safety Glasses On: YOU ARE SAFE")
            os.rename(impath, os.path.join('tf_files/classified_safety_glasses', name))
            safe = True
        else:
            print("WARNING: YOU ARE NOT WEARING SAFETY GLASSES")
    return safe


def crop(image, boxes, i):

    [h, w] = image.shape[:2]
    ymin, ymax = (int)(boxes[0][i][0] * h), (int)(boxes[0][i][2] * h)
    ymin, ymax = min(ymin, ymax), max(ymin, ymax)
    xmin, xmax = (int)(boxes[0][i][1] * w), (int)(boxes[0][i][3] * w)
    xmin, xmax = min(xmin, xmax), max(xmin, xmax)
    return image[ymin:ymax, xmin:xmax]


def checkForFace(cameraID):

    cap = cameraID
    looking = True
    while looking:
        ret, image = cap.read()
        if ret == 0:
            break

        [h, w] = image.shape[:2]
        image = cv2.flip(image, 1)

        (boxes, scores, classes, num_detections) = tDetector.run(image)

        for i, score in enumerate(scores[0]):
            if score > .7:
                def mean(x1, x2): return (int)((x1 + x2) / 2)
                ymin, ymax = (int)(boxes[0][i][0] * h), (int)(boxes[0][i][2] * h)
                ymin, ymax = min(ymin, ymax), max(ymin, ymax)
                xmin, xmax = (int)(boxes[0][i][1] * w), (int)(boxes[0][i][3] * w)
                xmin, xmax = min(xmin, xmax), max(xmin, xmax)
                x, y = mean(xmin, xmax), mean(ymin, ymax)
                #print("success")
                #print(x, y)

                return x, y, image, boxes, scores


def classifyImage(image, boxes, scores):
    for i, score in enumerate(scores[0]):
        if score > .7:
            face = crop(image, boxes, i)
            [h, w] = face.shape[:2]
            if h > 40 and w > 40:
                name = "{}.jpg".format(unicode(datetime.now()))
                cv2.imwrite('tf_files/' + name, face)

                safe = classify(name)
                return safe

tDetector = TensoflowFaceDector(PATH_TO_CKPT)

# define video capture object
while True:
    a=time.time()
    x, y, img, boxes, scores = checkForFace(camL)
    b=time.time() 
    windowNameL = "LEFT Camera Input"  # window name
    windowNameR = "RIGHT Camera Input"  # window name

    print()
    print("-> calc. disparity image")

    max_disparity = 128
    window_size = 3
    #stereoProcessor = cv2.StereoSGBM_create(0, max_disparity, 21);
    stereoProcessor = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=160,  # max_disp has to be dividable by 16
        blockSize=5,
        P1=8 * 3 * window_size ** 2,
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    keep_processing = True

    windowNameD = "SGBM Stereo Disparity - Output"  # window name
   
    # while (keep_processing):

    # grab frames from camera (to ensure best time sync.)

    camL.grab()
    camR.grab()

    # then retrieve the images in slow(er) time
    # (do not be tempted to use read() !)

    ret, frameL = camL.retrieve()
    ret, frameR = camR.retrieve()

    # remember to convert to grayscale (as the disparity matching works on grayscale)
    grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

    # undistort and rectify based on the mappings (could improve interpolation and image border settings here)
    # N.B. mapping works independant of number of image channels

    undistorted_rectifiedL = cv2.remap(grayL, mapL1, mapL2, cv2.INTER_LINEAR)
    undistorted_rectifiedR = cv2.remap(grayR, mapR1, mapR2, cv2.INTER_LINEAR)

    # compute disparity image from undistorted and rectified versions
    # (which for reasons best known to the OpenCV developers is returned scaled by 16)

    disparity = stereoProcessor.compute(undistorted_rectifiedL, undistorted_rectifiedR)
    cv2.filterSpeckles(disparity, 0, 4000, 128)
    dImage = cv2.reprojectImageTo3D(disparity, Q)
    # scale the disparity to 8-bit for viewing

    height, width = disparity.shape[:2]
    # print(dImage[width/2,height/2])
    print(dImage[y, x])
    disparity_scaled = (disparity / 16.).astype(np.uint8) + abs(disparity.min())

    # display image

    # cv2.imshow(windowNameL,undistorted_rectifiedL);
    # cv2.imshow(windowNameR,undistorted_rectifiedR);

    # display disparity

    #cv2.imshow(windowNameD, disparity_scaled)
    #cv2.imshow('3dImage', dImage)
    # start the event loop - essential

    key = cv2.waitKey(40) & 0xFF  # wait 40ms (i.e. 1000ms / 25 fps = 40 ms)
    [h, w] = frameL.shape[:2]
    image = cv2.flip(frameL, 1)
    c= time.time()
    #print(dImage[y,x][2] < distToTape)
    if ((dImage[y,x][2] < distToTape) and (dImage[y,x][2]>0)):

        windowNotSet=True
        safe = classifyImage(img, boxes, scores)
      

        (boxes, scores, classes, num_detections) = tDetector.run(image)

        vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=4)
        if windowNotSet is True:
            cv2.namedWindow("tensorflow based (%d, %d)" % (w, h), cv2.WINDOW_NORMAL)
            windowNotSet = False

        cv2.imshow("tensorflow based (%d, %d)" % (w, h), image)
        k = cv2.waitKey(1) & 0xff
        if k == ord('q') or k == 27:
            
            print("I")
            print("NEED")
            print("MORE")
            print("FACES!!!")
            break
        for i, score in enumerate(scores[0]):
            if score > .7:
                face = crop(image, boxes, i)
                [h, w] = face.shape[:2]
                if h > 40 and w > 40:
                    name = "{}.jpg".format(unicode(datetime.now()))
                    cv2.imwrite('tf_files/' + name, face)

                    classify(name)
        print "all done"
        #cv2.drawContours()
    d = time.time()
    # cv2.imshow("tensorflow based (%d, %d)" % (w, h), image)
    #key=cv2.waitKey(1) & 0xFF
    if (key == ord('q')):
        break
    fps.update()
    print "find faces {}".format(b-a)
    print "find distance {}".format(c-b)
    print "classify face {}".format(d-c)
fps.stop()
print("FPS: {:.2f}".format( fps.fps() ) )
camL.release()
camR.release()
cv2.destroyAllWindows()
