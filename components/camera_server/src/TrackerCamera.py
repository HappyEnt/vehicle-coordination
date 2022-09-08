'''
Created on 08.10.2021.
Most parts of this code were coded by Jonas Lange.

This file replaces the file TrackerPyuEyeLinux.py. This program can be used for cameras, which are not a uEye camera.
@author: torben
'''
import cv2 as cv
import numpy as np
import time
from CameraConverter import CameraConverter

def start(connector_obj, headless):
    marker_dictionary = cv.aruco.DICT_4X4_50 # Dictionary
    max_fps = 60

    cameraConverter = CameraConverter()

    def unwrap(corners, ids):
        """
        Takes the strange structure returned by the aruco detection and returns a list of positions,
        each position beeing a list of 4 2d vectors of the corners the order stays the same
        """
        if ids is None:
            return []    
        
        unwraped = []
        for i in range(0, len(corners)):
            unwraped.append([int(ids[i][0]),
                            [corners[i].item(0), corners[i].item(1)],
                            [corners[i].item(2), corners[i].item(3)],
                            [corners[i].item(4), corners[i].item(5)],
                            [corners[i].item(6), corners[i].item(7)],
                            ])
        return unwraped


    def getRelativePositions(positions):
        """
        analyzes the positions and returns the positions relative to the marked testing area. This is done using
        a homography.
        """
        #filter the markers from positions that mark the boundry of the testing area
        boundry_markers= [(-1,-1),(-1,-1),(-1,-1),(-1,-1)]
        for i in range(0, len(positions)):
            if positions[i][0] == 0:
                #the marker denotes the top left corner of the testing area
                boundry_markers[0]=(positions[i][3][0], positions[i][3][1])
            elif positions[i][0] == 1:
                #the marker denotes the top right corner of the testing area
                boundry_markers[1]=(positions[i][4][0], positions[i][4][1])
            elif positions[i][0] == 2:
                #the marker denotes the bottom right corner of the testing area
                boundry_markers[2]=(positions[i][1][0], positions[i][1][1])
            elif positions[i][0] == 3:
                #the marker denotes the bottom left corner of the testing area
                boundry_markers[3]=(positions[i][2][0], positions[i][2][1])
        
        #remove boundry_markers from positions    
        for i in range(0,4):
            for position in positions:
                if position[0] == i:
                    positions.remove(position)
            
        if (-1,-1) in boundry_markers:
            #the empty array is returned because the testing area is not properly defined
            return [], boundry_markers
        
        #set homography
        pts_src = np.array(boundry_markers)
        pts_dst = np.array([(0,0),(1,0),(1,1),(0,1)])
        h, status = cv.findHomography(pts_src, pts_dst)    
        
        #use transformation on the markers positions
        #print("transformed positions:")
        for position in positions:
            for c in range(1,5): #iterate through all 4 corners of each marker            
                A = np.array([[[position[c][0], position[c][1]]]], dtype=np.float32)
                transformed_position = cv.perspectiveTransform(src = A, m = h)
                #print(position[c])
                #print(transformed_position)
                #save as string becase float32 is not json serializable
                position[c][0] = str(transformed_position[0][0][0])
                position[c][1] = str(transformed_position[0][0][1])        
        #print(f"realtive positions are: {positions}")
        return positions, boundry_markers

    #####################
    # Main program start
    #####################
    cap = cv.VideoCapture()
    # The device number might be 0 or 1 depending on the device and the webcam
    cap.open(0, cv.CAP_DSHOW)

    #set dictionary and parameters
    dictionary = cv.aruco.Dictionary_get(marker_dictionary)
    parameters =  cv.aruco.DetectorParameters_create()

    time_start = time.time()
    time_between_frames = 1/max_fps

    # extract coordinates of anchors
    anchors = cameraConverter.calculateAnchors()
    connector_obj.anchors = anchors

    while(True):
        ret, frame = cap.read()
        
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        #unwrap the strange datastructures returned by the detectMarkers function
        unwraped = unwrap(markerCorners, markerIds)
        relative_marker_positions, boundry_positions = getRelativePositions(unwraped)

        if not relative_marker_positions == []:
            #print(f"sending {relative_marker_positions}"))
            
            #limit fps to the max_fps
            time_now = time.time()
            time_difference = time_now - time_start
            time_start = time_now
            if(time_difference < time_between_frames):
                time.sleep(time_between_frames-time_difference)

            cameraConverter.sendCameraValues(relative_marker_positions)
            connector_obj.cam_data = cameraConverter.get_centered_coordinates()
        # show debug view of the aruco detection
        # cv.imshow('frame', frame)
        frame2 = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        if((-1,-1) not in boundry_positions):
            for i in range(0,3):
                frame2 = cv.line(frame2, (int(boundry_positions[i][0]), int(boundry_positions[i][1])),
                                (int(boundry_positions[i+1][0]), int(boundry_positions[i+1][1])), (0, 255, 0), thickness=3)
            frame = cv.line(frame2, (int(boundry_positions[3][0]), int(boundry_positions[3][1])),
                            (int(boundry_positions[0][0]), int(boundry_positions[0][1])), (0, 255, 0), thickness=3)
        frame = cv.aruco.drawDetectedMarkers(frame2, markerCorners, markerIds)
        if not headless:
            cv.imshow('frame2', frame2)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    cv.destroyAllWindows()
    cap.release()
