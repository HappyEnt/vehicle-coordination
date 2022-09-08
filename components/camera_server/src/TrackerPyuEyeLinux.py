'''
Created on 08.10.2021.
Most parts of this code were coded by Jonas Lange.
This program can be used on a Raspberry Pi.

This program can read camera data from a UEye camera and detect aruco markers in the video.
@author: torben
'''
import cv2 as cv
import numpy as np
import time
import ctypes
from CameraConverter import CameraConverter
from pyueye import ueye
import numpy as np
import cv2

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
    

    #setup videostream like in the example given

    #===========================================================================#
    #                                                                           #
    #  Copyright (C) 2006 - 2018                                                #
    #  IDS Imaging Development Systems GmbH                                     #
    #  Dimbacher Str. 6-8                                                       #
    #  D-74182 Obersulm, Germany                                                #
    #                                                                           #
    #  The information in this document is subject to change without notice     #
    #  and should not be construed as a commitment by IDS Imaging Development   #
    #  Systems GmbH. IDS Imaging Development Systems GmbH does not assume any   #
    #  responsibility for any errors that may appear in this document.          #
    #                                                                           #
    #  This document, or source code, is provided solely as an example          #
    #  of how to utilize IDS software libraries in a sample application.        #
    #  IDS Imaging Development Systems GmbH does not assume any responsibility  #
    #  for the use or reliability of any portion of this document or the        #
    #  described software.                                                      #
    #                                                                           #
    #  General permission to copy or modify, but not for profit, is hereby      #
    #  granted, provided that the above copyright notice is included and        #
    #  reference made to the fact that reproduction privileges were granted     #
    #  by IDS Imaging Development Systems GmbH.                                 #
    #                                                                           #
    #  IDS Imaging Development Systems GmbH cannot assume any responsibility    #
    #  for the use or misuse of any portion of this software for other than     #
    #  its intended diagnostic purpose in calibrating and testing IDS           #
    #  manufactured cameras and software.                                       #
    #                                                                           #
    #===========================================================================#

    # Developer Note: I tried to let it as simple as possible.
    # Therefore there are no functions asking for the newest driver software or freeing memory beforehand, etc.
    # The sole purpose of this program is to show one of the simplest ways to interact with an IDS camera via the uEye API.
    # (XS cameras are not supported)
    #---------------------------------------------------------------------------------------------------------------------------------------

    #Libraries


    #---------------------------------------------------------------------------------------------------------------------------------------
    # init camera
    hcam = ueye.HIDS(0)
    ret = ueye.is_InitCamera(hcam, None)
    print(f"initCamera returns {ret}")

    # set color mode
    ret = ueye.is_SetColorMode(hcam, ueye.IS_CM_BGR8_PACKED)
    print(f"SetColorMode IS_CM_BGR8_PACKED returns {ret}")

    exposure_ms = 10
    exposure_ms_double = ctypes.c_double(exposure_ms)
    nRet = ueye.is_Exposure(
        hcam,
        ueye.IS_EXPOSURE_CMD_SET_EXPOSURE,
        exposure_ms_double,
        ctypes.sizeof(exposure_ms_double),
        )
    if nRet != ueye.IS_SUCCESS:
        raise RuntimeError("IS_EXPOSURE_CMD_SET_EXPOSURE failed")
    actual = exposure_ms_double.value
    if actual != exposure_ms:
        print("Warning: actual value of exposure time is", actual, "ms")

    # set region of interest
    width = 1280
    height = 1080
    rect_aoi = ueye.IS_RECT()
    rect_aoi.s32X = ueye.int(0)
    rect_aoi.s32Y = ueye.int(0)
    rect_aoi.s32Width = ueye.int(width)
    rect_aoi.s32Height = ueye.int(height)
    ueye.is_AOI(hcam, ueye.IS_AOI_IMAGE_SET_AOI, rect_aoi, ueye.sizeof(rect_aoi))
    print(f"AOI IS_AOI_IMAGE_SET_AOI returns {ret}")

    # allocate memory
    mem_ptr = ueye.c_mem_p()
    mem_id = ueye.int()
    bitspixel = 24 # for colormode = IS_CM_BGR8_PACKED
    ret = ueye.is_AllocImageMem(hcam, width, height, bitspixel,
                                mem_ptr, mem_id)
    print(f"AllocImageMem returns {ret}")

    # set active memory region
    ret = ueye.is_SetImageMem(hcam, mem_ptr, mem_id)
    print(f"SetImageMem returns {ret}")

    # continuous capture to memory
    ret = ueye.is_CaptureVideo(hcam, ueye.IS_DONT_WAIT)
    print(f"CaptureVideo returns {ret}")

    # get data from camera and display
    lineinc = width * int((bitspixel + 7) / 8)

    #---------------------------------------------------------------------------------------------------------------------------------------

    #set dictionary and parameters
    dictionary = cv.aruco.Dictionary_get(marker_dictionary)
    parameters =  cv.aruco.DetectorParameters_create()

    #sets a high frame rate for low exposure time
    #For a exposure time of 4 ms we must put a FPS of 1000/4=250

    time_start = time.time()
    time_between_frames = 1/max_fps

    # extract coordinates of anchors
    anchors = cameraConverter.calculateAnchors()
    connector_obj.anchors = anchors
    
    while(True):
        #set up the frame like in the example
        
        # In order to display the image in an OpenCV window we need to...
        # ...extract the data of our image memory
        array = ueye.get_data(mem_ptr, width, height, bitspixel, lineinc, copy=True)

        # bytes_per_pixel = int(nBitsPerPixel / 8)

        # ...reshape it in an numpy array...
        frame = np.reshape(array, (height, width, 3))

        # ...resize the image
        frame = cv2.resize(frame,(0,0),fx=0.7, fy=0.7)
        
        #gray out the image
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            
        #unwrap the strange datastructures returned by the detectMarkers function
        unwraped = unwrap(markerCorners, markerIds)

        
        #send positions to connection
        #marker_positions = positionMarkers(markerCorners, markerIds)
        relative_marker_positions, boundry_positions = getRelativePositions(unwraped)
        # check, if we got our boundaries correct 
        if not (-1, -1) in boundry_positions:
            #print(f"sending {relative_marker_positions}"))
            
            #limit fps to the max_fps
            time_now = time.time()
            time_difference = time_now - time_start
            time_start = time_now
            if(time_difference < time_between_frames):
                time.sleep(time_between_frames-time_difference)

            cameraConverter.sendCameraValues(relative_marker_positions)
            connector_obj.cam_data = cameraConverter.get_centered_coordinates()
            # s.send(bytes(json.dumps(relative_marker_positions), "utf-8"))

            
        
        #for debugging: show detected markers
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if((-1,-1) not in boundry_positions):
            for i in range(0,3):
                frame = cv2.line(frame, (int(boundry_positions[i][0]), int(boundry_positions[i][1])),
                                (int(boundry_positions[i+1][0]), int(boundry_positions[i+1][1])), (0, 255, 0), thickness=3)
            frame = cv2.line(frame, (int(boundry_positions[3][0]), int(boundry_positions[3][1])),
                            (int(boundry_positions[0][0]), int(boundry_positions[0][1])), (0, 255, 0), thickness=3)
        frame = cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        if not headless:
            cv.imshow('frame', frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        
    # if tracker is exited, end like in example
    #---------------------------------------------------------------------------------------------------------------------------------------

    # cleanup
    ret = ueye.is_StopLiveVideo(hcam, ueye.IS_FORCE_VIDEO_STOP)
    print(f"StopLiveVideo returns {ret}")
    ret = ueye.is_ExitCamera(hcam)
    print(f"ExitCamera returns {ret}")

    # Destroys the OpenCv windows
    cv2.destroyAllWindows()

    print()
    print("END")









