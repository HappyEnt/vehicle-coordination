'''
Created on 08.10.2021.
Some logic of this relies on code by Jonas Lange.

This program does some math for detected aruco markers in a camera-stream.
It can calculate coordinates or distances between markers. The program uses a config file, which holds the length of the edges between a four-edge field, marked by aruco markers.
@author: torben
'''
import configparser
import math

class CameraConverter:
    '''
    Class to convert values of the camera to distances, etc.
    '''
    def __init__(self, path_to_config="config.ini"):
        # Read information about the testing area
        self.path_to_config = path_to_config
        config = configparser.ConfigParser()
        config.read(path_to_config)
        # x-axis, in m
        self.testing_area_length = float(config['DEFAULT']['testing_area_length'])
        # y-axis, in m
        self.testing_area_width = float(config['DEFAULT']['testing_area_width'])
        self.markerCoords = dict()
        self.markerCoordsCentered = dict()
        self.numberToUUID = dict()
        self.anchors = dict()

    def sendCameraValues(self, data):
        '''
        Gets the camera values from the camera.
        '''
        #              MarkerName coordinates (tl-corner of aruco)    coordinates(tr)            coordinates(br)                coordinates(bl)         other marker
        #                   v                v                             v                           v                             v                     v
        # data structure: [[5, ['0.07692605', '0.45075944'], ['0.44681168', '0.4664847'], ['0.44756806', '0.9074692'], ['0.06501283', '0.87948215']], [4, ['0.514744', '0.15360369'], ['0.89275146', '0.16822404'], ['0.89788616', '0.601223'], ['0.5107496', '0.5755567']]]
        # iterate through all detected non-corner aruco markers
        self.markerCoords = dict()
        self.markerCoordsCentered = dict()
        for i in range(0, len(data)):
            # get distance values to corners for this marker
            marker = data[i]
            marker_name = marker[0]
            marker = marker[1:]
            self.calculateCoordinates(marker, marker_name)

    def calculateCoordinates(self, markerData, markerName):
        '''
        Calculates the coordinates of a marker using the distance to all corners if the testing area.
        '''
        # calculate the absolute coords with the config
        coords_list = []
        for i in range(len(markerData)):
            x = float(markerData[i][0]) * self.testing_area_length
            y = float(markerData[i][1]) * self.testing_area_width
            coords_list.append([x,y])
            
        # add solution to local dict
        self.markerCoords[markerName] = coords_list
        self.markerCoordsCentered[markerName] = self.getCenter(markerName)

    def calculateAnchors(self):
        '''
        Calculate the coordinates of the corner markers.
        '''
        self.anchors[0] = [0,0]
        self.anchors[1] = [self.testing_area_length, 0]
        self.anchors[2] = [self.testing_area_length, self.testing_area_width]
        self.anchors[3] = [0, self.testing_area_width]
        return self.anchors

    def pythagoras(self, a, b):
        '''
        Calculates the Pythagoras' theorem for a and b.
        '''
        return math.sqrt(a * a + b * b)

    def getCenter(self, markerName):
        '''
        Calculates the absolute coordinates of the center point of the aruco marker.
        '''
        if markerName in self.markerCoords:
            coords = self.markerCoords[markerName]
            tl = coords[0]
            br = coords[2]
            tlbrMidPoint = [(br[0] - tl[0]) / 2.0, (br[1] - tl[1]) / 2.0]
            # print(tlbrMidPoint)
            return [tl[0] + tlbrMidPoint[0], tl[1] + tlbrMidPoint[1]]
        else:
            # marker isn't in saved dictionary
            return []

    def getDistance(self, markerName1, markerName2):
        '''
        Returns the distance between the middle points of two aruco markers.
        '''
        if markerName1 == markerName2:
            return 0
        
        center1 = self.getCenter(markerName1)
        center2 = self.getCenter(markerName2)

        # one of ther markers isn't in dictionary
        if center1 == [] or center2 == []:
            return -1
        
        return self.pythagoras(center2[0] - center1[0], center2[1] - center1[1])

    def get_centered_coordinates(self):
        return self.markerCoordsCentered
