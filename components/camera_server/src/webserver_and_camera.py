'''
Created on 09.06.2022.
This program connects the webserver and the camera, by running the two systems in different threads.
It also provides some storage functions.
@author: torben
'''
import threading
import sys
import json
from scipy.spatial import distance
import argparse

import flask_server
import TrackerPyuEyeLinux as camera # uEye. choose one of those, depending on if you use an uEye camera or not.
# import TrackerCamera as camera # no uEye. choose one of those, depending on if you use an uEye camera or not.

webserver = True
headless = False

ANCHORS = [0,1,2,3]

class Connector:
    '''
    Object for connection of services.
    '''
    def __init__(self):
        print("Created Connector object")
        self.cam_data = {}
        self.anchors = {}
        self.particle_storage = {}
        # estimate has scheme: (self.id_int, [estimate_x, estimate_y], self.car_radius, guess_radius)
        self.estimate_storage = {}
        self.data_storage = {} # storage for all kind of data, to prevent always needing to add more reqeusts (dict of dicts)

    def cam_data_json(self):
        return json.dumps(self.cam_data)

    def distance(self, marker_id_1, marker_id_2):
        '''
        Calculate the distance between two nodes (anchors or tags).
        '''
        marker_id_1 = int(marker_id_1)
        marker_id_2 = int(marker_id_2)
        try:
            if marker_id_1 in ANCHORS:
                marker_1 = self.anchors[marker_id_1]
            else:
                marker_1 = self.cam_data[marker_id_1]

            if marker_id_2 in ANCHORS:
                marker_2 = self.anchors[marker_id_2]
            else:
                marker_2 = self.cam_data[marker_id_2]
            # return "Distance between " + str(marker_id_1) + " and " + str(marker_2) + " is " + str(distance.euclidean(marker_1, marker_2))
            return distance.euclidean(marker_1, marker_2)
        except KeyError:
            return {}

    def get_one_pos(self, marker_id):
        '''
        Return the position of one car/marker.
        '''
        marker_id = int(marker_id)
        try:
            if marker_id in ANCHORS:
                marker = self.anchors[marker_id]
            else:
                marker = self.cam_data[marker_id]
                
            return str(marker)
        except KeyError:
            return {}

    def receive_particles(self, marker_id, particles):
        '''
        Receive particles from a node and save them locally.
        '''
        for key in self.anchors.keys():
            self.particle_storage[key] = [self.anchors[key]]
        marker_id = int(marker_id)
        self.particle_storage[marker_id] = particles

    def get_saved_particles(self, marker_id):
        '''
        Return all saved particles.
        '''
        for key in self.anchors.keys():
            self.particle_storage[key] = [self.anchors[key]]
        marker_id = int (marker_id)
        try:
            particles = self.particle_storage[marker_id]
            return particles
        except KeyError:
            return {}

    def clear_particles(self):
        '''
        Deletes all non anchor-particles.
        '''
        self.particle_storage = {}
        for key in self.anchors.keys():
            self.particle_storage[key] = [self.anchors[key]]

    def receive_estimate(self, marker_id, estimate):
        '''
        Receive estimate from a node and save it locally.
        '''
        for key in self.anchors.keys():
            self.estimate_storage[key] = (key, self.anchors[key], 2, 1)
        marker_id = int(marker_id)
        self.estimate_storage[marker_id] = estimate

    def get_saved_estimate(self, marker_id):
        '''
        Return a specific saved estimate.
        '''
        marker_id = int (marker_id)
        try:
            estimates = self.estimate_storage[marker_id]
            return estimates
        except KeyError:
            return {}

    def clear_estimates(self):
        '''
        Deletes all estimates.
        '''
        self.estimate_storage = {}

    def get_data(self, data_keyword : str):
        '''
        Return data from internal data storage.
        '''
        data_keyword = data_keyword.lower()
        if data_keyword == "all":
            return self.data_storage
        else:
            if data_keyword in self.data_storage:
                return self.data_storage[data_keyword]
            else:
                return {}
    
    def set_data(self, data_keyword : str, data):
        self.data_storage[data_keyword] = data
        return {}

    def close(self):
        sys.exit()


if __name__ == "__main__":
    connector_obj = Connector()
    
    parser = argparse.ArgumentParser(description='Start the camera and a webserver to retrieve data from the camera.')
    parser.add_argument(
            "--headless",
            action="store_true",
            help="runs the webserver in headless mode, meaning do not use a GUI to show the markers.",
        )
    args = parser.parse_args()

    if webserver:
        thread1 = threading.Thread(target=flask_server.run, args=[connector_obj], daemon=True)
        thread1.start()

    thread2 = threading.Thread(target=camera.start, args=[connector_obj, args.headless], daemon=True)
    thread2.start()

    while(True):
        # print(connector_obj.cam_data)
        continue
