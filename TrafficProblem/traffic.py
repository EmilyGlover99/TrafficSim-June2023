import numpy as np
import os, sys
import time
import subprocess
import shutil
import pandas as pd
from multiprocessing import Pool
from itertools import chain, combinations
import Networks.Original.network_traffic as traffic_densities
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    print(tools)
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import traci.constants

import sumolib

sumoBinary = "F:\Programming Files\Eclipse" + os.sep + "Sumo" + os.sep + "bin" + os.sep + "sumo-gui.exe"

dir_path = os.path.dirname(os.path.realpath(__file__))


class TrafficProblemManager:
    networks = []
    node_ids = []
    temp = []
    debug_mode = False
    # network_lanes = []
    # current_emissions = {}
    # traffic_flows = []
    # current_vehicles = []
    # lane_information = {}
    # initial_matrix = []
    vehicle_counter = 0
    crude_counter = 0

    # test_cars = []
    # test_street = "-23353548#1"  # We wish to reduce traffic on this street
    # action_set = ["-25156946#0","-23353543","-23353378#0","-23353548#2","-23353532#3"]
    # action_set = ["-1069087493#1","-23353548#1","-25156946#0","-23353553#0","-23353532#3","-1082152568"]
    # action_set = ["-43308108#6", "-43308108#8", "-25099570", "-1069087493#1", "-23353543"]

    test_set = []

    def __init__(self, debug):
        self.temp.append(['.Original/'])
        self.debug_mode = debug
        self.net = sumolib.net.readNet('./Networks/Original/osm.net.xml.gz')
        # self.edge_ids = self.net.getEdges()
        # for edge in self.edge_ids:
        #     edge_location = sumolib.net.edge.Edge.getShape(edge)
        #     print(edge_location)
        #
        # print(self.edge_ids)

    def runState(self, state, runparameter=1000, testvehicles="", teststreet=""):
        if state == "Original":
            state = os.sep + "Networks\\Original" + os.sep
        else:
            state = os.sep + "Networks\\Temp" + os.sep + str(state) + os.sep
        # testvehicles = self.test_cars
        # def runState(self, params):
        #     (state, runparameter, testvehicles, teststreet) = params
        #     print(dir_path + state + "osm.sumocfg")
        sumoCmd = [sumoBinary, "-c", dir_path + state + "osm.sumocfg", "--time-to-teleport=10000", "--start",
                   "--verbose=False", "--duration-log.disable=True", "--duration-log.statistics=False",
                   "--no-step-log=True", "--threads=4","--step-length=0.5"]
        # sumoCmd = [sumoBinary, "-c", dir_path + state + "osm.sumocfg", "--time-to-teleport=10000", "--start",
        #            "--quit-on-end", "--verbose=False", "--duration-log.disable=True", "--duration-log.statistics=False",
        #            "--no-step-log=True", "--threads=4"]
        if self.debug_mode: print("Starting SUMO")
        traci.start(sumoCmd)

        # Run the network in SUMO
        j = 0  # j is the time step
        # Set the end point
        if runparameter != -1:
            last_j = runparameter
        else:
            last_j = 5000
        initial_journey_time = {}
        total_journey_time = 0
        vehicles_still_travelling = []
        counter = 0
        # for vehicle in testvehicles:
        #     vehicles_still_travelling.append(["TestVehicle_" + str(counter), vehicle[0], vehicle[1]])
        #     counter += 1

        while (j < last_j):
            # for each time step (which equals 1 second)
            traci.simulationStep()
            #time.sleep(0.25)  # delay by 1 to allow time for viewing the gui

            if j % 8 == 0:
                # every 5 seconds, spawn a car from residential
                # Spawn one car in each residential area
                self.spawn_vehicle(0, "residential")
                self.spawn_vehicle(1, "residential")
                self.spawn_vehicle(2, "residential")
                self.spawn_vehicle(3, "residential")
                self.spawn_vehicle(4, "residential")
            if j % 25 == 0:
                # every 15 seconds, spawn a car from city
                self.spawn_vehicle(-1, "city")
            if j % 35 == 0:
                # every 15 seconds, spawn a car from retail
                self.spawn_vehicle(-1, "retail")
            if j % 50 == 0:
                # every 20 seconds, spawn a car from industrial
                    self.spawn_vehicle(-1, "industrial")
            if j % 25 == 0:
                # every 4 seconds, spawn a car on the minor roads
                for k in range(0, len(traffic_densities.minor_entrances)):
                    self.spawn_vehicle(k, "minor")
            if j % 12 == 0:
                # every 4 seconds, spawn a car on the major roads
                for k in range(0,len(traffic_densities.main_entrances)):
                    self.spawn_vehicle(k, "major")

            # if j == 5:
            #     # add test vehicles
            #     vehicle_counter = 0
            #     for vehicle in testvehicles:
            #         # print(vehicle)
            #         # print(vehicle[0])
            #         # print(vehicle[1])
            #         vehicle_route = traci.simulation.findRoute(fromEdge="" + str(vehicle[1]) + "",
            #                                                    toEdge="" + str(vehicle[0]) + "")
            #         traci.route.add("TestRoute_" + str(vehicle_counter), vehicle_route.edges)
            #         traci.vehicle.add("TestVehicle_" + str(vehicle_counter), "TestRoute_" + str(vehicle_counter))
            #         initial_journey_time["TestVehicle_" + str(vehicle_counter)] = traci.simulation.getTime()
            #         vehicle_counter += 1
            # if j > 6:
            #     for vehicle in vehicles_still_travelling:
            #         if self.check_vehicle_position(vehicle[0], vehicle[2]):
            #             # This vehicle has reached the end point
            #             total_journey_time = total_journey_time + traci.simulation.getTime() - initial_journey_time[
            #                 vehicle[0]]
            #             # print(str(vehicle[0]) + " has reached the end!")
            #             # Remove this vehicle from list
            #             vehicles_still_travelling.remove(vehicle)
            #         # print(vehicles_still_travelling)
            #         # print(total_journey_time)
            #         last_j = j + 500  # Play about with this value (It is used to set a cap on simulation times)
            #
            #     if len(vehicles_still_travelling) == 0:
            #         traci.close(wait=False)
            #         # print(total_journey_time)
            #         return total_journey_time
            if j > last_j:
                j = last_j
            j += 1
        # for vehicle in vehicles_still_travelling:
        #     total_journey_time = total_journey_time + traci.simulation.getTime() - initial_journey_time[vehicle[0]]

        traci.close(wait=False)
        return total_journey_time

    def check_vehicle_position(self, VehID, TargetEdge):
        current_edge_ID = traci.vehicle.getRoadID(VehID)

        if (current_edge_ID == TargetEdge):
            return True
        else:
            return False

    def spawn_vehicle(self, from_id, from_type, sub_id = -1):
        destination_type = 0
        start_road = ""
        match from_type:
            case "residential":
                start_road = self.generate_residential_road(from_id)
                destination_type = np.random.choice(np.arange(len(traffic_densities.route_distribution_from_residential_probabilities)), p=traffic_densities.route_distribution_from_residential_probabilities)
            case "city":
                new_point = np.random.multivariate_normal(traffic_densities.city_centre, np.matrix([[200 * 200, 0], [0, 150 * 150]]))
                radius = 100
                found = False
                list_edges = []
                while not found:
                    nearest_edges = self.net.getNeighboringEdges(new_point[0], new_point[1], r=radius)
                    if len(nearest_edges) == 0:
                        radius += 50
                    else:
                        for edge in nearest_edges:
                            edge_data, dist = edge
                            list_edges.append(dist)
                        found = True
                        start_road = nearest_edges[list_edges.index(min(list_edges))][0].getID()
                destination_type = np.random.choice(np.arange(len(traffic_densities.route_distribution_from_city_probabilities)), p=traffic_densities.route_distribution_from_city_probabilities)

            case "retail":
                start_road = traffic_densities.retail_park[1]
                destination_type = np.random.choice(np.arange(len(traffic_densities.route_distribution_from_retail_probabilities)), p=traffic_densities.route_distribution_from_retail_probabilities)

            case "industrial":
                start_road = traffic_densities.industrial_estate[1]
                destination_type = np.random.choice(np.arange(len(traffic_densities.route_distribution_from_industrial_probabilities)), p=traffic_densities.route_distribution_from_industrial_probabilities)

            case "minor":
                start_road = traffic_densities.minor_entrances[from_id]
                destination_type = np.random.choice(np.arange(len(traffic_densities.route_distribution_from_minor_road_probabilities)), p=traffic_densities.route_distribution_from_minor_road_probabilities)

            case "major":
                start_road = traffic_densities.main_entrances[from_id]
                destination_type = np.random.choice(np.arange(len(traffic_densities.route_distribution_from_major_road_probabilities)), p=traffic_densities.route_distribution_from_major_road_probabilities)

        # Generate route:
        end_road = ""
        match destination_type:
            # 0 = residential, 1 = city centre, 2 = retail, 3 = industrial, 4 = minor road, 5 = major road
            case 0:
                # to residential
                if from_type == "residential":
                    # we need to be careful not to send to same location
                    area = random.choice(list(range(0, from_id)) + list(range(from_id + 1, 5)))
                    end_road = self.generate_residential_road(area)
                else:
                    # pick a residential area at random
                    area = random.choice(list(range(0,5)))
                    end_road = self.generate_residential_road(area)
            case 1:
                # to city centre
                new_point = np.random.multivariate_normal(traffic_densities.city_centre, np.matrix([[200 * 200, 0], [0, 150 * 150]]))
                radius = 100
                found = False
                list_edges = []
                while not found:
                    nearest_edges = self.net.getNeighboringEdges(new_point[0], new_point[1], r=radius)
                    if len(nearest_edges) == 0:
                        radius += 50
                    else:
                        for edge in nearest_edges:
                            edge_data, dist = edge
                            list_edges.append(dist)
                        found = True
                        end_road = nearest_edges[list_edges.index(min(list_edges))][0].getID()
            case 2:
                # to retail park
                end_road = traffic_densities.retail_park[0]
            case 3:
                # to industrial road
                end_road = traffic_densities.industrial_estate[0]
            case 4:
                # to minor road
                if from_type == "minor":
                    # ensure we don't send it to the same road
                    end_road = traffic_densities.minor_exits[random.choice(list(range(0, from_id)) + list(range(from_id + 1, len(traffic_densities.minor_entrances))))]
                else:
                    end_road = traffic_densities.minor_exits[random.choice(list(range(0, len(traffic_densities.minor_entrances))))]
            case 5:
                # to major road
                if from_type == "major":
                    # ensure we don't send it to the same road
                    end_road = traffic_densities.main_exits[random.choice(list(range(0, from_id)) + list(range(from_id + 1, len(traffic_densities.main_entrances))))]
                else:
                    end_road = traffic_densities.main_exits[random.choice(list(range(0, len(traffic_densities.main_entrances))))]

        if start_road != "" and end_road != "":
            self.vehicle_counter += 1
            # print("start_road = " + str(start_road) + " and end_road = " + str(end_road))
            vehicle_route = traci.simulation.findRoute(fromEdge=start_road, toEdge=end_road)
            traci.route.add("BackgroundRoute_" + str(self.vehicle_counter), vehicle_route.edges)
            traci.vehicle.add("TestVehicle_" + str(self.vehicle_counter), "BackgroundRoute_" + str(self.vehicle_counter))

        return 5

    def generate_residential_road(self, area_id):
        new_point = [0,0]
        match area_id:
            case 0:
                # generate from residential_0
                # First, generate a point from the residential_0 distribution
                new_point = np.random.multivariate_normal(traffic_densities.residential_0, np.matrix([[500 * 500, 0], [0, 500 * 500]]))
            case 1:
                # generate a point from the residential_1 distribution
                new_point = np.random.multivariate_normal(traffic_densities.residential_1, np.matrix([[500 * 500, 0], [0, 450 * 450]]))
            case 2:
                # generate a point from the residential_2 distribution
                new_point = np.random.multivariate_normal(traffic_densities.residential_2,  np.matrix([[400 * 400, 0], [0, 500 * 500]]))
            case 3:
                # generate a point from the residential_3 distribution
                new_point = np.random.multivariate_normal(traffic_densities.residential_3, np.matrix([[500 * 500, 0], [0, 350 * 350]]))
            case 4:
                # generate a point from the residential_4 distribution
                new_point = np.random.multivariate_normal(traffic_densities.residential_4, np.matrix([[500 * 500, 0], [0, 500 * 500]]))

        radius = 100
        found = False
        list_edges = []
        while not found:
            nearest_edges = self.net.getNeighboringEdges(new_point[0], new_point[1], r=radius)
            if len(nearest_edges) == 0:
                radius += 50
            else:
                for edge in nearest_edges:
                    edge_data, dist = edge
                    list_edges.append(dist)
                closest_edge = nearest_edges[list_edges.index(min(list_edges))][0].getID()
                found = True
                return closest_edge




Manager = TrafficProblemManager(debug=False)
Manager.runState("Original")