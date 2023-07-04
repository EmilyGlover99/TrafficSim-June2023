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
import xml.etree.ElementTree as ET

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    print(tools)
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import traci.constants

import sumolib

sumoBinary = "F:\Programming Files\Eclipse" + os.sep + "Sumo" + os.sep + "bin" + os.sep + "sumo.exe"
# sumoBinary = "/usr/local/share/sumo/bin" + os.sep + "sumo-gui"
dir_path = os.path.dirname(os.path.realpath(__file__))


class TrafficProblemManager:
    networks = []
    node_ids = []
    temp = []
    debug_mode = False
    vehicle_counter = 0
    total_pollution =[]

    def __init__(self, debug):
        self.temp.append(['.Original/'])
        self.debug_mode = debug
        self.net = sumolib.net.readNet('./Networks/Original/osm.net.xml.gz')

    def runState(self, state, runparameter=1000):
        if state == "Original":
            state = os.sep + "Networks" + os.sep + "Original" + os.sep
        else:
            state = os.sep + "Networks" + os.sep + "Modified" + os.sep + str(state) + os.sep

        # sumoCmd = [sumoBinary, "-c", dir_path + state + "osm.sumocfg", "--time-to-teleport=10000", "--start",
        #            "--verbose=False", "--duration-log.disable=True", "--duration-log.statistics=False",
        #            "--no-step-log=True", "--threads=4","--step-length=0.5"]
        sumoCmd = [sumoBinary, "-c", dir_path + state + "osm.sumocfg", "--time-to-teleport=10000", "--start",
                   "--quit-on-end", "--verbose=False", "--duration-log.disable=True", "--duration-log.statistics=False",
                   "--no-step-log=True", "--threads=4","--step-length=0.5"]
        if self.debug_mode: print("Starting SUMO")
        traci.start(sumoCmd)
        all_edges = traci.edge.getIDList()
        vehicles = []
        # Check if vehicles have already been generated if not generate them!
        if not os.path.isfile(dir_path + os.sep + "vehicles.csv"):
            self.generate_vehicles(1000, dir_path + os.sep + "vehicles.csv")
            vehicles = pd.read_csv(dir_path + os.sep + "vehicles.csv")
        else:
            vehicles = pd.read_csv(dir_path + os.sep + "vehicles.csv")

        # Run the network in SUMO
        j = 0  # j is the time step
        # Set the end point
        if runparameter != -1:
            last_j = runparameter
        else:
            last_j = 5000

        while (j < last_j):
            # for each time step (which equals 1 second)
            traci.simulationStep()

            # Generate cars from vehicles.csv for current timestep
            temp = vehicles.loc[vehicles['timestep'] == j]
            if len(temp) != 0:
                # There are vehicles which match this timestep, generate them!
                for index, row in temp.iterrows():
                    if row[1] in all_edges and row[2] in all_edges:
                        self.vehicle_counter += 1
                        vehicle_route = traci.simulation.findRoute(fromEdge=row[1], toEdge=row[2])
                        traci.route.add("BackgroundRoute_" + str(self.vehicle_counter), vehicle_route.edges)
                        traci.vehicle.add("TestVehicle_" + str(self.vehicle_counter),
                                          "BackgroundRoute_" + str(self.vehicle_counter))

            self.read_road_sensors()

            if j > last_j:
                j = last_j
            j += 1
        traci.close(wait=False)
        print(str(sum(self.total_pollution)/500))
        return sum(self.total_pollution)/500

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
            return start_road, end_road

        return 5

    def read_road_sensors(self):
        sensors = traffic_densities.road_sensors

        if not self.total_pollution:
            for sensor in sensors:
                self.total_pollution.append(traci.edge.getCOEmission(sensor))
        else:
            i = 0
            for sensor in sensors:
                self.total_pollution[i] = self.total_pollution[i] + traci.edge.getCOEmission(sensor)
                i+= 1
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

    def generate_vehicles(self, max_timestep, output_dir):
        vehicles = []
        progress_count = 0
        progress_range = 10
        for j in range(0,max_timestep):
            if j % 8 == 0:
                # every 5 seconds, spawn a car from residential
                # Spawn one car in each residential area
                car_locations = self.spawn_vehicle(0, "residential")
                vehicles.append([j, car_locations[0], car_locations[1]])

                car_locations = self.spawn_vehicle(1, "residential")
                vehicles.append([j, car_locations[0], car_locations[1]])

                car_locations = self.spawn_vehicle(2, "residential")
                vehicles.append([j, car_locations[0], car_locations[1]])

                car_locations = self.spawn_vehicle(3, "residential")
                vehicles.append([j, car_locations[0], car_locations[1]])

                car_locations = self.spawn_vehicle(4, "residential")
                vehicles.append([j, car_locations[0], car_locations[1]])

            if j % 25 == 0:
                # every 15 seconds, spawn a car from city
                car_locations = self.spawn_vehicle(-1, "city")
                vehicles.append([j, car_locations[0], car_locations[1]])
            if j % 35 == 0:
                # every 15 seconds, spawn a car from retail
                car_locations = self.spawn_vehicle(-1, "retail")
                vehicles.append([j, car_locations[0], car_locations[1]])
            if j % 50 == 0:
                # every 20 seconds, spawn a car from industrial
                car_locations = self.spawn_vehicle(-1, "industrial")
                vehicles.append([j, car_locations[0], car_locations[1]])
            if j % 25 == 0:
                # every 4 seconds, spawn a car on the minor roads
                for k in range(0, len(traffic_densities.minor_entrances)):
                    car_locations = self.spawn_vehicle(k, "minor")
                    vehicles.append([j, car_locations[0], car_locations[1]])
            if j % 12 == 0:
                # every 4 seconds, spawn a car on the major roads
                for k in range(0,len(traffic_densities.main_entrances)):
                    car_locations = self.spawn_vehicle(k, "major")
                    vehicles.append([j, car_locations[0], car_locations[1]])

            if j%(max_timestep/progress_range) == 0:

                print(str(progress_count) + "% complete")
                progress_count = progress_count + progress_range
        vehicles = np.array(vehicles)
        vehicles = pd.DataFrame(vehicles)
        vehicles.columns = ["timestep", "start_loc", "end_loc"]
        vehicles.to_csv(output_dir, index=False)
        print("Saved to: " + output_dir)

    def generate_powerset(self, action_set):
        # This function generates the powerset of the action set. i.e. generates every combination of actions
        test_set = list(powerset(action_set))
        temp_set = []
        for item in test_set:
            if len(item) == 1:
                temp_set.append([item[0]])
            else:
                temp_set.append(list(item))
        return temp_set
    def generate_states(self):
        # Converting actionset into powerset
        actions_set = self.generate_powerset(traffic_densities.action_set)
        print(actions_set)
        # Generating network files
        i = 0
        for action in actions_set:
            # for each action, generate a new network
            if not action:
                # This is the empty set
                if not os.path.isdir(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + "0"):
                    os.makedirs(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + "0")
                subprocess.run(["netconvert", "--sumo-net-file=" + dir_path + os.sep + "Networks" + os.sep + "Original" + os.sep + "osm.net.xml.gz",
                                "--output-file=" + dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + "0" + os.sep + "osm.net.xml",
                                "--lefthand=True", "--no-warnings", "--no-turnarounds"],  stdout=subprocess.DEVNULL)
                generate_sumocfg(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + "0" + os.sep + "osm.sumocfg")
                i = i + 1
            else:
                # We aren't working in the empty set case
                # Check if the directory of the new file exists yet
                if not os.path.isdir(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(i)):
                    os.makedirs(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(i))
                # prepare action set
                final_actions = ""
                for this_action in action:
                    if final_actions == "":
                        final_actions = str(this_action) + ", -" + str(this_action)
                    else:
                        final_actions = final_actions + "," + str(this_action) + ", -" + str(this_action)
                subprocess.run(["netconvert", "--sumo-net-file=" + dir_path + os.sep + "Networks" + os.sep + "Original" + os.sep + "osm.net.xml.gz",
                                "--remove-edges.explicit", "" + final_actions + "",
                                "--output-file=" + dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(i) + os.sep + "osm.net.xml",
                                "--lefthand=True", "--no-warnings", "--no-turnarounds"], stdout=subprocess.DEVNULL)
                generate_sumocfg(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(i) + os.sep + "osm.sumocfg")

                completeness = i/len(actions_set)
                print(str(completeness * 100) + ", or " + str(i) + " out of " + str(len(actions_set)))
                i = i + 1

    def run_crude(self):
        action_set = self.generate_powerset(traffic_densities.action_set)
        print(len(action_set))
        num_of_trials = len(action_set)
        air_pollution = []
        for i in range(0, num_of_trials):
            # Run network i
            air_pollution.append(self.runState(i))
            self.total_pollution = []

        print(air_pollution)





def powerset(s):
    s = list(s)
    return chain.from_iterable(combinations(s, r) for r in range(len(s) + 1))

def generate_sumocfg(output_path):
    # Create the root element for the sumocfg XML
    root = ET.Element("configuration")

    # Add the input section to the configuration
    input_section = ET.SubElement(root, "input")
    ET.SubElement(input_section, "net-file", {"value": "osm.net.xml"})

    processing_section = ET.SubElement(root, "processing")
    ET.SubElement(processing_section, "ignore-route-errors", {"value": "true"})
    # Add the output section to the configuration
    report_section = ET.SubElement(root, "report")
    ET.SubElement(report_section, "verbose", {"value": "true"})

    # Create an ElementTree object from the root and write it to a file
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t", level=0)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


Manager = TrafficProblemManager(debug=False)
# Manager.runState("Original")
#Manager.generate_states()
Manager.run_crude()

