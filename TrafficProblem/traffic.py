import numpy as np
import os, sys
import time
import subprocess
import shutil
import pandas as pd
import multiprocessing
from queue import Queue
import tqdm
from itertools import chain, combinations
import Networks.Original.network_traffic as traffic_densities
import random
import xml.etree.ElementTree as ET
import socket

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import traci.constants as tc

import sumolib
import libsumo


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

    def runState(self, state):
        # runparameter = 1000
        # if state == "Original":
        #     state = os.sep + "Networks" + os.sep + "Original" + os.sep
        # else:
        #     state = os.sep + "Networks" + os.sep + "Modified" + os.sep + str(state) + os.sep
        #
        # libsumo.start(["sumo", "-c", dir_path + state + "osm.sumocfg", "--time-to-teleport=10000", "--start",
        #                "--quit-on-end", "--verbose=False", "--duration-log.disable=True", "--duration-log.statistics=False",
        #                "--no-step-log=True", "--threads=3","--step-length=0.5","--no-warnings=True"])
        #
        # for j in range(0, runparameter):
        #     libsumo.simulationStep()
        #     self.read_road_sensors()
        # libsumo.close()

        return 0
        # return sum(self.total_pollution)/500

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
                self.total_pollution.append(libsumo.edge.getCOEmission(sensor))
        else:
            i = 0
            for sensor in sensors:
                self.total_pollution[i] = self.total_pollution[i] + libsumo.edge.getCOEmission(sensor)
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
        if __name__ == '__main__':
            # Converting actionset into powerset
            # Create a multiprocessing pool
            multiprocessing.freeze_support()
            pool = multiprocessing.Pool(processes=multiprocessing.cpu_count())
            a= 0
            for result in tqdm.tqdm(pool.imap(generate_action, global_actions_set), total=len(global_actions_set)):
                a = a + 1
            pool.close()
            pool.join()

    def run_crude(self):

        i = 0
        # for j in range(0,4):
        p1 = multiprocessing.Process(target=self.runState, args=(0,))
        p1.start()
        p1.join()

        # if __name__ == '__main__':
        #     all_states = range(0,len(global_actions_set))
        #     task_queue = Queue()
        #     for trial in all_states:
        #         task_queue.put(trial)
        #     # Create a multiprocessing pool
        #     num_processes = 1
        #     workers = []
        #     for _ in range(num_processes):
        #         worker = multiprocessing.Process(target=self.simulation_worker,args=(task_queue,))
        #         worker.start()
        #         workers.append(worker)
        #
        #     task_queue.join()
        #     # Stop the worker processes
        #     for _ in range(num_processes):
        #         task_queue.put(None)  # Add termination signal to the task queue
        #
        #     for worker in workers:
        #         worker.join()

            #
            # result_list_tqdm = []
            # for result in tqdm.tqdm(pool.imap(self.runState, range(0,len(global_actions_set))), total=len(global_actions_set)):
            #     result_list_tqdm.append(result)
            # pool.close()
            # pool.join()
            # print(result_list_tqdm)
        # action_set = self.generate_powerset(traffic_densities.action_set)
        # num_of_trials = len(action_set)
        # air_pollution = []
        # for i in range(0, num_of_trials):
        #     start_time = time.time()
        #     # Run network i
        #     air_pollution.append(self.runState(i))
        #     self.total_pollution = []
        #     print("Iteration (" + str(i) + ") took " + str(time.time() - start_time) + " to run!")
        #
        # print(air_pollution)


def powerset(s):
    s = list(s)
    return chain.from_iterable(combinations(s, r) for r in range(len(s) + 1))

def generate_sumocfg(output_path):
    # Create the root element for the sumocfg XML
    root = ET.Element("configuration")

    # Add the input section to the configuration
    input_section = ET.SubElement(root, "input")
    ET.SubElement(input_section, "net-file", {"value": "osm.net.xml"})
    ET.SubElement(input_section, "route-files", {"value": "osm.rou.xml"})
    processing_section = ET.SubElement(root, "processing")
    ET.SubElement(processing_section, "ignore-route-errors", {"value": "true"})
    # Add the output section to the configuration
    report_section = ET.SubElement(root, "report")
    ET.SubElement(report_section, "verbose", {"value": "true"})

    # Create an ElementTree object from the root and write it to a file
    tree = ET.ElementTree(root)
    ET.indent(tree, space="\t", level=0)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)

def generate_routesxml(netfile, vehicles_file, output_path):
    net = sumolib.net.readNet(netfile)
    edge_ids = [edge.getID() for edge in net.getEdges()]
    vehicles = pd.read_csv(vehicles_file)

    # Create the root element for the sumocfg XML
    root = ET.Element("routes")
    # root2 = ET.Element("routes")
    counter = 0
    for index, vehicle in vehicles.iterrows():
        if vehicle['start_loc'] in edge_ids and vehicle['end_loc'] in edge_ids:
            # generate route
            route = net.getShortestPath(net.getEdge(vehicle['start_loc']), net.getEdge(vehicle['end_loc']))[0]
            if route:
                route = [edge.getID() for edge in route]
                route = " ".join(route)
                vehicle_section = ET.SubElement(root, "vehicle", {"id": str(counter), "depart": str(vehicle['timestep'])})
                route_section = ET.SubElement(vehicle_section, "route", {"edges": route})
                counter = counter + 1
                # trip_section = ET.SubElement(root2, "trip", {"id": str(index), "depart": str(vehicle['timestep']), "from": vehicle['start_loc'], "to": vehicle['end_loc']})

    # Create an ElementTree object from the root and write it to a file
    tree = ET.ElementTree(root)
    # tree2 = ET.ElementTree(root2)
    ET.indent(tree, space="\t", level=0)
    # ET.indent(tree2, space="\t", level=0)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    # tree2.write(trips_path, encoding="utf-8", xml_declaration=True)
def generate_action(action):

    if not action:
        if not os.path.isdir(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(0)):
            os.makedirs(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(0))
            # prepare action set
        subprocess.run(["netconvert",
                        "--sumo-net-file=" + dir_path + os.sep + "Networks" + os.sep + "Original" + os.sep + "osm.net.xml.gz",
                        "--output-file=" + dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(0) + os.sep + "osm.net.xml",
                        "--lefthand=True", "--no-warnings", "--no-turnarounds"], stdout=subprocess.DEVNULL)
        generate_routesxml(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(0) + os.sep + "osm.net.xml",
                           dir_path + os.sep + "vehicles.csv",
                           dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(0) + os.sep + "osm.rou.xml")
                           # dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(0) + os.sep + "osm.trips.xml")
        generate_sumocfg(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(0) + os.sep + "osm.sumocfg")


    else:

        index = global_actions_set.index(action)

        if not os.path.isdir(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(index)):
            os.makedirs(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(index))
        # prepare action set
        final_actions = ""
        for this_action in action:
            if final_actions == "":
                final_actions = str(this_action) + ", -" + str(this_action)
            else:
                final_actions = final_actions + "," + str(this_action) + ", -" + str(this_action)
        subprocess.run(["netconvert", "--sumo-net-file=" + dir_path + os.sep + "Networks" + os.sep + "Original" + os.sep + "osm.net.xml.gz",
                        "--remove-edges.explicit", "" + final_actions + "",
                        "--output-file=" + dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(index) + os.sep + "osm.net.xml",
                        "--lefthand=True", "--no-warnings", "--no-turnarounds"], stdout=subprocess.DEVNULL)
        generate_routesxml(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(index) + os.sep + "osm.net.xml",
                           dir_path + os.sep + "vehicles.csv",
                           dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(index) + os.sep + "osm.rou.xml")
                           # dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(index) + os.sep + "osm.trips.xml")
        generate_sumocfg(dir_path + os.sep + "Networks" + os.sep + "Modified" + os.sep + str(index) + os.sep + "osm.sumocfg")


Manager = TrafficProblemManager(debug=False)
global_actions_set = Manager.generate_powerset(traffic_densities.action_set)
def runState(state):
    runparameter = 1000
    if state == "Original":
        state = os.sep + "Networks" + os.sep + "Original" + os.sep
    else:
        state = os.sep + "Networks" + os.sep + "Modified" + os.sep + str(state) + os.sep

    libsumo.start(["sumo", "-c", dir_path + state + "osm.sumocfg", "--time-to-teleport=10000", "--start",
                   "--quit-on-end", "--verbose=False", "--duration-log.disable=True", "--duration-log.statistics=False",
                   "--no-step-log=True", "--threads=3","--step-length=0.5","--no-warnings=True"])
    current_pollution = []
    for j in range(0, runparameter):
        libsumo.simulationStep()
        current_pollution = read_road_sensors(current_pollution)
    libsumo.close()
    return sum(current_pollution)/500


def run_crude():
    if __name__ == '__main__':
        # Converting actionset into powerset
        # Create a multiprocessing pool
        multiprocessing.freeze_support()
        pool = multiprocessing.Pool(processes=multiprocessing.cpu_count())
        result_list_tqdm = []
        for result in tqdm.tqdm(pool.imap(runState, range(0, len(global_actions_set))), total=len(global_actions_set)):
            result_list_tqdm.append(result)
        pool.close()
        pool.join()
        print(result_list_tqdm)

        df = pd.DataFrame(result_list_tqdm)
        df.columns = ["COEmissions"]
        df.to_csv(dir_path + os.sep + "Networks" + os.sep + "COEmissions.csv", index=True)



def read_road_sensors(pollution):
    sensors = traffic_densities.road_sensors

    if not pollution:
        for sensor in sensors:
            pollution.append(libsumo.edge.getCOEmission(sensor))
    else:
        i = 0
        for sensor in sensors:
            pollution[i] = pollution[i] + libsumo.edge.getCOEmission(sensor)
            i += 1
    return pollution

# Manager.runState(0)
# Manager.generate_states()
# Manager.run_crude()

# run_crude()

file = pd.read_csv(dir_path + os.sep + "Networks" + os.sep + "COEmissions.csv")
list = list(file['COEmissions'])

minimum = min(list)
print(minimum)
print(list.index(min(list)))
print(global_actions_set[list.index(min(list))])