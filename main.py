# This is a project where I will test toy case traffic simulator

import math
import numpy as np
import pandas as pd
import TrafficProblem.traffic as TrafficProblemManager

class DesignerController:

    states = []
    actions = []
    rewards = []
    recommended_actions = []
    problem_type = ""
    statefiles = ""
    debug_mode = True


    def __init__(self, problem_type,  state_files, debug_mode):
        self.statefiles = state_files
        self.states.append("\\Networks\\Original\\")
        self.debug_mode = debug_mode
        self.problem_type = problem_type

        self.ClassController()


    def ClassController(self):
        finished = False
        while not finished:
            # Await an input action by the user
            next_action = input("Select next action")

            if str(next_action) == "exit" or str(next_action) == "q":
                # Exit the code. In the future, there could be code to save current state for later etc...
                finished = True
                print("Ending design episode")

            if "generate states" in str(next_action):





NewController = DesignerController("TrafficProblem", "./TrafficProblem/Networks/", True)