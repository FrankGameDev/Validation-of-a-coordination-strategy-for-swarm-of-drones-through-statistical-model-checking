#!/usr/bin/env python
# -*- coding: utf-8 -*-a

import matplotlib.pyplot as plt
import numpy as np
import json,ast

#f is the file where the data are stored. It needs 4 keys: collDD, collDO, arrived, arrivalTime
def printPlot(file):
    with open(file, "r") as f: 
        data = json.load(f)
        f.flush()
        f.close()

    names = np.array(["collDD", "collDO", "arrived", "arrivalTime"])
    
    average_coll_DD = np.sum(data["collDD"]) / len(data["collDD"])
    average_coll_DO = np.sum(data["collDO"]) / len(data["collDO"])
    average_arrived = np.sum(data["arrived"]) / len(data["arrived"])
    average_time = np.sum(data["arrivalTime"]) / len(data["arrivalTime"])

    print(average_coll_DD,average_coll_DO, average_arrived, average_time)

    values = np.array([average_coll_DD,average_coll_DO, average_arrived, average_time])

    plt.bar(names,values)
    plt.show()

printPlot("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_5_1_1_no_100.json")