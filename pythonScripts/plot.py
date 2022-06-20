import matplotlib.pyplot as plt
import numpy as np
import json,ast
import os
import time

path = "/home/francesco/Scrivania/Drones/Simulation_Data"
os.system("mkdir " + path + "/graphs/")
path += "graphs/"


def extractArrivalTimes(file):
    with open(file, "r") as f: 
        data = json.load(f)
        f.flush()
        f.close()
    times = set()
    tmp = data["arrivalTime"]
    for d in tmp:
        times.update(tmp[d])
    timesCumulated = dict.fromkeys(times, 0)
    for d in tmp:
        for x in tmp[d]:
            timesCumulated[x] += 1
    if(-1 in timesCumulated.keys()): timesCumulated.pop(-1)
    # if(-1 in timesCumulated.keys()): print(timesCumulated[-1], np.sum(list(timesCumulated.values()))-timesCumulated[-1])
    return timesCumulated



#Prese 3 simulazioni, estraggo i valori medi dei KPI dati dalle 3 simulazioni
def extractAverages(file1,file2,file3):
    with open(file1, "r") as f: 
        data1 = json.load(f)
        f.flush()
        f.close()
    with open(file2, "r") as f: 
        data2 = json.load(f)
        f.flush()
        f.close()
    with open(file3, "r") as f: 
        data3 = json.load(f)
        f.flush()
        f.close()

    """
    Itero su i 3 file scelti e estraggo i valori medi dei KPI
    """
    lista = [data1,data2,data3]
    dizList = list()
    for i,d in enumerate(lista):
        #Calcolo la media delle collisioni tra droni, tra droni e ostacoli, la media del numero di droni arrivati 
        average_coll_DD = np.sum(d["collDD"]) / len(d["collDD"])
        average_coll_DO = np.sum(d["collDO"]) / len(d["collDO"])
        average_arrived = np.sum(d["arrived"]) / len(d["arrived"])
        
        media_droni = list()
        #calcolo media tempo di arrivo
        for x in d["arrivalTime"]:
            somma = 0
            for j in d["arrivalTime"][x]:
                if(j > 0.0): #se il drone è arrivato
                    somma += j
            media_droni.append(somma/len(d["arrivalTime"][x]))
        average_time = np.sum(media_droni) / len(media_droni)
        dizList.append({"collDD":average_coll_DD, "collDO":average_coll_DO, "arrived": average_arrived, "arrivalTime" : average_time})
    
    diz = dict()
    for k,l in enumerate(dizList):
        diz.update({k:l})
    # print(json.dumps(diz, indent=4))
    return diz

#Crea un grafico che confronta lo stesso kpi di 3 simulazioni differenti
def average_plot(titolo, xlabel, ylabel,colore, nome1, kpi1, nome2, kpi2, nome3, kpi3):
    fig = plt.figure()
    plt.autoscale()
    names = [nome1,nome2,nome3]
    kpis = [kpi1,kpi2,kpi3]
    plt.bar(names,kpis, color = colore, edgecolor = "black", width = 0.25)
    plt.grid(color = "grey", linestyle ="-", linewidth = 0.5, alpha = 0.2)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.savefig(path+titolo+'.png', bbox_inches='tight')
    plt.clf()

#Crea una grafico della cumulata dei tempi di arrivo
def cumulated_plot(titolo, xlabel, ylabel, cumulatedTimes):
    fig = plt.figure()
    # plt.autoscale() 
    times = list(cumulatedTimes.keys())
    num = list(cumulatedTimes.values())
    plt.grid(color = "grey", linestyle ="-", linewidth = 0.5, alpha = 0.2)
    plt.bar(times, num, edgecolor = "black")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if(max(times) > 50 and max(times) < 100):
        plt.xticks(np.arange(0, max(times), 5))
    elif(max(times) >= 100):
        plt.xticks(np.arange(0, max(times), 10))
    else: plt.xticks(times)
    plt.savefig(path+titolo+'.png', bbox_inches='tight')
    plt.clf()

"""
Colori grafici: 
- Coll d-to-d: arancione
- Coll d-to-o: rosso
- arrivati: verde
- tempo di arrivo: blue

"""


print("Genero grafici per scenari 4 droni - no fault...")
#Simulazione 4 droni no fault
kpi4D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_200.json")
average_plot("Collisioni drone-to-drone_scenario_4_droni_no", "Dimensione area di volo", "Media collisioni",
"purple","100", kpi4D[0]["collDD"], "150", kpi4D[1]["collDD"], "200", kpi4D[2]["collDD"])
average_plot("Collisioni drone-to-obstacles_scenario_4_droni_no","Dimensione area di volo", "Media collisioni",
 "orange","100", kpi4D[0]["collDO"], "150", kpi4D[1]["collDO"], "200", kpi4D[2]["collDO"])
average_plot("Droni arrivati_scenario_4_droni_no", "Dimensione area di volo", "Droni arrivati",
"green","100", kpi4D[0]["arrived"], "150", kpi4D[1]["arrived"], "200", kpi4D[2]["arrived"])
#Estraggo la cumulata dei tempi di arrivo
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_100", "Tempi di arrivo", "Numero droni", cumulatedTimes)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_150", "Tempi di arrivo", "Numero droni", cumulatedTimes)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_200", "Tempi di arrivo", "Numero droni", cumulatedTimes)
print("OK")

print("Genero grafici per scenario 10 droni - no fault...")
#Simulazione 10 droni no fault
kpi10D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_200.json")
average_plot("Collisioni drone-to-drone_scenario_10_droni_no", "Dimensione area di volo", "Media collisioni",
"purple","100", kpi10D[0]["collDD"], "150", kpi10D[1]["collDD"], "200", kpi10D[2]["collDD"])
average_plot("Collisioni drone-to-obstacles_scenario_10_droni_no","Dimensione area di volo", "Media collisioni",
 "orange","100", kpi10D[0]["collDO"], "150", kpi10D[1]["collDO"], "200", kpi10D[2]["collDO"])
average_plot("Droni arrivati_scenario_10_droni_no", "Dimensione area di volo", "Droni arrivati",
"green","100", kpi10D[0]["arrived"], "150", kpi10D[1]["arrived"], "200", kpi10D[2]["arrived"])
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_100", "Tempi di arrivo", "Numero droni", cumulatedTimes)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_150", "Tempi di arrivo", "Numero droni", cumulatedTimes)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_200", "Tempi di arrivo", "Numero droni", cumulatedTimes)
print("OK")

print("Genero grafici per scenari 20 droni - no fault...")
#Simulazione 20 droni no fault
kpi20D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_200.json")
average_plot("Collisioni drone-to-drone_scenario_20_droni_no", "Dimensione area di volo", "Media collisioni",
"purple","100", kpi20D[0]["collDD"], "150", kpi20D[1]["collDD"], "200", kpi20D[2]["collDD"])
average_plot("Collisioni drone-to-obstacles_scenario_20_droni_no","Dimensione area di volo", "Media collisioni",
 "orange","100", kpi20D[0]["collDO"], "150", kpi20D[1]["collDO"], "200", kpi20D[2]["collDO"])
average_plot("Droni arrivati_scenario_20_droni_no", "Dimensione area di volo", "Droni arrivati",
"green","100", kpi20D[0]["arrived"], "150", kpi20D[1]["arrived"], "200", kpi20D[2]["arrived"])
#Estraggo la cumulata dei tempi di arrivo
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_100", "Tempi di arrivo", "Numero droni", cumulatedTimes)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_150", "Tempi di arrivo", "Numero droni", cumulatedTimes)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_200", "Tempi di arrivo", "Numero droni", cumulatedTimes)
print("OK")
