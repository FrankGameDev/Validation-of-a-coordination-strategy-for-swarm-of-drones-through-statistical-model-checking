import matplotlib.pyplot as plt
import numpy as np
import json,ast, math
import os
import time

path = "/home/francesco/Scrivania/Drones/Simulation_Data/"
os.system("mkdir " + path + "/graphs/")
path += "graphs/"

os.system("mkdir " + path + "/noFaultGraphs")
os.system("mkdir " + path + "/FaultGraphs")

#Estraggo da una simulazione il tempo di arrivo medio
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

#Estrae dai dati della simulazione l'incidenza media delle fault
def extract_fault_average(file):
    with open(file, "r") as f: 
        data = json.load(f)
        f.flush()
        f.close()
    faults = data["droneFault"]
    sommaFault = 0
    for simulation in faults.values():
        for drone in simulation:
            for ff in simulation[drone]:
                if(ff != 1.0): sommaFault += 1
    return sommaFault/len(list(faults.keys()))

def extract_averege_variance_times(file):
    with open(file, "r") as f: 
        data = json.load(f)
        f.flush()
        f.close()
    times = data["arrivalTime"]
    somma = 0
    media = []
    varianza = []
    totSim = len(data["arrived"])
    for d in times:
        numVarianza = 0
        somma = np.sum(times[d])
        tmpMedia = somma / totSim
        media.append(tmpMedia)
        for x in times[d]:
            numVarianza += math.pow(x-tmpMedia, 2)
        varianza.append(numVarianza/totSim)
    mediaTot = np.sum(media)/len(times.keys())
    varianzaTot = np.sum(varianza)/len(times.keys())
    return (mediaTot, varianzaTot)
    

#Crea un grafico che confronta lo stesso kpi di 3 simulazioni differenti
def compare_average_plot(titolo, graphtitle,xlabel, ylabel, ylimit,colore, nome1, kpi1, nome2, kpi2, nome3, kpi3, fault = False):
    fig = plt.figure()
    plt.autoscale()
    names = [nome1,nome2,nome3]
    kpis = [kpi1,kpi2,kpi3]
    plt.bar(names,kpis, color = colore, edgecolor = "black", width = 0.25)
    plt.grid(color = "grey", linestyle ="-", linewidth = 0.5, alpha = 0.2)
    plt.title(graphtitle)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.ylim(0,ylimit)
    if(fault): save_path = path + "/FaultGraphs/" + titolo +".png"
    else: save_path = path + "/noFaultGraphs/" + titolo +".png"
    plt.savefig(save_path, bbox_inches='tight')
    plt.clf()

#Crea una grafico della cumulata dei tempi di arrivo
def cumulated_plot(titolo, xlabel, ylabel, cumulatedTimes, graphtitle = "", fault = False):
    fig = plt.figure()
    # plt.autoscale() 
    times = list(cumulatedTimes.keys())
    num = list(cumulatedTimes.values())
    plt.grid(color = "grey", linestyle ="-", linewidth = 0.5, alpha = 0.2)
    plt.bar(times, num, edgecolor = "black")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(graphtitle)
    if(max(times) > 50 and max(times) < 100):
        plt.xticks(np.arange(0, max(times), 5))
    elif(max(times) >= 100):
        plt.xticks(np.arange(0, max(times), 10))
    else: plt.xticks(times)
    if(fault): save_path = path + "/FaultGraphs/" + titolo +".png"
    else: save_path = path + "/noFaultGraphs/" + titolo +".png"
    plt.savefig(save_path, bbox_inches='tight')
    plt.clf()


def plot_faults(titolo, xlabel, ylabel, l1,l2,l3):
    barWidth = 0.15
    fig = plt.subplots(figsize =(12, 8))
    
    scenari = ["4 droni", "10 droni", "20 droni"]
    br1 = np.arange(len(l1))
    br2 = [x + barWidth for x in br1]
    br3 = [x + barWidth for x in br2]
    plt.bar(br1, l1, color ='r', width = barWidth,
            edgecolor ='black', label ='4 droni')
    plt.bar(br2, l2, color ='g', width = barWidth,
            edgecolor ='black', label ='10 droni')
    plt.bar(br3, l3, color ='b', width = barWidth,
            edgecolor ='black', label ='20 droni')
    plt.grid(color = "grey", linestyle ="-", linewidth = 0.5, alpha = 0.2)
    plt.legend(labels = ["100 m", "150 m", "200 m"],loc = "upper left")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.xticks([r + barWidth for r in range(len(l1))], scenari)
    plt.savefig(path+titolo+'.png', bbox_inches='tight')

def plot_times_average_variance(titolo, graphtitle,xlabel, ylabel, colore, nome1, kpi1, nome2, kpi2, nome3, kpi3, fault = False):
    fig = plt.figure()
    plt.autoscale()
    names = [nome1,nome2,nome3]
    kpis = [kpi1,kpi2,kpi3]
    plt.bar(names,kpis, color = colore, edgecolor = "black", width = 0.25)
    plt.grid(color = "grey", linestyle ="-", linewidth = 0.5, alpha = 0.2)
    plt.title(graphtitle)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if(fault): save_path = path + "/FaultGraphs/" + titolo +".png"
    else: save_path = path + "/noFaultGraphs/" + titolo +".png"
    plt.savefig(save_path, bbox_inches='tight')
    plt.clf()

"""
Colori grafici: 
- Coll d-to-d: arancione
- Coll d-to-o: rosso
- arrivati: verde
- tempo di arrivo: blue

"""


print("Genero grafici per scenario 4 droni - no fault...")
#Simulazione 4 droni no fault
kpi4D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_200.json")
compare_average_plot("Collisioni drone-to-drone_scenario_4_droni_no", "4 droni, No fault","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 4,
"purple","100", kpi4D[0]["collDD"], "150", kpi4D[1]["collDD"], "200", kpi4D[2]["collDD"])
compare_average_plot("Collisioni drone-to-obstacles_scenario_4_droni_no", "4 droni, No fault", "Dimensione area di volo(n, n, n)", "Incidenza collisioni", 4,
 "orange","100", kpi4D[0]["collDO"], "150", kpi4D[1]["collDO"], "200", kpi4D[2]["collDO"])
compare_average_plot("Droni arrivati_scenario_4_droni_no", "4 droni, No fault","Dimensione area di volo(n, n, n)", "Droni arrivati", 4,
"green","100", kpi4D[0]["arrived"], "150", kpi4D[1]["arrived"], "200", kpi4D[2]["arrived"])
#Estraggo tempo medio e varianza
(tm4100, v4100) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_100.json")
(tm4150, v4150) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_150.json")
(tm4200, v4200) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_200.json")
plot_times_average_variance("Tempo_medio_scenario_4_droni", "4 droni, no fault", "Dimensione area di volo(n,n,n)", "Tempo medio", "b", 
"100", tm4100, "150", tm4150, "200", tm4200)
#Estraggo la cumulata dei tempi di arrivo
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_100", "Tempi di arrivo", "Numero droni", cumulatedTimes,"4 droni, 100 metri cubi,No fault")
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_150.json",)
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_150", "Tempi di arrivo", "Numero droni", cumulatedTimes,"4 droni, 150 metri cubi,No fault")
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_no_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_200", "Tempi di arrivo", "Numero droni", cumulatedTimes,"4 droni, 200 metri cubi,No fault")
print("OK")

print("Genero grafici per scenario 10 droni - no fault...")
#Simulazione 10 droni no fault
kpi10D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_200.json")
compare_average_plot("Collisioni drone-to-drone_scenario_10_droni_no", "10 droni, No fault", "Dimensione area di volo(n, n, n)", "Incidenza collisioni", 10,
"purple","100", kpi10D[0]["collDD"], "150", kpi10D[1]["collDD"], "200", kpi10D[2]["collDD"])
compare_average_plot("Collisioni drone-to-obstacles_scenario_10_droni_no", "10 droni, No fault","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 10,
 "orange","100", kpi10D[0]["collDO"], "150", kpi10D[1]["collDO"], "200", kpi10D[2]["collDO"])
compare_average_plot("Droni arrivati_scenario_10_droni_no", "10 droni, No fault", "Dimensione area di volo(n, n, n)", "Droni arrivati", 10,
"green","100", kpi10D[0]["arrived"], "150", kpi10D[1]["arrived"], "200", kpi10D[2]["arrived"])
#Estraggo tempo medio e varianza
(tm10100, v10100) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_100.json")
(tm10150, v10150) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_150.json")
(tm10200, v10200) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_200.json")
plot_times_average_variance("Tempo_medio_scenario_10_droni", "10 droni, no fault", "Dimensione area di volo(n,n,n)", "Tempo medio", "b", 
"100", tm10100, "150", tm10150, "200", tm10200)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_100", "Tempi di arrivo", "Numero droni", cumulatedTimes,"10 droni, 100 metri cubi,No fault")
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_150", "Tempi di arrivo", "Numero droni", cumulatedTimes, "10 droni, 150 metri cubi,No fault")
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_no_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_200", "Tempi di arrivo", "Numero droni", cumulatedTimes, "10 droni, 200 metri cubi,No fault")
print("OK")

print("Genero grafici per scenari 20 droni - no fault...")
#Simulazione 20 droni no fault
kpi20D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_200.json")
compare_average_plot("Collisioni drone-to-drone_scenario_20_droni_no", "20 droni, No fault","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 20,
"purple","100", kpi20D[0]["collDD"], "150", kpi20D[1]["collDD"], "200", kpi20D[2]["collDD"])
compare_average_plot("Collisioni drone-to-obstacles_scenario_20_droni_no","20 droni, No fault","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 20,
 "orange","100", kpi20D[0]["collDO"], "150", kpi20D[1]["collDO"], "200", kpi20D[2]["collDO"])
compare_average_plot("Droni arrivati_scenario_20_droni_no", "20 droni, No fault", "Dimensione area di volo(n, n, n)", "Droni arrivati", 20,
"green","100", kpi20D[0]["arrived"], "150", kpi20D[1]["arrived"], "200", kpi20D[2]["arrived"])
#Estraggo tempo medio e varianza
(tm20100, v20100) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_100.json")
(tm20150, v20150) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_150.json")
(tm20200, v20200) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_200.json")
plot_times_average_variance("Tempo_medio_scenario_20_droni", "20 droni, no fault", "Dimensione area di volo(n,n,n)", "Tempo medio", "b", 
"100", tm20100, "150", tm20150, "200", tm20200)
#Estraggo la cumulata dei tempi di arrivo
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_100", "Tempi di arrivo", "Numero droni", cumulatedTimes, "20 droni, 100 metri cubi,No fault")
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_150", "Tempi di arrivo", "Numero droni", cumulatedTimes, "20 droni, 150 metri cubi,No fault")
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_no_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_200", "Tempi di arrivo", "Numero droni", cumulatedTimes, "20 droni, 200 metri cubi,No fault")
print("OK")

print("Genero grafici per scenari 4 droni - fault attive...")
#Simulazione 4 droni fault
kpi4D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_200.json")
compare_average_plot("Collisioni drone-to-drone_scenario_4_droni_si", "4 droni, fault attive","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 4,
"purple","100", kpi4D[0]["collDD"], "150", kpi4D[1]["collDD"], "200", kpi4D[2]["collDD"], True)
compare_average_plot("Collisioni drone-to-obstacles_scenario_4_droni_si","4 droni, fault attive","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 4,
 "orange","100", kpi4D[0]["collDO"], "150", kpi4D[1]["collDO"], "200", kpi4D[2]["collDO"], True)
compare_average_plot("Droni arrivati_scenario_4_droni_si","4 droni, fault attive", "Dimensione area di volo(n, n, n)", "Droni arrivati", 4,
"green","100", kpi4D[0]["arrived"], "150", kpi4D[1]["arrived"], "200", kpi4D[2]["arrived"], True)
#Estraggo tempo medio e varianza
(tm4100, v4100) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_100.json")
(tm4150, v4150) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_150.json")
(tm4200, v4200) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_200.json")
plot_times_average_variance("Tempo_medio_scenario_4_droni_si", "4 droni, fault attive", "Dimensione area di volo(n,n,n)", "Tempo medio",  "b", 
"100", tm4100, "150", tm4150, "200", tm4200, True)
#Estraggo la cumulata dei tempi di arrivo
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_100_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "4 droni, 100 metri cubi, fault attive",True)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_150_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "4 droni, 150 metri cubi, fault attive", True)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_4_droni_200_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "4 droni, 200 metri cubi, fault attive", True)
print("OK")

print("Genero grafici per scenario 10 droni - fault attive...")
#Simulazione 10 droni no fault
kpi10D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_200.json")
compare_average_plot("Collisioni drone-to-drone_scenario_10_droni_si", "10 droni, fault attive","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 10,
"purple","100", kpi10D[0]["collDD"], "150", kpi10D[1]["collDD"], "200", kpi10D[2]["collDD"], True)
compare_average_plot("Collisioni drone-to-obstacles_scenario_10_droni_si","10 droni, fault attive","Dimensione area di volo(n, n, n)", "Incidenza collisioni", 10,
 "orange","100", kpi10D[0]["collDO"], "150", kpi10D[1]["collDO"], "200", kpi10D[2]["collDO"], True)
compare_average_plot("Droni arrivati_scenario_10_droni_si", "10 droni, fault attive","Dimensione area di volo(n, n, n)", "Droni arrivati", 10,
"green","100", kpi10D[0]["arrived"], "150", kpi10D[1]["arrived"], "200", kpi10D[2]["arrived"], True)
#Estraggo tempo medio e varianza
(tm10100, v10100) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_100.json")
(tm10150, v10150) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_150.json")
(tm10200, v10200) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_200.json")
plot_times_average_variance("Tempo_medio_scenario_10_droni_si", "10 droni, fault attive", "Dimensione area di volo(n,n,n)", "Tempo medio", "b", 
"100", tm10100, "150", tm10150, "200", tm10200, True)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_100_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "10 droni, 100 metri cubi, fault attive",True)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_150_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "10 droni, 150 metri cubi, fault attive",True)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_10_droni_200_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "10 droni, 200 metri cubi, fault attive",True)
print("OK")

print("Genero grafici per scenari 20 droni - fault attive...")
#Simulazione 20 droni no fault
kpi20D = extractAverages("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_100.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_150.json",
"/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_200.json")
compare_average_plot("Collisioni drone-to-drone_scenario_20_droni_si","20 droni, fault attive", "Dimensione area di volo(n, n, n)", "Incidenza collisioni", 20,
"purple","100", kpi20D[0]["collDD"], "150", kpi20D[1]["collDD"], "200", kpi20D[2]["collDD"], True)
compare_average_plot("Collisioni drone-to-obstacles_scenario_20_droni_si","20 droni, fault attive", "Dimensione area di volo(n, n, n)", "Incidenza collisioni", 20,
 "orange","100", kpi20D[0]["collDO"], "150", kpi20D[1]["collDO"], "200", kpi20D[2]["collDO"], True)
compare_average_plot("Droni arrivati_scenario_20_droni_si", "20 droni, fault attive","Dimensione area di volo(n, n, n)", "Droni arrivati", 20,
"green","100", kpi20D[0]["arrived"], "150", kpi20D[1]["arrived"], "200", kpi20D[2]["arrived"], True)
#Estraggo tempo medio e varianza
(tm20100, v20100) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_100.json")
(tm20150, v20150) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_150.json")
(tm20200, v20200) = extract_averege_variance_times("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_200.json")
plot_times_average_variance("Tempo_medio_scenario_20_droni_si", "20 droni, fault attive", "Dimensione area di volo(n,n,n)", "Tempo medio",  "b", 
"100", tm20100, "150", tm20150, "200", tm20200, True)
#Estraggo la cumulata dei tempi di arrivo
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_100.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_100_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "20 droni, 100 metri cubi, fault attive",True)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_150.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_150_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "20 droni, 150 metri cubi, fault attive",True)
cumulatedTimes = extractArrivalTimes("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_200.json")
cumulated_plot("Cumulata tempo di arrivo_scenario_20_droni_200_si", "Tempi di arrivo", "Numero droni", cumulatedTimes, "20 droni, 200 metri cubi, fault attive",True)
print("OK")


#Estraggo i valori medi delle fault per ogni simulazione degli scenari con fault attivi
averageFault_4_100 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_100.json")
averageFault_4_150 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_150.json")
averageFault_4_200 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_4_10_10_8_si_200.json")
averageFault_10_100 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_100.json")
averageFault_10_150 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_150.json")
averageFault_10_200 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_10_10_10_8_si_200.json")
averageFault_20_100 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_100.json")
averageFault_20_150 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_150.json")
averageFault_20_200 = extract_fault_average("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_20_10_10_8_si_200.json")
l_100_fault = [averageFault_4_100, averageFault_10_100, averageFault_20_100]
l_150_fault = [averageFault_4_150, averageFault_10_150, averageFault_20_150]
l_200_fault = [averageFault_4_200, averageFault_10_200, averageFault_20_200]
plot_faults("average_faults_compare", "Scenari", "Media fault", l_100_fault, l_150_fault, l_200_fault)