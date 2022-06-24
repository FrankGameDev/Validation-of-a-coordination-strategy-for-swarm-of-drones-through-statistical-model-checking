import random
import sys
import math
import numpy as np

"""
Classe destinata all'utilizzo dell'algoritmo di stopping utile a determinare il numero di simulazioni necessarie ad ottenere
giusti risultati
"""
class EBStop:
    def __init__(self):
        self.LB = 0
        self.UB = math.inf
        self.delta, self.epsilon = 0.1,0.1
        self.beta = 1.1
        self.p = 1.1
        self.k = 0
        self.c = (self.delta*(self.p - 1))/self.p
        self.d = math.pow(1*(1+1), -1)

    #Implementazione riadattata dell'algoritmo EBGStop per il model checking
    def find_stop_value(self,elements, R):
        t = len(elements)
        alpha, x, = float(),float()
        xt = elements[-1]
        if (t > math.floor(math.pow(self.beta,self.k))):
            self.k += 1
            alpha = math.floor(math.pow(self.beta,self.k))/math.floor(math.pow(self.beta,self.k-1))
            self.d = math.pow(t*(t+1), -1)
            x = -alpha*math.log(self.d/3)
        sumXt = 1/t * (np.sum(elements))            
        sum = [math.pow(elements[i] - sumXt,2) for i in range(len(elements))]
        deviation = math.sqrt(1/t * (np.sum(sum)))
        self.c = (deviation*math.sqrt(2*x/t)) + (3*R*x)/t if (x > 0) else (3*R*x)/t
        # print("Confronto valori ebstop: ", self.LB, math.fabs(sumXt) - self.c, self.UB, math.fabs(sumXt) + self.c)
        self.LB = max(self.LB, math.fabs(math.fabs(sumXt) - self.c))
        self.UB = min(self.UB, math.fabs(sumXt) + self.c)
        return ((1+self.epsilon)*self.LB >= (1-self.epsilon)*self.UB)

