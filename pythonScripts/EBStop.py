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
        self.delta, self.epsilon = 0.01,0.01
        self.beta = 1.1
        self.p = 1.1
        self.k = 1

    #Standard implementation of EBGStop algorithm
    def ebg_stop(self,elements, R):
        t = 1
        k = 0
        d = math.pow((t)*(t+1), -1)
        alpha, x,c = float(),float(),float()
        while((1+self.epsilon)*self.LB < (1-self.epsilon)*self.UB):
            xt = elements[t-1]
            t+=1
            if (t > math.floor(math.pow(self.beta,k))):
                k += 1
                alpha = math.floor(math.pow(self.beta,k))/math.floor(math.pow(self.beta,k-1))
                x = -alpha*math.log(d/3)
            sumXt = 1/t * (np.sum(elements))            
            sum = [math.pow(elements[i] - sumXt,2) for i in range(len(elements))]
            deviation = math.sqrt(1/t * (np.sum(sum)))
            c = (deviation*math.sqrt(2*x/t)) + (3*R*x)/t
            self.LB = max(self.LB, math.fabs(sumXt) - c)
            self.UB = min(self.UB, math.fabs(sumXt) + c)
        print(t)
        return np.sign(xt) * 1/2 * ((1+self.epsilon)*self.LB + (1-self.epsilon)*self.UB)

    #Implementazione riadattata dell'algoritmo EBGStop per il model checking
    def find_stop_value(self,elements, R):
        t = len(elements)
        d = math.pow((t)*(t+1), -1)
        alpha, x, c = float(),float(),float()
        xt = elements[-1]
        if (t > math.floor(math.pow(self.beta,self.k))):
            self.k += 1
            alpha = math.floor(math.pow(self.beta,self.k))/math.floor(math.pow(self.beta,self.k-1))
            x = -alpha*math.log(d/3)
        sumXt = 1/t * (np.sum(elements))            
        sum = [math.pow(elements[i] - sumXt,2) for i in range(len(elements))]
        deviation = math.sqrt(1/t * (np.sum(sum)))
        c = (deviation*math.sqrt(2*x/t)) + (3*R*x)/t
        print("Confronto valori ebstop: ", self.LB, math.fabs(sumXt) - c, self.UB, math.fabs(sumXt) + c)
        self.LB = max(self.LB, math.fabs(sumXt) - c)
        self.UB = min(self.UB, math.fabs(sumXt) + c)
        return ((1+self.epsilon)*self.LB >= (1-self.epsilon)*self.UB)


        
