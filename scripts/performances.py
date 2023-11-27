#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  1 23:12:41 2023

@author: Salem
"""
import numpy as np
import matplotlib.pyplot as plt

# Chargement des données depuis le fichier txt
"""
err1 = np.loadtxt('test1.challenge2.txt')
err2 = np.loadtxt('test2.challenge2.txt')
err3 = np.loadtxt('test3.challenge2.txt')
"""
# Chargement des données depuis le fichier txt
err1 = np.loadtxt('test1.challenge1.txt')
err2 = np.loadtxt('test2.challenge1.txt')
 
# Tracé du graphique des valeurs de l'erreur en fonction de l'indice
"""
plt.plot(err1, label = "vitesse angulaire = 0.9 m/s")
plt.plot(err2, label = "vitesse angulaire = 0.3 m/s")
plt.plot(err3, label = "vitesse angulaire = 2.2 m/s")
"""
print("l'errreur moyenne pour le test1 est de :", np.mean(err1))
print("l'errreur moyenne pour le test2 est de :", np.mean(err2))
#print("l'errreur moyenne pour le test3 est de :", np.mean(err3))
plt.plot(err1, label = "test 1")
plt.plot(err2, label = "test 2")

plt.xlabel('Indice')
plt.ylabel('Erreur')
plt.legend()
plt.title('Erreur pour 2 vitesses angulaires différentes')
plt.show()

