#!/usr/bin/python
# -*- coding: utf-8 -*-

from numpy import *
import matplotlib.pyplot as plt
import random
import sys
import math

	
#Valores de ejemplo, PROBAD TAMBIÉN CON OTROS DISTINTOS
alpha1=0.002 #influencia de la rotacion en la rotacion
alpha2=0.001 #influencia de la distancia en la rotacion
alpha3=0.001 #influencia de la distancia en la distancia
alpha4=0.002 #influencia de la rotacion en la distancia

class Pose:
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta
	def __str__(self):
		return "(%f,%f,%f)" % (self.x,self.y,self.theta) 	

#TODO: implementar la función. De momento devuelve siempre 0. CAMBIAR
def sample_normal_distribution(b):
	"""Devuelve una muestra de una gaussiana de media 0 y desviación típica b
    Argumentos:
      b (float): desviación típica
	"""
	return 0

#TODO: debe devolver la muestra. De momento devuelve siempre 0,0,0. CAMBIAR
def sample_motion_model(u_act, u_prev, x):
	"""Devuelve una muestra del modelo de movimiento.
	Argumentos:
	  u_act (Pose): posición estimada por la odometría en el instante actual
	  u_prev (Pose): posición estimada por la odometría en el instante anterior
	  x (Pose): posición a partir de la que generar la muestra    
	Devuelve
	  Pose: muestra del modelo de movimiento
	"""
	return Pose(0,0,0)

if len(sys.argv) <= 2:
	print "practica1 fich_odometria numero_de_muestras"
	exit()		

N = int(sys.argv[2])


odometria = []
odometria.append(Pose(0,0,0))

fichero = open(sys.argv[1],"r")
for linea in fichero:
	datos = linea.split(" ")
	odometria.append(Pose(float(datos[0]),
		                  float(datos[1]),
	                      math.radians(float(datos[2]))))					  

#inicialmente todas las muestras están en (0,0,0)
muestras = []
for i in range(0,N):
	muestras.append(Pose(0,0,0))

#pinta las muestras en el instante inicial
for i in range(0,N):
	plt.plot(muestras[i].x,muestras[i].y,"r.")	

#Para cada posición estimada por la odometría 
for it in range(1,len(odometria)):
	#Para cada muestra
	for index, obj in enumerate(muestras):
		#generamos una nueva muestra a partir de esa y sustituimos la antigua por la nueva
		muestras[index] = sample_motion_model(odometria[it], odometria[it-1], obj)
		plt.plot(muestras[index].x,muestras[index].y,"r.")	
plt.show()