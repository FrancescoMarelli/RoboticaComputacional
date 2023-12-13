#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Rob�tica Computacional 
# Grado en Ingenier�a Inform�tica (Cuarto)
# Pr�ctica 5:
#     Simulaci�n de robots m�viles holon�micos y no holon�micos.

#localizacion.py

import sys
from math import *
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
# ******************************************************************************
# Declaraci�n de funciones

def distancia(a,b):
  # Distancia entre dos puntos (admite poses)
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Diferencia angular entre una pose y un punto objetivo 'p'
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def mostrar(objetivos,ideal,trayectoria):
  # Mostrar objetivos y trayectoria:
  #plt.ion() # modo interactivo
  # Fijar los bordes del gr�fico
  objT   = np.array(objetivos).T.tolist()
  trayT  = np.array(trayectoria).T.tolist()
  ideT   = np.array(ideal).T.tolist()
  bordes = [min(trayT[0]+objT[0]+ideT[0]),max(trayT[0]+objT[0]+ideT[0]),
            min(trayT[1]+objT[1]+ideT[1]),max(trayT[1]+objT[1]+ideT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])*.75
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar objetivos y trayectoria
  idealT = np.array(ideal).T.tolist()
  plt.plot(idealT[0],idealT[1],'-g')
  plt.plot(trayectoria[0][0],trayectoria[0][1],'or')
  r = radio * .1
  for p in trayectoria:
    plt.plot([p[0],p[0]+r*cos(p[2])],[p[1],p[1]+r*sin(p[2])],'-r')
    #plt.plot(p[0],p[1],'or')
  objT   = np.array(objetivos).T.tolist()
  plt.plot(objT[0],objT[1],'-.o')
  plt.show()
  input()
  plt.clf()

def localizacion(balizas, real, ideal, centro, radio, mostrar=0):
    # Inicializar la imagen y el incremento
    imagen = []
    incremento = 1  # Ajusta este valor según sea necesario

    # Inicializar la probabilidad anterior a un número muy grande
    probabilidad_anterior = float('inf')

    # Iterar sobre el rango de -radio a +radio
    for i in range(-radio, radio+1, incremento):
        for j in range(-radio, radio+1, incremento):
            # Colocar al robot ideal en (centroX + i, centroY + j)
            centroX, centroY = centro
            robot_ideal = (centroX + i, centroY + j)  # Asume que la orientación no es necesaria aquí

            # Calcular la probabilidad
            # Aquí necesitarás agregar tu propia lógica para calcular la probabilidad
            probabilidad = real.measurement_prob(ideal.sense(balizas), balizas)
            # Si la probabilidad es menor que la probabilidad anterior, actualizar la mejor pose
            if probabilidad < probabilidad_anterior:
                mejor_pose = robot_ideal
                probabilidad_anterior = probabilidad

            # Agregar la probabilidad a la imagen
            imagen.append(probabilidad)

    # Si mostrar es 1, imprimir la imagen
    if mostrar:
      #plt.ion() # modo interactivo
      plt.xlim(centro[0]-radio,centro[0]+radio)
      plt.ylim(centro[1]-radio,centro[1]+radio)
      imagen.reverse()
      plt.imshow(imagen,extent=[centro[0]-radio,centro[0]+radio,\
                                centro[1]-radio,centro[1]+radio])
      balT = np.array(balizas).T.tolist();
      plt.plot(balT[0],balT[1],'or',ms=10)
      plt.plot(ideal.x,ideal.y,'D',c='#ff00ff',ms=10,mew=2)
      plt.plot(real.x, real.y, 'D',c='#00ff00',ms=10,mew=2)
      plt.show()
      #input()
      plt.clf()

      # Devolver la imagen
      return imagen


# ******************************************************************************

# Definición del robot:
P_INICIAL = [0.,4.,0.] # Pose inicial (posición y orientacion) -x,y,orientación
V_LINEAL  = .7         # Velocidad lineal max  (m/s)
V_ANGULAR = 140.       # Velocidad angular max   (�/s)
FPS       = 10.        # Resoluci�n temporal (fps)

HOLONOMICO = 1
GIROPARADO = 0
LONGITUD   = .2 # caso triciclo

# Definición de trayectorias:
trayectorias = [
    [[1,3]],
    [[0,2],[4,2]],
    [[2,4],[4,0],[0,0]],
    [[2,4],[2,0],[0,2],[4,2]],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)],
    [[1,10]]
    ]

# Definición de los puntos objetivo:
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <indice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definici�n de constantes:
EPSILON = .1                # Umbral de distancia
V = V_LINEAL/FPS            # Metros por fotograma
W = V_ANGULAR*pi/(180*FPS)  # Radianes por fotograma

ideal = robot()
ideal.set_noise(0,0,.1)   # Ruido lineal / radial / de sensado
ideal.set(*P_INICIAL)     # operador 'splat'

real = robot()
real.set_noise(.01,.01,.1)  # Ruido lineal (de avance) / radial / de sensado
real.set(*P_INICIAL)

random.seed(0)
tray_ideal = [ideal.pose()]  # Trayectoria percibida
tray_real = [real.pose()]     # Trayectoria seguida (solamente accedemos por la grafica (linea roja), pero en algoritmo no podemos)

tiempo  = 0.
espacio = 0.
#random.seed(0)
random.seed(datetime.now())
for punto in objetivos:
  while distancia(tray_ideal[-1],punto) > EPSILON and len(tray_ideal) <= 1000:
    pose = ideal.pose()

    w = angulo_rel(pose,punto) # calcula incremento de angulo para llegar a ese punto con cinematica inversa
    if w > W:  w =  W
    if w < -W: w = -W
    v = distancia(pose,punto) # lo mismo, distancia entre punto y posicion ideal, velocidad de avance, siempre compara con limite  (noseque)
    if (v > V): v = V
    if (v < 0): v = 0

    if HOLONOMICO:
      if GIROPARADO and abs(w) > .01:
        v = 0
      ideal.move(w,v) # muevo ambos robots
      real.move(w,v)
    else:
      ideal.move_triciclo(w,v,LONGITUD)
      real.move_triciclo(w,v,LONGITUD)
    tray_ideal.append(ideal.pose()) # almaceno nuevas poses del robot para representarlos en las graficas 
    tray_real.append(real.pose())

    # comprobar si difieren posiciones de l real y ideal cada instante de tiempo o cada umbral cada x tiempo. 
    # si la diferencia no es suficiente comparada al umbral localizo sino no es necesario
    
    # comprobar si difieren posiciones de l real y ideal cada instante de tiempo o cada umbral cada x tiempo. 
    # si la diferencia no es suficiente comparada al umbral localizo sino no es necesario
    if distancia(tray_ideal[-1],tray_real[-1]) > EPSILON:
      localizacion(objetivos,real,ideal, tray_real[-1], 10, 1)

    espacio += v
    tiempo  += 1

if len(tray_ideal) > 1000:
  print ("<!> Trayectoria muy larga - puede que no se haya alcanzado la posicion final.")
print ("Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s")
print ("Distancia real al objetivo: "+\
    str(round(distancia(tray_real[-1],objetivos[-1]),3))+"m")
mostrar(objetivos,tray_ideal,tray_real)  # Representaci�n gr�fica

