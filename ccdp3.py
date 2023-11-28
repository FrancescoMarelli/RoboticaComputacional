  #! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs
import os # Extra: Importar la libreria OS para leer el fichero

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.grid(True)
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.show()
  plt.pause(0.1)  # Para que se vea la animación
  plt.clf() # Limpia la pantalla

  

def matriz_T(d,th,a,al):
  # Calcula la matriz T (ángulos de entrada en grados)
  
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

def acotaAngulo(angulo):
    # Se normaliza el ángulo para que esté entre 0 y 360
    if angulo < 0:
        angulo += 360
    else:
        angulo = angulo % (360)
    return angulo

def getCoordenadas(file):
    # Se obtienen las coordenadas del objetivo (primeras dos líneas del fichero)
    with open(file, 'r') as f:
        x = float(f.readline().split()[1])
        y = float(f.readline().split()[1])
    return x, y

def getArticulaciones(file):
    with open(file, 'r') as f:
        articulations = []
        for line in f.readlines()[2:]:
            tokens = line.split() # separar por espacios
            type = tokens[0] # tipo de articulación
            if type != 'r' and type != 'p':
                sys.exit('Error: la articulación tiene que ser de tipo p o r')
            limits = [float(val) for val in tokens[1:]] # límites de la articulación
            articulations.append({'type': type, 'limits': limits}) # añadir articulación a la lista

        articulations.reverse()

    return articulations

def calculaAngulo(articulaciones, O, th, i, objetivo, acotaAngulo):
    # Calcular los vectores del último eslabon y del EF al punto objetivo
    lastArt = [O[i][len(th)][0] - O[i][len(th) - i - 1][0], O[i][len(th)][1] - O[i][len(th) -i -1][1]]
    EFtoPoint = [objetivo[0] - O[i][len(th) - i - 1][0], objetivo[1] - O[i][len(th) - i - 1][1]] 
    # Calcular los ángulos alfa1 y alfa2
    alfa1 = atan2(EFtoPoint[1], EFtoPoint[0])
    alfa2 = atan2(lastArt[1], lastArt[0])

    th[len(th) - i - 1] = acotaAngulo(th[len(th) - i - 1])
    # Calcular el nuevo ángulo de la articulación
    nuevoAngulo = th[len(th) - i - 1] + alfa1 - alfa2
    nuevoAngulo = acotaAngulo(nuevoAngulo)

    # Comprobar que el nuevo ángulo está dentro de los límites
    if articulaciones[i]['limits'][0] <= nuevoAngulo <= articulaciones[i]['limits'][1]:
        th[len(th) - i - 1] = nuevoAngulo
    elif nuevoAngulo < articulaciones[i]['limits'][0]:
        th[len(th) - i - 1] = articulaciones[i]['limits'][0]
    else:
        th[len(th) - i - 1] = articulaciones[i]['limits'][1]
    
    return th

def calculaDistancia(articulaciones, O, th, a, i, objetivo):
    w = sum(th[:i])
    vectorW = [cos(w), sin(w)]
    vectorP = [objetivo[0] - O[i][len(th) - i - 1][0], objetivo[1] - O[i][len(th) - i - 1][1]]
    distancia = np.dot(vectorW, vectorP) + a[len(th) - i - 2]

    # distancia minimizada
    if distancia > articulaciones[i]['limits'][0] and distancia < articulaciones[i]['limits'][1]:
        a[len(th) - i - 1] = distancia
    elif distancia >= articulaciones[i]['limits'][1]:
        a[len(th) - i - 1] = articulaciones[i]['limits'][1]
    else:
        a[len(th) - i - 1] = articulaciones[i]['limits'][0]
    
    return a


# ******************************************************************************
# control de errores en fichero de entrada
if len(sys.argv) != 2:
  sys.exit("python " + sys.argv[0] + " archivo")
elif len(sys.argv) == 2 and sys.argv[1] == "-h":
  sys.exit("python " + sys.argv[0] + " archivo")
elif len(sys.argv) == 2:
    filename = sys.argv[1]
    if not os.path.isfile(filename):
        sys.exit("Error: El archivo " + filename + " no existe en el directorio actual.")

# Cálculo de la cinemática inversa de forma iterativa por el método CCD
# introducción del punto para la cinemática inversa
articulaciones = getArticulaciones(sys.argv[1])
objetivo=getCoordenadas(sys.argv[1])

th = [0. for i in range(len(articulaciones))] # valores articulares arbitrarios para la cinemática directa inicial
a = [5. for i in range(len(articulaciones))] # valores articulares arbitrarios para la cinemática directa inicial

L = sum(a) # variable para representación gráfica
EPSILON = .1 # umbral de convergencia
plt.ion() # modo interactivo


O=cin_dir(th,a)
#O=zeros(len(th)+1) # Reservamos estructura en memoria
 # Calculamos la posicion inicial
print ("- Posicion inicial:")
muestra_origenes(O)

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  O=[cin_dir(th,a)]
  for i in range(len(th)):
      # cálculo de la cinemática inversa empezando por la última articulación 
      if articulaciones[i]['type'] == 'r': # rotacional
        # Calcular el nuevo ángulo de la articulación y devuelve lista de ángulos actualizados
        th = calculaAngulo(articulaciones, O, th, i, objetivo, acotaAngulo)

      else: # prismatico
        a = calculaDistancia(articulaciones, O, th, a, i, objetivo)

      O.append(cin_dir(th, a))



  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print ("\n- Iteracion " + str(iteracion) + ':')
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print ("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print ("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print ("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print ("- Umbral de convergencia epsilon: " + str(EPSILON))
print ("- Distancia al objetivo:          " + str(round(dist,5)))
print ("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print ("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))
