#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
import os # Extra: Importar la libreria OS para leer el fichero
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

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
  plt.grid(True) # Extra: muestra la cuadrícula
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.show()
  # raw_input()
  plt.pause(0.01) # Extra: automático
  plt.clf()

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

# Extra: ajustar los valores de los thetas a los límites de las articulaciones
def adjustAngle(angle):
  if angle < 0:
    angle = 360 + angle
  else:
    angle = angle % 360
  return angle

# Extra: Obtener los valores desde el fichero de entrada
def getXandY(file):
  f = open(file, 'r')
  x = float(f.readline().split(':')[1])
  y = float(f.readline().split(':')[1])
  f.close()
  return x, y

# Extra: Obtener los valores desde el fichero de entrada
def getArticulations(file):
  f = open(file, 'r')
  articulations = []
  for line in f.readlines()[2:]:
    type = line.split(':')[0]
    if type != 'r' and type != 'p':
      sys.exit('Error: Tipo de articulación no reconocido')
    limits = line.split(':')[1].split(']')[0].split('[')[1].split(',')
    limits = [int(limits[0]), int(limits[1])]
    articulations.append([type, limits])

  f.close()

  # poner los valores en el orden correcto
  articulations.reverse()
  
  return articulations

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# valores articulares arbitrarios para la cinemática directa inicial
art = getArticulations(sys.argv[1])
# Extra: Obtener los valores desde el fichero de entrada
th = [0. for i in range(len(art))]
# Extra: Obtener los valores desde el fichero de entrada
a =[5. for i in range(len(art))]
# L = sum(a) # variable para representación gráfica
EPSILON = .01

plt.ion() # modo interactivo

# introducción del punto para la cinemática inversa
# Extra: Gestion de errores en la entrada de datos
if (len(sys.argv) == 2 and (sys.argv[1] == '-h' or sys.argv[1] == '--help')):
  sys.exit("Usage: python " + sys.argv[0] + " <file>")
elif len(sys.argv) == 1:
  sys.exit("Incorrect Usage: python " + sys.argv[0] + " [-h|--help] for more information.")
elif len(sys.argv) > 2:
  print("Error: number of arguments is incorrect")
  sys.exit("Use -h or --help for more information")
elif not os.path.isfile(sys.argv[1]) or sys.argv[1].split('.')[1] != 'txt':
  sys.exit("Error: file does not exist or is not a .txt file")
else:
  objetivo = getXandY(sys.argv[1])


size = sqrt(objetivo[0]**2 + objetivo[1]**2) 
L = max(sum(a), size) # variable para representación gráfica


O=list(range(len(th)+1)) # Reservamos estructura en memoria
O[0]=cin_dir(th,a) # Calculamos la posicion inicial
print ("- Posicion inicial:")
muestra_origenes(O[0])

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  # Para cada combinación de articulaciones:
  for i in range(len(th)):
    # cálculo de la cinemática inversa:
    if art[i][0] == "r":
      # con articulaciones de rotacion
      v1 = [O[i][len(th)][0] - O[i][len(th) - i - 1][0], O[i][len(th)][1] - O[i][len(th) - i -1][1]]
      v2 = [objetivo[0] - O[i][len(th) - i - 1][0], objetivo[1] - O[i][len(th) - i - 1][1]]
      alfa2 = atan2(v1[1], v1[0])
      alfa1 = atan2(v2[1], v2[0])
      
      th[len(th) - i - 1] = adjustAngle(th[len(th) - i - 1])
      
      newAngle = th[len(th) - i - 1] + alfa1 - alfa2

      # normalizar el angulo
      newAngle = adjustAngle(newAngle)

      # angulo dentro de los limites
      if newAngle > art[i][1][0] and newAngle < art[i][1][1]:
        th[len(th) - i - 1] = newAngle
      elif newAngle > art[i][1][1]:
        th[len(th) - i - 1] = art[i][1][1]
      else:
        th[len(th) - i - 1] = art[i][1][0]

    elif art[i][0] == "p":
      # con articulaciones prismaticas
      w = sum(th[:i])
      vectorW = [cos(w), sin(w)]
      vectorP = [objetivo[0] - O[i][len(th) - i - 1][0], objetivo[1] - O[i][len(th) - i - 1][1]]
      distance = np.dot(vectorW, vectorP) + a[len(th) - i - 2]

      # distancia minimizada
      if distance > art[i][1][0] and distance < art[i][1][1]:
        a[len(th) - i - 1] = distance
      elif distance > art[i][1][1]:
        a[len(th) - i - 1] = art[i][1][1]
      else:
        a[len(th) - i - 1] = art[i][1][0]

    O[i+1] = cin_dir(th,a)

  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print("\n- Iteracion " + str(iteracion) + ':')
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print("- Umbral de convergencia epsilon: " + str(EPSILON))
print("- Distancia al objetivo:          " + str(round(dist,5)))
print("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print("  L" + str(i+1) + "     = " + str(round(a[i],3)))
