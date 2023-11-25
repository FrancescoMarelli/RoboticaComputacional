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
  plt.figure()
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.pause(0.0001)
  plt.show()
  
#  input()
  plt.close()

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

# Devuelve True si la articulación 'i' es de revolución
def es_de_revolucion(theta):
    # Si theta es diferente de cero, la articulación es de revolución
    return theta != 0

def angulo_incluido(angulo):
    # Si el ángulo es mayor que 180º, se resta 360º
    if angulo < 0:
        angulo += 2*pi
    else:
        angulo = angulo % (2*pi)
    return angulo


# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# valores articulares arbitrarios para la cinemática directa inicial
th=[radians(360), 0, radians(360)]  # ángulos en radianes, 0 para la articulación prismática
a =[5., 5., 5.]  # longitudes, la longitud de la articulación prismática se actualizará durante el cálculo de la cinemática inversa
L = sum(a) # variable para representación gráfica
EPSILON = .01

#plt.ion() # modo interactivo

# introducción del punto para la cinemática inversa
if len(sys.argv) != 3:
  sys.exit("python " + sys.argv[0] + " x y")
objetivo=[float(i) for i in sys.argv[1:]]
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
      if es_de_revolucion(th[i]):
          # Solo realiza cálculos si la articulación es de revolución
          # Calculate the vectors
          V1 = [O[i][len(th)][0] - O[i][len(th) - i - 1][0], O[i][len(th)][1] - O[i][len(th) -i -1][1]]
          V2 = [objetivo[0] - O[i][len(th) - i - 1][0], objetivo[1] - O[i][len(th) - i - 1][1]]
          # Calculate the angle between V1 and V2
          alfa1 = atan2(V2[1], V2[0])
          alfa2 = atan2(V1[1], V1[0])
          # Check if the angle is between 0 and 2pi
          th[len(th) - i - 1] = angulo_incluido(th[len(th) - i - 1])
          # Calculate the new angle
          newAngle = th[len(th) - i - 1] + alfa1 - alfa2
          # Check if the angle is between 0 and 2pi
          th[len(th) - i - 1] = angulo_incluido(newAngle)
          # Update the angle
          th[len(th) - i - 1] = newAngle
      else:
          # Calcula el vector de la posición actual a la posición objetivo
          vector = np.subtract(O[i][-1], objetivo)
          # Calcula la norma (longitud) del vector manualmente
          dist_prismatica = np.sqrt(np.sum(np.square(vector)))
          # Actualiza a con la distancia prismática
          a[len(a) - i - 1] = dist_prismatica

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
