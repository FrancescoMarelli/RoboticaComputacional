# Practica 1

# @correo Profe bfarinaj@ull.edu.es
Manipulador 1

1) Primero paso
 Sentido rotación de articulación
 Punto -> Oi -> N+1 -> O0 ... On
 Art -> N

2) Definir sistemas de referencia en cada uno de los puntos
    SRi : 
         - Sistema destrogiro. pulgar z, indice x, y medio. 
          - ArtRot -> eje Z coincide con eje de giro. X e Y destroido.
          - ArtPris -> SRi == SRi-1
          - Xi-1 Perpendicular a Zi-1
          - Xi Perpendicular Zi-1

3) Aplicare tabla de D-H

|      |   1 |   2  |
|------|-----|------|       
|   d  | 0   |   0  |
|   θ  |+θ1  |  +θ2 |
|   a  | +10 |   +5 |
|   α  | 0   |   0  | 


A la hora de establecer Theta siempre mirar como estan orientadas las articulaciones en reposo. es decir al limite minimo digamos. 

Definir (O1)1 -> (O1)0 <- T10
        (O2)2 -> (O2)1 <- T21

Matrices T
    (O1)0 = T10 * (O1)1
    T20 = T21 x T10
    (O2)0 = T20*(O2)2

Manipulador 2

|     | 1   |   2   |   3 |
|-----|-----|-------|------|
|  d  | 5   |   0   |   0 |
|  θ  |+θ1  |  0    |  +θ3|
|   a | 0   |  l2   |   2 |
|  α  | 0   |  pi/2 |   0 |  

Definir (O1)1 -> (O1)0 <- T10
        (O2)2 -> (O2)1 <- T21
        (O3)3 -> (O3)2 <- T32

Matrices T
    (O1)0 = T10 * (O1)1
    T20 = T21 x T10
    T30 = T32 x T20
    (O3)0 = T30 * (O3)3

Manipulador 3

|     |  1   |   2   |  3   |    4  |    5.1 |   5.2 
|-----|------|-------|------|-------|--------|------
|  d  |      |       |      |       |        |
|  θ  |      |       |      |       |        |
|  a  |      |       |      |       |        |
|  α  |      |       |      |       |        |

Definir (O1)1 -> (O1)0 <- T10
        (O2)2 -> (O2)1 <- T21
        (O3)3 -> (O3)2 <- T32
        (O4)4 -> (O4)3 <- T43
        (O5.1)4 -> (O5.1)4 <- T5.14
        (O5.2)4 -> (O5.2)4 <- T5.24


Matrices T
    (O1)0 = T10 * (O1)1
    (O2)0 = T20*(O2)2
    T20 = T21 x T10
    T30 = T32 x T21

    (O3)0 = T30 * (O3)3  



