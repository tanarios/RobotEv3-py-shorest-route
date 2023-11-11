#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import heapq


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


city_info = { #grafo de la ciudad, x e y son las coordenadas de la ciudad
    'inicio': {'x':0, 'y':0, 'conexion':{'bomberos':1}},
    'bomberos': {'x':0, 'y':1, 'conexion':{'inicio':1, 'museo':2, 'policia': 2, 'iglesia':1}},
    'museo': {'x':-1, 'y':1, 'conexion':{'bomberos':2, 'restaurante':1}},
    'restaurante': {'x':-1, 'y':2, 'conexion':{'museo':1, 'iglesia':3, 'gasolinera':1}},
    'gasolinera': {'x':-1, 'y':3, 'conexion':{'restaurante':1, 'parque':2, 'centro_comercial':1}},
    'centro_comercial': {'x':0, 'y':4, 'conexion':{'gasolinera':1, 'gym':1}},
    'gym': {'x':0, 'y':4, 'conexion':{'centro_comercial':1, 'parque':3, 'hotel':1}},
    'parque': {'x':0, 'y':3, 'conexion':{'gym':3, 'gasolinera':2, 'iglesia':3, 'colegio':3}},
    'iglesia': {'x':0, 'y':2, 'conexion':{'parque':3, 'restaurante':3, 'bomberos':1, 'municipalidad':2}},
    'municipalidad': {'x':1, 'y':2, 'conexion':{'iglesia':2, 'colegio':2, 'policia':2}},
    'policia': {'x':1, 'y':1, 'conexion':{'municipalidad':2, 'bomberos':2}},
    'hotel': {'x':1, 'y':4, 'conexion':{'gym':1, 'colegio':1}},
    'colegio': {'x':1, 'y':3, 'conexion':{'hotel':1, 'parque':3, 'municipalidad':2}}
}

def dijkstra(city_info, inicio, final):
    cola_prioridad = [(0, inicio, ())] #lista para almacenar
    distancias = {punto: float('inf') for punto in city_info} #inicializar las distancias en infinito
    distancias[inicio] = 0 #distancia del inicio a el mismo es 0

    while cola_prioridad:
        distancia_actual, punto_actual, ruta_actual = heapq.heappop(cola_prioridad)
        ruta_actual = ruta_actual + [punto_actual]

        if distancia_actual > distancias[punto_actual]:
            continue
        
        for vecino, peso in city_info[punto_actual]['conexion'].items():
           distancia = distancia_actual + peso
           nueva_ruta = ruta_actual + [vecino] #agrega el punto actual a vecino

           if distancia<distancias[vecino]:
            distancias[vecino] = distancia
            heapq.heappush(cola_prioridad, (distancia, vecino, nueva_ruta))

        return distancias[final], nueva_ruta

#ahora bien
inicio = 'inicio'
final = 'policia'
#distancia_min, ruta= dijkstra(city_info, inicio, final)
#print("La distancia minima es: ", distancia_min)
#print("La ruta es: ", ruta)


# Create your objects here.
ev3 = EV3Brick()

# Write your program here.



# Initialize the motors.
left_motor = Motor(Port.A) 
right_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.S1)
while color_sensor.color() != Color.WHITE:
    ev3.speaker.beep()


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

def seguir_linea():
    #este metodo sigue la linea blanca del mapa hasta llegar al edificio
    while True:
        if color_sensor.color() == Color.WHITE:
            robot.turn(10)
        else:
            robot.drive(100, 0)


def mover_robot():
    #este metodo mueve el robot entre edificios
    
    #origen y destino son cordenadas x, y del mapa
    distancia_x = destino['x'] - origen['x']
    distancia_y = destino['y'] - origen['y']

    #calcular direccion y distancia entre edificios
    direccion = math.atan2(distancia_y, distancia_x) #angulo en radianes
    distancia = math.sqrt(distancia_x**2 + distancia_y**2) #distancia en cm

    #convertir el angulo a grados y ajustar direccion del robot
    direccion_grados = math.degrees(direccion)
    robot.on_for_degrees(velocidad=50, degrees=direccion_grados)
    
    distancia_totalcm = sqrt(distancia_x**2 + distancia_y**2)*100 #distancia en cm
    #quiero que el robot se mueva 20 cm por cada punto del grafo

    distancia_incremento = 20
    while distancia_totalcm >0:
        if distancia_totalcm <distancia_incremento:
            distancia_incremento = distancia_totalcm
    
    robot.on_for_seconds(velocidad=50, seconds=distancia_incremento/10)

    seguir_linea() #seguir la linea blanca hasta llegar al edificio
    distancia_totalcm -= distancia_incremento #actualizar la distancia total


    #mover el robot hacia el edificio
    robot.on_for_distance(velocidad=50, distance=distancia/10)


#entonces deberia moverse entre los puntos del dkistra
punto_inicio= city_info['inicio']
punto_final= city_info['hotel']

distancia_minima, ruta = dijkstra(city_info, punto_inicio, punto_final)
for i in range(len(ruta) -1):
    mover_robot(ruta[i], ruta[i+1])









