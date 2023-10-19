# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1636442'
__group__ = 'DM.12'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2022 - 2023
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    llista = []
    Ultim = path.last
    dicc = map.connections[Ultim]
    for connections in dicc:
        caminou = copy.deepcopy(path)
        caminou.add_route(connections)
        llista.append(caminou)

    return llista

def remove_cycles(path_list):
    llista = copy.deepcopy(path_list)
    for path in path_list:
        trobat = 0
        for j in range(len(path.route)):
            for z in range(len(path.route)):
                if j != z:
                    if path.route[j] == path.route[z]:
                        trobat = 1
        if trobat == 1:
            llista.remove(path)
    return llista

def insert_depth_first_search(expand_paths, list_of_path):
    if len(list_of_path) != 0:
        expand_paths += list_of_path
    return expand_paths

def depth_first_search(origin_id, destination_id, map):
    n1 = Path(origin_id)
    cami = [n1]

    if cami[0].last == destination_id:
        return cami

    while cami[0].last != destination_id or not cami:
        expand_camins = copy.deepcopy(expand(cami[0], map))
        removed_camins = copy.deepcopy(remove_cycles(expand_camins))
        cami2 = copy.deepcopy(cami[1:])
        cami = copy.deepcopy(insert_depth_first_search(removed_camins, cami2))
        if cami[0].last == destination_id:
            return cami[0]
    return None


def insert_breadth_first_search(expand_paths, list_of_path):
    if len(list_of_path) != 0:
        expand_paths = list_of_path + expand_paths
    return expand_paths

def breadth_first_search(origin_id, destination_id, map):
    n1 = Path(origin_id)
    cami = [n1]
    if cami[0].last == destination_id:
        return cami
    while cami[0].last != destination_id or not cami:
        expand_camins = copy.deepcopy(expand(cami[0], map))
        removed_camins = copy.deepcopy(remove_cycles(expand_camins))
        cami2 = copy.deepcopy(cami[1:])
        cami = copy.deepcopy(insert_breadth_first_search(removed_camins, cami2))
        if cami[0].last == destination_id:
            return cami[0]
    return None

def calculate_cost(expand_paths, map, type_preference=0):
    cami = copy.deepcopy(expand_paths)
    if type_preference == 0:
        for path in cami:
                path.update_g(len(path.route)-1)
    elif type_preference == 1: #temps que tarda
        for path in cami:
                path.update_g(map.connections[path.last][path.penultimate])
    elif type_preference == 2: #distancia = time * velocitat
        for path in cami:
                estacio = map.stations[path.last]
                linia = estacio['line']
                estacio2 = map.stations[path.penultimate]
                linia2 = estacio2['line']
                if linia == linia2:
                    velocity = map.velocity[linia]
                    time = map.connections[path.last][path.penultimate]
                    path.update_g(velocity*time)
                elif estacio['x'] == estacio2['x'] and estacio['y'] == estacio2['y']:
                    path.update_g(0)
                else:
                    velocity = map.velocity[linia2]
                    time = map.connections[path.last][path.penultimate]
                    path.update_g(velocity * time)
    elif type_preference == 3: #si una persona canvia de metro, el g.transer+1
        for path in cami:
            for i in range(len(path.route) - 1):
                estacio1= map.stations[path.last]
                estacio2 = map.stations[path.penultimate]
                if estacio1['x'] == estacio2['x'] and estacio1['y'] == estacio2['y']:
                    path.update_g(1)
    return cami

def insert_cost(expand_paths, list_of_path):
    for path in expand_paths:
        trobat = 0
        pos = 0
        if len(list_of_path) == 0:
            list_of_path.append(path)
        else:
            while trobat == 0 and pos != len(list_of_path):
                if path.g < list_of_path[pos].g:
                    list_of_path.insert(pos, path)
                    trobat = 1
                pos= pos + 1
            if trobat == 0:
                list_of_path.append(path)
    return list_of_path

def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    llista = [Path(origin_id)]
    while llista[0].route[-1] != destination_id and len(llista) != 0:
        E = expand(llista.pop(0), map)
        E = remove_cycles(E)
        E = calculate_cost(E, map, type_preference)
        llista = insert_cost(E, llista)
    if llista is None:
        return 'No hi ha solució'
    else:
        return llista[0]

def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    cami = copy.deepcopy(expand_paths)
    for path in cami:
            if type_preference == 0: # min = 1 si no estas a la final if no es destiontion es 1,
                if path.last != destination_id:
                    path.update_h(1)
                else:
                    path.update_h(0)
            elif type_preference == 1:# uscar vel max de totes les linies, distancia entre velo
                max_velocity = 0
                for velocity in map.velocity:
                    if map.velocity[velocity] > max_velocity:
                        max_velocity = map.velocity[velocity]
                distancia = euclidean_dist([map.stations[path.last]['x'], map.stations[path.last]['y']], [map.stations[destination_id]['x'], map.stations[destination_id]['y']])
                path.update_h( distancia/ max_velocity)
            elif type_preference == 2: # euclidean dist
                distancia = euclidean_dist([map.stations[path.last]['x'], map.stations[path.last]['y']], [map.stations[destination_id]['x'], map.stations[destination_id]['y']])
                path.update_h(distancia)
            elif type_preference == 3: # si linia actual es igual a linea final, si so iguals
                if map.stations[destination_id]['line'] != map.stations[path.last]['line']:
                    path.update_h(1)
                else:
                    path.update_h(0)
    return cami

def update_f(expand_paths):
    for path in expand_paths:
        path.update_f()
    return expand_paths

def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """
    pass


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    pass


def coord2station(coord, map):
    distancia = []
    coordStation = []
    llista = []
    distmin= 1000000000
    for estacions in map.stations:
        coordStation = [map.stations[estacions]['x'], map.stations[estacions]['y']]
        dist2 =euclidean_dist(coord, coordStation)
        if dist2 < distmin:
            distancia.clear()
            distancia.append(estacions)
            distmin=dist2
        elif dist2 == distmin:
            distancia.append(estacions)
    return distancia

def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    pass
