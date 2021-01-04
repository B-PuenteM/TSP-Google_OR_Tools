#!/usr/bin/env python
# coding: utf-8

# In[1]:


from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.spatial import distance

from statistics import mean

# Importing the geodesic module to calculate geodesic distance matrix
from geopy.distance import geodesic 

import pandas as pd
import folium #To plot maps of the solution


# In[ ]:





# In[2]:


def Mapa_Inicial():
    
    m = folium.Map(
    location=[-0.176290, -78.483158],
    tiles='openstreetmap',
    zoom_start=13)

    m.add_child(folium.LatLngPopup())
    display(m)
    r='running'
    global input_coords_list
    input_coords_list=[]
    print('Ingrese sus ubicaciones comenzando por su lugar central. Para terminar ingrese la palabra "FIN": \n')
    while r== 'running':
    
        #coordenadas=input('Ingrese sus ubicaciones comenzando por su lugar central. Para terminar ingrese la palabra "FIN": \n')
        coordenadas=input()
        
        if coordenadas == 'FIN':
            r='FIN'
            break
    
        coordenadas=coordenadas.split(',')
        coordenadas = list(map(float, coordenadas))
        if type(coordenadas) == list:
            input_coords_list.append(coordenadas) #Cambio hecho aquí
            print('Punto: ',coordenadas,' registrado correctamente.\n')
        elif type(coordenadas) != list:
            print('Valores ingresados no válidos. Ingrese únicamente valores válidos de latitud y longitud según el mapa, o FIN para finalizar.\n')

def Initial_Map_FromCSV():
    coords=pd.read_csv("Input_Data_TSP.csv")
    print("Los puntos a considerar para la optimziación son:")

    PUNTO_DE_INICIO=4 #Variable donde se ingresa el nodo de inicio o "Home", este DEBE ser un valor entero entre 0 - len(coords), el usuario deberia seleccionar esto

    #This block assigns lat, long to separate lists
    global array
    array=coords.values.tolist()
    latitudes =[]
    longitudes=[]
    for i in range (0,len(array)):
        latitudes.append(array[i][0])
        longitudes.append(array[i][1])

    basemap = folium.Map(
                location=[mean(latitudes), mean(longitudes)],
                zoom_start=11)


    #This block plots the points to be considered
    global tooltip
    tooltip = 'Informacion'
    for i in range (0,len(array)):
        corresponding_icon='truck'
        corresponding_color='blue'
    
        if i==PUNTO_DE_INICIO:
            corresponding_icon='home'
            corresponding_color='green'
    
        folium.Marker(
        location=[latitudes[i], longitudes[i]],
        tooltip=tooltip,
        popup=('Punto: '+str(i)),
        icon=folium.Icon(icon=corresponding_icon, prefix='fa',color=corresponding_color)
        ).add_to(basemap)
    display(basemap)

def Initial_Map_From_Selected_Points():
    print("Los puntos a considerar para la optimziación son:")
    
    global PUNTO_DE_INICIO
    PUNTO_DE_INICIO=0 #Variable donde se ingresa el nodo de inicio o "Home", este DEBE ser un valor entero entre 0 - len(coords), el usuario deberia seleccionar esto

    #This block assigns lat, long to separate lists
    #print(input_coords_list)
    global array #CUIDADO, SE USA EL MISMO NOMBRE DE VARIABLE EN LA FUNCIÓN Initial_Map_FromCSV()
    array=input_coords_list
    latitudes =[]
    longitudes=[]
    for i in range (0,len(array)):
        latitudes.append(array[i][0])
        longitudes.append(array[i][1])

    basemap = folium.Map(
                location=[mean(latitudes), mean(longitudes)],
                zoom_start=11)


    #This block plots the points to be considered
    global tooltip #CUIDADO, SE USA EL MISMO NOMBRE DE VARIABLE EN LA FUNCIÓN Initial_Map_FromCSV()
    tooltip = 'Informacion'
    for i in range (0,len(array)):
        corresponding_icon='truck'
        corresponding_color='blue'
    
        if i==PUNTO_DE_INICIO:
            corresponding_icon='home'
            corresponding_color='green'
    
        folium.Marker(
        location=[latitudes[i], longitudes[i]],
        tooltip=tooltip,
        popup=('Punto: '+str(i)),
        icon=folium.Icon(icon=corresponding_icon, prefix='fa',color=corresponding_color)
        ).add_to(basemap)
    display(basemap)

    
def Matriz_Distancias():
    #Se genera la matriz de distancias utilizando la librería geodesic
    #El modelo DEBE utilizar distancias discretas, el solucionador no acepta distancias con punto decimal, las distancias se deben redondear siempre.
    #global array
    #array=coords.values.tolist()
    global distance_matrix
    distance_matrix=[]
    global points_tuple_list
    points_tuple_list=[]
    
    for i in range(0,len(array)):
        inner_list=[]
        for j in range (0,len(array)):
            orig=tuple(array[i])
            points_tuple_list.append(orig)
            dest=tuple(array[j])
            distancia_i=round(geodesic(orig, dest).m) #se ejecuta el calculo de distancia en METROS redondeada
            inner_list.append(distancia_i)
        distance_matrix.append(inner_list)
    #print("La matriz de distancia (distancia geodesica) en formato de lista es:\n")    
    #print(distance_matrix) habilitar si se quiere mostrar la matriz de distancias    
     
    
def create_data_model(): #Función que se encarga de guardar los datos necesarios para el modelo.
    """Stores the data for the problem."""
    data = {} #Crea un diccionario vacío el cual incluirá todos los datos de input del modelo: matriz de distancias, nodo de origen.
    data['distance_matrix'] = distance_matrix # Incluye la matriz de distancia generada del archivo leido previamente
    data['num_vehicles'] = 1 #1 vehículo por ser TSP, no tiene sentido por ahora añadir otro vehículo
    data['depot'] = PUNTO_DE_INICIO #define el punto de inicio, se puede cambiar la ubicación de inicio 0..n según se tengan los datos de inicio
    return data

def print_solution(manager, routing, solution): #Función que se encarga de imprimir la solución del problema
    """Prints solution on console."""
    "Esta función tiene 3 parametros"
    "manager: "
    "routing: "
    "solution:"
    
    global SOLUTION_LIST
    SOLUTION_LIST=[]
    
    print('Objective: {} km'.format(solution.ObjectiveValue()/1000)) #Imprime el formato de la función objetivo. Al ingresarse distancias de la matriz en metros, se divide para 1000, cambiar deacuerdo a los datos de inicio.
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n' #imprime el texto de plan de ruta
    route_distance = 0 #inicializa la distancia total del plan de ruta.
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        ##IBP
        #CREA UNA LISTA DEL ORDEN DE LAS SOLUCIONES
        SOLUTION_LIST.append(manager.IndexToNode(index))        
        
        ##IBP
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        #print(index) #included BP
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}km\n'.format(route_distance/1000)


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)


def Mostrar_Solucion_Mapa():
    #Crea indices para la lista de resultados
    indices=[]
    coords=pd.DataFrame(array, columns=['latitude','longitude'])
    latitudes=[]
    longitudes=[]
    for x in range(0,len(input_coords_list)):
        latitudes.append(input_coords_list[x][0])
    
    
    for y in range(0,len(input_coords_list)):
        longitudes.append(input_coords_list[y][1])
    
    #print(latitudes)
    #print(longitudes)
    for i in range(0,len(SOLUTION_LIST)):
        indices.append(i)
    indices
    #Crea una lista con los indices de la solución para poder ordenar el DF original
    indices=[]
    for i in range(0,len(SOLUTION_LIST)):
        indices.append(i)
    
    #Crea un DF para ordenar el DF original a partir de la solución
    dataframe_sol=pd.DataFrame()
    dataframe_sol['Solucion']=SOLUTION_LIST
    dataframe_sol['orden']=indices
    dataframe_sol=dataframe_sol.sort_values(by='Solucion') #ordena los datos en función de la solución encontrada
    #print(dataframe_sol)
    #Crea una columna 'orden' con el orden asignado a cada punto original de los datos de ingreso
    coords['orden']=dataframe_sol['orden'].values.tolist()
    DF_SOLUCION_PLOT=coords.sort_values(by='orden') #ordena en forma ascendente para tener el DF original ordenado con la solución encontrada
    #print(DF_SOLUCION_PLOT) #Dataframe original ordenado por el orden optimo encontrado en la solución

    #Genera una lista de tuples de los puntos ordenados según la solución
    #La librería solo acepta puntos en forma de tuple
    ordered_sol_list=DF_SOLUCION_PLOT.drop(columns=['orden']).values.tolist()
    plot_sol_list=[]
    for j in range (0,len(ordered_sol_list)):
            coord=tuple(ordered_sol_list[j])
            plot_sol_list.append(coord)
    #print(plot_sol_list)
    
    #Solucion con la ruta
    solution_map = folium.Map(
                location=[mean(latitudes), mean(longitudes)],
                zoom_start=11.5)

    #points=points_tuple_list
    points=plot_sol_list
    #print(points)
    j=0
    for each in points:
    
        corresponding_icon='truck'
        corresponding_color='blue'
    
        if j==0:
            corresponding_icon='home'
            corresponding_color='green'
    
        folium.Marker(
            each,
        tooltip=tooltip,
        popup=('Punto: '+str(SOLUTION_LIST[j])),
        icon=folium.Icon(icon=corresponding_icon, prefix='fa',color=corresponding_color)
        ).add_to(solution_map)
        j+=1
    
    #incluye lineas
    folium.PolyLine(points, color="red", weight=2.5, opacity=1).add_to(solution_map)
    display(solution_map)

def Ejecutar_Programa():
    Mapa_Inicial()
    Initial_Map_From_Selected_Points()
    Matriz_Distancias()
    main()
    Mostrar_Solucion_Mapa()


# ## Bienvenido a la aplicación de generación de rutas CATENA-USFQ. Para comenzar siga los siguientes pasos:
# 
# Al hacer click en el mapa desplegado abajo se mostrará la ubicación seleccionada en "latitud" y "longitud".
# 1. Haga click en la ubicación del lugar central desde donde sale a hacer las entregas.
# 2. Copie los valores encontrados de latitud y longitud y peguelos en la caja de texto "Ubicación:" debajo del mapa. Pegue ambos valores separados por una coma y presione enter, por ejemplo: -0.181030, -78.484018
# 3. De la misma forma que seleccionó su ubicación central de despacho, seleccione los puntos de entrega en el mapa e ingréselos en la caja de texto en el mismo formato (latitud y longitud separado por coma), y presione enter. Puede ingresar cuantos puntos desee. Para terminar ingrese la palabra "FIN".

# In[3]:


Ejecutar_Programa()


# In[ ]:





# In[4]:


#-0.2281,-78.3370
#-0.2130,-78.4063
#-0.2131,-78.4309
#-0.1823,-78.4402
#-0.2018,-78.4767
#-0.2061,-78.4853
#-0.2210,-78.5143
#-0.1372,-78.5009


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




