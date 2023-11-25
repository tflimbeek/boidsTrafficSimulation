import graphics as gr
import simulation as Sim

import numpy as np
import imgui.core as imgui
import glm

import sys
import random
import csv
import os

# Read the world files
# Information is used later on in aInit
# Using the Vehicle class to store vehicle data
class Vehicle():
    def __init__(self, x, y, rot, speed, start):
        self.x = x
        self.y = y
        self.rot = rot
        self.speed = speed
        self.start = start
    def __str__(self):
        return '[%.2f, %.2f] %.2f -> %.2f @ %d'%(self.x, self.y, self.rot, self.speed, self.start)

walls = []
exitBlockoffWalls = []  # Old name for road markers
vehicles = []
locationMarkers = []

def readWorld(file, simtime):
    N = 0   # Amount of vehicles
    M = 0   # Amount of walls
    P = 0   # Amount of road markers
    L = 0   # Amount of location markers

    # Use csv reader to import walls from maps/<file>.wls
    with open(file + '.wls', 'r') as f:
        reader = csv.reader(f)
        for r in reader:
            if r[0].startswith('#'):
                continue
            elif r[0].startswith('EBW'):
                exitBlockoffWalls.append([float(r[1]), float(r[2]), float(r[3]), float(r[4])])
                P+=1
                continue
            elif r[0].startswith('LOC'):
                continue
            M += 1
            walls.append([float(r[0]), float(r[1]), float(r[2]), float(r[3])])

    # Use csv reader to import vehicle start positions from maps/<file>.spn
    with open(file + '.spn', 'r') as f:
        reader = csv.reader(f)
        for r in reader:
            if r[0].startswith('#'):
                continue
            center = glm.vec2(float(r[0]), float(r[1]))
            pointer = glm.vec2(float(r[2]), float(r[3]))
            rate = float(r[4])
            speed = float(r[5])
            rot = glm.atan(pointer.y-center.y, pointer.x-center.x)

            for i in range(0, simtime, int(60/rate)):
                N += 1
                v = Vehicle(center.x, center.y, rot, speed, i)
                vehicles.append(v)
    
    # Use csv reader to import location markers from maps/<file>.loc
    with open(file + '.loc', 'r') as f:
        # Open file
        reader = csv.reader(f)
        
        # Create a list to be filled with zeros and find maximum number of location marker entries
        zeroList = []
        numberOfEntries = max(len(sublist) for sublist in reader) 
        L = numberOfEntries     
        f.seek(0)   # Reset file pointer to position 0
        
        # Fill list with zeros
        for cnt in range(numberOfEntries):
            zeroList.append(float(0.0))

        for r in reader:
            if r[0].startswith('#NA'):
                locationMarkers.append(zeroList)
                # L+=1
                continue
            
            tempList = []
            cnt = 0
            for tmp in r:
                tempList.append(float(tmp))
                cnt+=1
            tempList[cnt:numberOfEntries] = [float(0.0)]*(numberOfEntries-cnt)
            locationMarkers.append(tempList)
            
    return N, M, P, L


# Algorithm pass
#   This function is called each frame
#   Simulation object is passed as parameter
def aPass(sim:Sim.Simulation):
    algoProgram.dispatch(sim.N)

def aData(sim:Sim.Simulation):
    # Get data from first car
    posState = np.frombuffer(sim.posStateBuffer.getData(0), dtype='f').reshape((sim.N, 8))
    movState = np.frombuffer(sim.movStateBuffer.getData(0), dtype='f').reshape((sim.N, 16))
    internalData = np.frombuffer(sim.internalDataBuffer.getData(0), dtype='f').reshape((sim.N, 16))
    simState = np.frombuffer(sim.simStateBuffer.getData(0), dtype='uint32').reshape((sim.N, 4))
    # Do something with the data

# Initializing routine
#   This function is called before running simulation
#       the posStateBuffer and movStateBuffer should be set with initial data
#       in this function
#   Simulation object is passed as parameter
#   Must return a numpy array of floats with the positions of obstacles
#       in the form of [start.x start.y end.x end.y] dtype=float
def aInit(sim:Sim.Simulation):
    global walls, vehicles, exitBlockoffWalls, locationMarkers

    # Initialize posState, movState and simState buffers
    posState = np.zeros([sim.N, 8], dtype="f")
    movState = np.zeros([sim.N, 16], dtype="f")
    simState = np.zeros([sim.N, 4], dtype='uint32')

    # Fill posState, movState and simState buffers
    i = 0
    for v in vehicles:
        posState[i][0:4] = [v.x, v.y, 0.0, 1.0]
        posState[i][4] = v.rot-glm.pi()/2
        unitv = glm.vec2(glm.cos(v.rot), glm.sin(v.rot))
        movState[i][0:4] = [unitv.x*v.speed, unitv.y*v.speed, 0.0, 0.0]
        movState[i][4:8] = [unitv.x*v.speed, unitv.y*v.speed, 0.0, 0.0]
        movState[i][8] = 0.0
        movState[i][9] = 0.0
        simState[i][3] = v.start
        i+=1

    sim.posStateBuffer.setData(posState)
    sim.movStateBuffer.setData(movState)

    # Initialize wall, road marker location marker and visitedList buffers
    wallVertices = np.zeros([len(walls)*4], dtype='f')
    EBWVertices = np.zeros([len(exitBlockoffWalls)*4], dtype = 'f')     # EBW = exitBlockoffWall, old name for road markers
    locationMarkerCoords = np.zeros([len(locationMarkers)*sim.L], dtype = 'f')
    visitedList = np.zeros([sim.N*sim.L], dtype = 'f')

    # Fill wallVertices
    i = 0
    for w in walls:
        wallVertices[i:i+4] = w
        i += 4

    # Fill EBWVertices
    i = 0
    for ebw in exitBlockoffWalls:
        EBWVertices[i:i+4] = ebw
        i+=4

    # Fill in locationMarkerCoords
    i = 0
    print(sim.L)
    for lm in locationMarkers:
        locationMarkerCoords[i:i+sim.L] = lm
        i+=sim.L

    # Create list of out of bounds (oob) coordinates for the visited list
    # Compute shader needs an initialized, filled in buffer to function
    # These values are therefore placeholders
    i = 0
    oobList = []
    for cnt in range(int(sim.L/2)):
        oobList.append([float(0.0), float(0.0)])    # [x (float), y (float)]

    # Put the coordinate pairs from oobList into the visited list   
    for oobCoords in oobList:
        visitedList[i:i+2] = oobCoords
        i+=2
    

    return wallVertices, simState, EBWVertices, locationMarkerCoords, visitedList

# GUI drawing routine
#   This function should draw the GUI using ImGui
#       Changes can be made in sim.globalSettings
#   Simulation object is passed as parameter
def aGUI(sim:Sim.Simulation):
    imgui.begin("Settings")
    imgui.text("N=%d, M=%d"%(sim.globalSettings[1][0][1], sim.globalSettings[1][0][2]))
    # uDeltaTime
    _, sim.globalSettings[1][0][0] = imgui.slider_float("dt", sim.globalSettings[1][0][0], 0.0, 0.5, '%.4f')
    _, sim.globalSettings[1][0][3] = imgui.slider_float("coh", sim.globalSettings[1][0][3], 0.0, 10.0, '%.4f')
    _, sim.globalSettings[1][1][0] = imgui.slider_float("ali", sim.globalSettings[1][1][0], 0.0, 10.0, '%.4f')
    _, sim.globalSettings[1][1][1] = imgui.slider_float("sep", sim.globalSettings[1][1][1], 0.0, 10.0, '%.4f')
    _, sim.globalSettings[1][1][2] = imgui.slider_float("d_v", sim.globalSettings[1][1][2], 0.0, 100.0, '%.1f')
    _, sim.globalSettings[1][1][3] = imgui.slider_float("d_s", sim.globalSettings[1][1][3], 0.0, 100.0, '%.1f')
    _, sim.globalSettings[1][2][1] = imgui.slider_float("phi_max", sim.globalSettings[1][2][1], 0.0, 3.141592/360*90, '%.4f')
    _, sim.globalSettings[1][2][0] = imgui.slider_float("dphi_max", sim.globalSettings[1][2][0], 0.0, 3.141592/360*90, '%.4f')

    if imgui.button('Reset'):
        random.seed(sim.seed)
        sim.reset()
    imgui.end()

def main():
    global algoProgram

    # cohesion, alignment and separation factors
    c=0.1
    a=6.0
    s=2.0
    # visual distance and separation distance
    dv = 15.0
    ds = 10.0
    # maximum steering angle and steering angle speed
    # now practically infinite
    dphi_max = 3.141592/360*37
    phi_max = 3.141592/360*37

    # Simulation time in frames
    simtime=90*60

    # Ask the user for the predefined map or the most recent iteration from the map builder
    # If not wanted, disable this section and manually set the path to the file as "file"
    # The map builder outputs a map as out.spn and out.wls 
    # The map builder is currently broken, and went unused. This is why it does not output a .loc file. 
    while True:
        usePredefWorld = (input("Use predefined world or map builder? (type p or m)"))
        if usePredefWorld == "p" or "m":
            break
        else:
            print("Invalid input, please try again")
            continue

    file = os.getcwd() + "/maps/"

    if usePredefWorld == "m":
        file += "out"
    else:   # Ask the user which map to use
        mapList = []
        for files in os.listdir(file):
            
            if files.endswith('.wls'):
                if files == 'out.wls':
                    continue
                mapList.append(files.replace(".wls",""))
        while True:
            print("Choose map: (type name)")
            print(mapList)
            choice = input()
            if not choice in mapList:
                print("Invalid map name, please try again")
                continue
            else:
                file += choice
                break

    # Read the desired map into the simulation environment
    N, M, P, L = readWorld(file, simtime)

    # Initialize simulation
    # Needs to be ran before doing graphics stuff! (this creates the openGL context)
    sim = Sim.Simulation(N, L, False, simtime, aInit, aPass, aGUI, aData, 1, True, False) 

    # Create algorithm shader
    shaders = []
    with open("shaders/algorithm.comp") as f:
        shaders.append(gr.Shader(sim.header + f.read(), gr.COMPUTE_SHADER))
    algoProgram = gr.ShaderProgram(shaders)

    sim.globalSettings[1][0][3] = c           # cohesion
    sim.globalSettings[1][1][0] = a           # alignment
    sim.globalSettings[1][1][1] = 1.6           # seperation
    sim.globalSettings[1][1][2] = dv          # d_v
    sim.globalSettings[1][1][3] = ds          # d_s
    sim.globalSettings[1][2][0] = dphi_max
    sim.globalSettings[1][2][1] = phi_max
    sim.globalSettings[1][0][0] = 0.05      # Time step per frame
    sim.globalSettings[1][3][0] = 5.90      #Emergency distance
    sim.globalSettings[1][3][1] = 8.75      #Brake multiplier
    sim.globalSettings[1][3][2] = 8.5       #Proportional braking distance

    # Start simulation
    sim.start()

    # Handle openGL context shutdown
    sim.window.close()
    sim.window.shutdown()

if __name__=='__main__':
    main()