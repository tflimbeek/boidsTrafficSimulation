import logging
logger = logging.getLogger(__name__)

import sys
sys.path.append("/home/thijmenlimbeek/Documents/Bachelor thesis/vehicleFlockingBthesis")

import graphics as gr
import simulation as Sim

import random
import numpy as np
import imgui.core as imgui
import matplotlib.pyplot as plt
import glm
import csv
import os
import csv

from contextlib import contextmanager
from timeit import default_timer
import time
import sys

@contextmanager
def elapsed_timer():
    start = default_timer()
    elapser = lambda: default_timer() - start
    yield lambda: elapser()
    end = default_timer()
    elapser = lambda: end-start

class Vehicle():
    def __init__(self, x, y, rot, speed, start):
        self.x = x
        self.y = y
        self.rot = rot
        self.speed = speed
        self.start = start
    def __str__(self):
        return '[%.2f, %.2f] %.2f -> %.2f @ %d'%(self.x, self.y, self.rot, self.speed, self.start)

# -------------------------------------------------------------------------

# Algorithm pass
#   This function is called each frame
#   Simulation object is passed as parameter
def aPass(sim:Sim.Simulation):
    algoProgram.dispatch(sim.N)

# Data gathering pass
#   This function is called after period amount of steps
#   Simulation object is passed as parameter
posY = [[], [], []]
posX = [[], [], []]
velY = [[], [], []]
velX = [[], [], []]
dvelY = [[], [], []]
dvelX = [[], [], []]
frames = []
time = []

# Internal data
iCohesion = []
iAlignment = []
iSeperation = []
iCollisions = []
test = []
colNumber = 0
xList = []
yList = []
endTime = []
endTimeTest = 0.0
earlyEnd = False
frameNumbers = []
skipList = []

initialCoords = []
def aData(sim:Sim.Simulation):
    # Get data from first car
    posState = np.frombuffer(sim.posStateBuffer.getData(0), dtype='f').reshape((sim.N, 8))
    movState = np.frombuffer(sim.movStateBuffer.getData(0), dtype='f').reshape((sim.N, 16))
    
    
    #REMOVE LATER
    # test.clear()
    # print("\nTEST\n")
    # print(movState)
    # print("\nTEST\n")
    # print(movState[8,0])
    # print(movState[8,1])

    
    internalData = np.frombuffer(sim.internalDataBuffer.getData(0), dtype='f').reshape((sim.N, 16))
    simState = np.frombuffer(sim.simStateBuffer.getData(0), dtype='uint32').reshape((sim.N, 4))
    posX[0].append(posState[0,0])
    posY[0].append(posState[0,1])
    velX[0].append(movState[0,0])
    velY[0].append(movState[0,1])
    dvelX[0].append(movState[0,4])
    dvelY[0].append(movState[0,5])
    posX[1].append(posState[sim.N//2,0])
    posY[1].append(posState[sim.N//2,1])
    velX[1].append(movState[sim.N//2,0])
    velY[1].append(movState[sim.N//2,1])
    dvelX[1].append(movState[sim.N//2,4])
    dvelY[1].append(movState[sim.N//2,5])
    # posX[2].append(posState[1+(sim.N//2),0])
    # posY[2].append(posState[1+(sim.N//2),1])
    # velX[2].append(movState[1+(sim.N//2),0])
    # velY[2].append(movState[1+(sim.N//2),1])
    # dvelX[2].append(movState[1+(sim.N//2),4])
    # dvelY[2].append(movState[1+(sim.N//2),5])
    frames.append(sim.stepCount)
    time.append(sim.time)

    collisions = 0
    for i in range(sim.N):
        collisions += simState[i, 1]
    iCollisions.append(collisions)
    global colNumber, earlyEnd
    colNumber = collisions
    if collisions > 0:
        earlyEnd = True
        # sim.window.close()
        # sim.window.shutdown()
        # return

    temptest = []
    # print(movState)
    for i in range(sim.N):
        temptest.append(np.sqrt(pow(movState[i,0], 2) + pow(movState[i,1], 2)))
    test.append(temptest)

    x = []
    y = []

    speedList = []

    for i in range(sim.N):
        x.append(posState[i,0])
        y.append(posState[i,1])

        if posState[i,1] < 0.01:
            initialCoords[i][0] = posState[i,0]
        # if posState[i,0] > 150:
        #     endTime[i] = sim.time
        speedList.append(test[-1][i])
    global endTimeTest
    if all(x < 0.01 for x in speedList) and endTimeTest < 0.1:
        endTimeTest = sim.time

    xList.append(x)
    yList.append(y)
        
    
    
    # print(test)

    # SAVE CAS DATA
    C = 0
    A = 0
    S = 0
    for i in range(sim.N):
        coh = np.array(internalData[i, 0:4])
        ali = np.array(internalData[i, 4:8])
        sep = np.array(internalData[i, 8:12])
        C += np.sqrt(np.dot(coh, coh))
        A += np.sqrt(np.dot(ali, ali))
        S += np.sqrt(np.dot(sep, sep))
    iCohesion.append(C/sim.N)
    iAlignment.append(A/sim.N)
    iSeperation.append(S/sim.N)

    global locationMarkers, frameNumbers, skipList
 
    for i in range(sim.N):
        if(skipList[i]) == True:
            continue
        if(posState[i,1] > -10.0 and np.sqrt(pow(locationMarkers[i][-2] - posState[i,0], 2) + pow(locationMarkers[i][-1] - posState[i,1], 2)) > 10.0):
           frameNumbers[i] += 1
           if frames[-1] >= sim.steps:
               frameNumbers[i] = 'DNF'
        if(np.sqrt(pow(locationMarkers[i][-2] - posState[i,0], 2) + pow(locationMarkers[i][-1] - posState[i,1], 2)) < 10.0):
            skipList[i] = True

    # test.append(np.sqrt(pow(velX[1][-1], 2) + pow(velY[1][-1], 2)))
    # print("\nVehicle 1\n")
    # print(movState)
    # print("\nVehicle 2\n")
    # print(np.frombuffer(sim.movStateBuffer.getData(0), dtype='f').reshape((sim.N, 16)))
    #print(np.sqrt(pow(velX[1][-1], 2) + pow(velY[1][-1],2)))
    
    # CHECK IF A CAR HAS COLLIDED
    

# Initializing routine
#   This function is called before running simulation
#       the posStateBuffer and movStateBuffer should be set with initial data
#       in this function
#   Simulation object is passed as parameter
#   Must return a numpy array of floats with the positions of obstacles
#       in the form of [start.x start.y end.x end.y] dtype=float
def aInit(sim:Sim.Simulation):
    global walls, vehicles, exitBlockoffWalls, locationMarkers

    posState = np.zeros([sim.N, 8], dtype="f")
    movState = np.zeros([sim.N, 16], dtype="f")
    simState = np.zeros([sim.N, 4], dtype='uint32')

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

    # Normals are from Start to end clockwise (PI/2)
    
    wallVertices = np.zeros([len(walls)*4], dtype='f')
    EBWVertices = np.zeros([len(exitBlockoffWalls)*4], dtype = 'f')
    locationMarkerCoords = np.zeros([len(locationMarkers)*sim.L], dtype = 'f')

    visitedList = np.zeros([sim.N*sim.L], dtype = 'f')

    i = 0
    for w in walls:
        wallVertices[i:i+4] = w
        i += 4

    # Fill EBWVertices
    i = 0
    for ebw in exitBlockoffWalls:
        EBWVertices[i:i+4] = ebw
        i+=4

    print(sim.L)

    # Fill in locationMarkerCoords
    i = 0
    for lm in locationMarkers:
        locationMarkerCoords[i:i+sim.L] = lm
        i+=sim.L

    i = 0
    _tempList = []
    for cnt in range(int(sim.L/2)):
        _tempList.append([float(0.0), float(0.0)])
        
    for test in _tempList:
        visitedList[i:i+2] = test
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

# -------------------------------------------------------------------------

def runSimulation(sim, c, a, s, dv, ds, phi_max, dphi_max):
    #print(c, a, s, dv, ds, phi_max, dphi_max)
    sim.seed = 0
    sim.globalSettings[1][0][3] = c           # cohesion
    sim.globalSettings[1][1][0] = a           # alignment
    sim.globalSettings[1][1][1] = s           # seperation
    sim.globalSettings[1][1][2] = dv          # d_v
    sim.globalSettings[1][1][3] = ds          # d_s
    sim.globalSettings[1][2][0] = dphi_max
    sim.globalSettings[1][2][1] = phi_max
    # sim.globalSettings[1][0][0] = 0.05

    # Start simulation
    sim.start()

walls = []
exitBlockoffWalls = []
vehicles = []
locationMarkers = []

def readWorld(file, simtime):
    N = 0
    M = 0
    P = 0
    L = 0

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
            #     locationMarkers.append([float(r[1]), float(r[2])])
            #     L+=1
                continue
            M += 1
            walls.append([float(r[0]), float(r[1]), float(r[2]), float(r[3])])
            # walls.append([float(r[2]), float(r[3]), float(r[0]), float(r[1])])

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
                #print(v)
                vehicles.append(v)

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
            # L+=1
    return N, M, P, L


def main(c=0.1, a=6.0, s=3.0, simtime=45*15, de = 5.0, fp = 6.0, dp = 8.5, fs = 3.0, ds = 10.0):
    global algoProgram

    walls.clear()
    vehicles.clear()

    file = 'out'
    # simtime = 60*15
    N, M, P, L = readWorld(file, simtime)

    for nmb in range(N):
        initialCoords.append([0.0, 0.0])
        endTime.append(0.0)
        frameNumbers.append(0)
        skipList.append(False)

    print(initialCoords)

    # Initialize simulation
    # Number of vehicles, number of x and y coordinates (number of coordinates times 2), VSync, time, aInit function, aPass function, 
    #   aGUI function, aData function, aData period, Rendering, data acquisition mode (true for this file)
    sim = Sim.Simulation(N, L, False, simtime, aInit, aPass, aGUI, aData, 1, True, True)
    sim.globalSettings[1][3][0] = de
    sim.globalSettings[1][3][1] = fp
    sim.globalSettings[1][3][2] = dp

    # Create algorithm shader
    shaders = []
    with open("../shaders/algorithm.comp") as f:
        shaders.append(gr.Shader(sim.header + f.read(), gr.COMPUTE_SHADER))
    algoProgram = gr.ShaderProgram(shaders)

    with elapsed_timer() as elapsed:
        #DON'T FORGET TO CHANGE fs BACK TO s
        runSimulation(sim, c, a, fs, 15.0, ds, 3.141592/360*37, 3.141592/360*37) #don't forget to change back
    t_el = elapsed()
    
    if not earlyEnd:
        sim.window.close()
        sim.window.shutdown()
    
    

    print("distanceBuffer", sim.distanceBuffer.ID)
    print("movStateBuffer", sim.movStateBuffer.ID)

    del(sim)
    del(algoProgram)
    return t_el, N
    
# if __name__=='__main__':

#     c = float(sys.argv[1])
#     a = float(sys.argv[2])
#     s = float(sys.argv[3])

#     nrstreams = 3
#     w = float(sys.argv[4])

#     # Create config files
#     with open('out.wls', 'w') as f:
#         f.write('%f, %f, %f, %f\r\n'%(0,-50,0,280))
#         f.write('%f, %f, %f, %f\r\n'%(w,280,w,-50))
        
#     with open('out.spn', 'w') as f:
#         # dist = 10.0
#         xoff = float(w)/2.0 - 10
#         dist = 10

#         if xoff <= 5:
#             dist = float(w)/float(nrstreams+1)
#             xoff = dist
#         for x in range(nrstreams):
#             for y in range(3):
#                     f.write('%f, %f, %f, %f, 0.01, 1.0\r\n'%(x*dist+xoff, y*dist, x*dist+xoff, y*dist+1.0))

#     t_el, N = main(c, a, s, int(60*20))
#     print('%d : %f'%(N, t_el))
if __name__=='__main__':

    c = float(sys.argv[1])
    a = float(sys.argv[2])
    s = float(sys.argv[3])

    
    de = float(sys.argv[4])
    fp = float(sys.argv[5])
    dp = float(sys.argv[6])
    fs = float(sys.argv[7])

    nrstreams = 3
    w = 40

    # Create config files
    # with open('out.wls', 'w') as f:
    #     f.write('%f, %f, %f, %f\r\n'%(0,-50,0,80))
    #     f.write('%f, %f, %f, %f\r\n'%(0,80,5,80+fl))
    #     f.write('%f, %f, %f, %f\r\n'%(5,80+fl,5,280+fl))
    #     f.write('%f, %f, %f, %f\r\n'%(35,280+fl,35,80+fl))
    #     f.write('%f, %f, %f, %f\r\n'%(35,80+fl,40,80))
    #     f.write('%f, %f, %f, %f\r\n'%(40,80,40,-50))


        
    # with open('out.spn', 'w') as f:
    #     # dist = 10.0
    #     xoff = float(w)/float(nrstreams+1)
    #     dist = xoff
    #     for x in range(nrstreams):
    #         for y in range(3):
    #             if x==1:
    #                 f.write('%f, %f, %f, %f, 0.01, 1.0\r\n'%(x*dist+xoff, y*dist+dist*fph, x*dist+xoff, y*dist+1.0+dist*fph))
    #             else:
    #                 f.write('%f, %f, %f, %f, 0.01, 1.0\r\n'%(x*dist+xoff, y*dist, x*dist+xoff, y*dist+1.0))

    t_el, N = main(c, a, s, int(75*20), de, fp, dp, fs)
    print('%d : %f'%(N, t_el))

    # analyzeVehicle = 0


    for car in range(N):
        xVehicle = []
        
        for idx in xList:
            xVehicle.append(idx[car])
        
        yVehicle = []
        
        for idx in yList:
            yVehicle.append(idx[car])
        plt.plot(xVehicle, yVehicle, linestyle = "dashed", label = 'Vehicle '+str(car), linewidth = 3)

    # if 'DNF' in frameNumbers: #frameNumbers [0] == 'DNF':#or frameNumbers [4] == 'DNF' or frameNumbers [6] == 'DNF' or frameNumbers [7] == 'DNF':   #'DNF' in frameNumbers
    #     simulationFrames = 'DNF'
    # else:
    #     # simulationFrames = str(frameNumbers[0])#str(round(np.average([frameNumbers[1], frameNumbers[4], frameNumbers[6], frameNumbers[7]])))
    #     simulationFrames = str(round(np.average(frameNumbers)))
    print(frameNumbers)
    simulationFrames = str(frameNumbers[0])
    #simulationFrames = str(round(np.average([frameNumbers[0], frameNumbers[2], frameNumbers[5], frameNumbers[7]])))
      
    # plt.plot(xVehicle, yVehicle, label = 'Vehicle 4')
    plt.plot([40.0], [50.0], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")
    plt.plot([65.0], [75.0], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")
    plt.plot([110.0], [82.5], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")
    plt.plot([160.0], [90.0], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")
    
    # plt.plot([-34], [161.0], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")
    # plt.plot([74.0], [161.0], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")
    # plt.plot([31.5], [115.0], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")
    # plt.plot([8.5], [115.0], marker = "o", markersize = 8, markeredgecolor = "black", markerfacecolor = "orange")

    plt.xlabel("x coordinate", fontsize = 13)
    plt.ylabel("y coordinate", fontsize = 13)
    plt.xlim([0, 170]) #-60 100
    plt.ylim([0, 110])
    # plt.xlim([-60, 100])
    # plt.ylim([65, 190])
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    plt.grid()
    plt.title("Actual path vs markers. Avg. time to goal from start pos: " + simulationFrames + " frames", fontsize = 13)
    plt.legend()
    plt.savefig('Figures/test.png', dpi = 500)
    plt.close()




    # if 'DNF' in frameNumbers:
    #     simulationFrames = 'DNF'
    # else:
    #     simulationFrames = str(round(max(frameNumbers), 1))

    
    # if not earlyEnd:
    #     # # Save data to csv
    #     with open('csv/time_' + simulationFrames + '_fs_' + str(fs)  + '_dp_'  + str(dp)  + '_fp_' + str(fp)  + '_de_' + str(de) + '.csv', 'w') as f:
    #         writer = csv.writer(f)
    #         writer.writerow(time)
    #         # writer.writerows(test)
    #         for i in range(N):
    #             transferlist = []
    #             for idx in test:
    #                 transferlist.append(idx[i])
    #             writer.writerow(transferlist)
    #         f.close()
    # else:
        
    #     # with open('csv/crash' + '_fs_' + str(fs)  + '_dp_'  + str(dp)  + '_fp_' + str(fp)  + '_de_' + str(de) + '.csv', 'w') as f:
    #     #     writer = csv.writer(f)
    #     #     writer.writerow(time)
    #     #     # writer.writerows(test)
    #     #     for i in range(N):
    #     #         transferlist = []
    #     #         for idx in test:
    #     #             transferlist.append(idx[i])
    #     #         writer.writerow(transferlist)
    #     #     f.close()
        
    #     # with open('csv/siminfo.csv', 'r') as f:
    #     #     reader = csv.reader(f)
    #     #     for r in reader:
    #     #         crashUpdate = int(r[0]) + 1
    #     #     f.close()
    #     #     print(crashUpdate)

    #     with open('csv/siminfo.csv', 'a') as f:  
    #         writer = csv.writer(f)
    #         writer.writerow([str(de)])
    #         f.close()
            


