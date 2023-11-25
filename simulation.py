import graphics as gr

import random
import numpy as np
import glm
import glfw
import OpenGL.GL as gl
import ctypes
from typing import Callable, Sequence

import imgui.core as imgui

CAMSPEED = 0.4
ZOOMSPEED = 0.05

ANGLESPEED = 0.05

class Simulation:


    def __init__(self, N:int=10, L:int=10, fastrun:bool=False, steps:int=0, algoInit:Callable=None, algoPass:Callable=None, guiPass:Callable=None, dataPass:Callable=None, dataPassPeriod:int=100, rendering:bool=True, dataAcquisitionMode:bool=False):
        """ Create simulation object
        parameters:
            N : int                 Number of vehicles
            fastrun : bool          If True, VSync is turned off, if false it runs at VSync
            steps : int             Amount of steps/frames are ran. 0 for endless run
            algoInit : function     Initializer function. This is called before the simulation 
                is started. Use this to fill the world with vehicles and obstacles. Vehicles are
                placed into the simulation by creating their posState and movState buffers (see
                shaders/header.glsl). Next to that obstacles (walls) are created by returning a
                numpy array with the vertices of the walls (start x,y end x,y). Road facing side
                of a wall is the right facing side (clockwise) from start to end. At last the
                simulation state buffer must be returned in a numpy array
                In short -> posState:[{4*float position, float rotation, 
                3*float padding}]. Note: rotation is with 0* up increasing clockwise!!!
                movState: [{4*float velocity, 4*float desired velocity (set to velocity),
                float steering angle (set to 0), float speed of rear axis (set to 0) and
                6*float padding}]
                wall vertices -> [{startx, starty, endx, endy}] all floats
                simState -> [{uint ran steps for vehicle, uint used as bool for collision,
                float elapsed time in sim, uint start frame number for vehicle}]. With the last
                member one can set the start time of a vehicle entering the simulation. All
                the vehicles must be created at the start of the simulation and this is the way
                to let them enter one after each other
            algoPass : function     Algorithm pass function. Function should dispatch the shader(s)
                with the algorithm. Using algoProgram.dispatch(sim.N) should work
            guiPass : function      Gui pass function. Draw the GUI (i.e. imgui). Settings of the
                simulation are stored in Simulation.globalSettings. See header.glsl for all the
                settings
            dataPass : function     Data pass function is called after period steps. This function
                can be used to gather simulation data. One can retrieve data from the buffers using
                the np.frombuffer functions 
                (i.e. sim.frombuffer(sim.posSataBuffer.getData(0), dtype='f')).reshape((sim.N,8))
            dataPassPeriod : int    Period of data pass function
            rendering : bool        Set to false if rendering must be disabled
        """

        self.N = N
        self.L = L
        self.steps = steps
        self.algoInit = algoInit
        self.algoPass = algoPass
        self.guiPass = guiPass
        self.dataPass = dataPass
        self.dataPassPeriod = dataPassPeriod
        self.rendering = rendering
        self.seed = 0 

        # Create window and OpenGL context
        self.window = gr.Window(800, 800, self._onEvent, self._renderPass, self._resize)

        # Set path to the desired shaders. This is used for the data collection code under the /ex/ directory because it can't find the shaders in its own directory
        self.shaderPathPrefix = "shaders/"
        if dataAcquisitionMode:
            self.shaderPathPrefix = "../shaders/"   # To specify, ".." is used to go up a directory.

        # Create assets and buffers
        self._createAssets()
        self._createBuffers()

        # If fastrun is enabled
        if fastrun:
            glfw.swap_interval(0)
            

        # Create global settings
        self.globalSettings = np.zeros([2,4,4], dtype="f")
        ratio = float(self.window.width)/float(self.window.height)                          # Calculate ratio for orhtographic projection
        self.globalSettings[0] = glm.ortho(-ratio*10, ratio*10, -10.0, 10.0).to_list()              # ViewProjection matrix
        self.globalSettings[1][0][0] = 0.1                                                  # deltaTime

        self.stepCount = 0
        self.time = 0.0
        self.inReset = False
        self.zoomLevel = 1.0
        self.started = False

    """ Start simulation
    """
    def start(self):
        if self.started:
            self._reset()
            return
        self.started = True

        # Initialize algorithm by calling algoInit function
        wallVertices, simState, EBWVertices, locationMarkerCoords, visitedList = self.algoInit(self)
        # Create obstacle vertex and index buffers
        self._wallVBuffer.setData(wallVertices)
        wallIndices = np.arange(0, len(wallVertices), 1, dtype="uint32")
        self._wallIBuffer.setData(wallIndices)
        self.M = len(wallVertices)//4           # M = amount of obstacles

        self._EBWVBuffer.setData(EBWVertices)
        EBWIndices = np.arange(0, len(EBWVertices), 1, dtype = "uint32")
        self._EBWIBuffer.setData(EBWIndices)
        self.P = len(EBWVertices)//4            # P = amount of road markers

        # Set the data for the location marker, and visited buffers.        
        self._locMarkBuffer.setData(locationMarkerCoords)
        self._visitedBuffer.setData(visitedList)

        print("N = %d, M = %d"%(self.N, self.M))

        # Set global settings
        self._bindBuffers()
        self.globalSettings[1][0][1] = float(self.N)
        self.globalSettings[1][0][2] = float(self.M)
        self.globalSettings[1][2][2] = float(self.P)
        self.globalSettings[1][2][3] = float(self.L)
        self.globalSettingsBuffer.setData(self.globalSettings)
        gl.glMemoryBarrier(gl.GL_UNIFORM_BARRIER_BIT)

        # Reserve space for M dependent buffers
        self.distanceBuffer.reserveData(4*2*self.N*(self.N+self.M+self.P))
        self.wallInfoBuffer.reserveData(4*1*self.M)
        self.roadMarkingBuffer.reserveData(4*1*self.P)

        # Zero out simStateBuffer
        #simState = np.zeros([self.N, 4], dtype="uint32")
        self.simStateBuffer.setData(simState)

        # Execute precalc shader
        # This will calculate obstacle normals
        self.precalcProgram.dispatch(self.M)
        gl.glMemoryBarrier(gl.GL_SHADER_STORAGE_BARRIER_BIT)

        # Execute road marker precalc shader
        # This will calculate road marker normals
        self.roadMarkerPrecalcProgram.dispatch(self.P)
        gl.glMemoryBarrier(gl.GL_SHADER_STORAGE_BARRIER_BIT)

        # Run the simulation
        self.stepCount = 0
        self.window.run()

    """ Create buffers for simulation
    """
    def _createBuffers(self):
        # Create buffer objects
        self.posStateBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)
        self.movStateBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)
        self.simStateBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)
        self.globalSettingsBuffer = gr.Buffer(gr.UNIFORM_BUFFER, gr.DYNAMIC_DRAW)
        self.wallInfoBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)
        self.distanceBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)
        self.debugBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)
        self.internalDataBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)
        self.roadMarkingBuffer = gr.Buffer(gr.SHADER_STORAGE_BUFFER, gr.STATIC_DRAW)

        # Reserve space for buffers
        self.posStateBuffer.reserveData(self.N*8*4)
        self.movStateBuffer.reserveData(self.N*16*4)
        self.globalSettingsBuffer.reserveData(22*4)
        self.debugBuffer.reserveData(self.N*16*4)
        self.internalDataBuffer.reserveData(self.N*16*4)

        # Zero out simStateBuffer
        simState = np.zeros([self.N, 4], dtype="uint32")
        self.simStateBuffer.setData(simState)

    """ Bind buffers to right binding
    """
    def _bindBuffers(self):
        self.globalSettingsBuffer.bindBase(1)
        self.posStateBuffer.bindBase(2)
        self.movStateBuffer.bindBase(3)
        self.simStateBuffer.bindBase(4)
        self.distanceBuffer.bindBase(5)
        self._wallVBuffer.bindBase(6, type=gr.SHADER_STORAGE_BUFFER)
        self.wallInfoBuffer.bindBase(7)
        self.debugBuffer.bindBase(8)
        self.internalDataBuffer.bindBase(9)
        self._EBWVBuffer.bindBase(10, type=gr.SHADER_STORAGE_BUFFER)
        self.roadMarkingBuffer.bindBase(11)
        self._locMarkBuffer.bindBase(12, type=gr.SHADER_STORAGE_BUFFER)
        self._visitedBuffer.bindBase(13, type=gr.SHADER_STORAGE_BUFFER)

    """ Create assets for drawing
    """
    def _createAssets(self):
        # CAR VAO (H)
        self._carVBuffer = gr.Buffer(gr.VERTEX_BUFFER, gr.STATIC_DRAW)
        vertices = np.array([
            # X, Y, Z,
            # H shape top
            -0.9, 1.4, 0.0,
            0.9, 1.4, 0.0,
            0.9, 1.3, 0.0,
            -0.9, 1.3, 0.0,
            # H shape center
            -0.05, 1.3, 0.0,
            0.05, 1.3, 0.0,
            0.05, -1.3, 0.0,
            -0.05, -1.3, 0.0,
            # H shape bottom
            -0.9, -1.3, 0.0,
            0.9, -1.3, 0.0,
            0.9, -1.4, 0.0,
            -0.9, -1.4, 0.0,
            # size Box
            -0.9, 2.45, 0.0,            #// 0 3
            0.9, 2.45, 0.0,             #// 1
            0.9, 2.42, 0.0,
            -0.9, 2.42, 0.0,
            -0.9, -2.45, 0.0,           #// 5
            0.9, -2.45, 0.0,            #// 2 4
            0.9, -2.42, 0.0,
            -0.9, -2.42, 0.0,
            -0.9, 2.45, 0.0,
            -0.87, 2.45,0.0,
            -0.87, -2.45, 0.0,
            -0.9, -2.45, 0.0,
            0.9, 2.45, 0.0,
            0.87, 2.45,0.0,
            0.87, -2.45, 0.0,
            0.9, -2.45, 0.0,
        ], dtype="f")
        self._carVBuffer.setData(vertices)
        self._carIBuffer = gr.Buffer(gr.INDEX_BUFFER, gr.STATIC_DRAW)
        indices = np.array([
            0, 1, 2, 0, 2, 3,
            4, 5, 6, 4, 6, 7,
            8, 9, 10, 8, 10, 11,
            12, 13, 14, 12, 14, 15,
            16, 17, 18, 16, 18, 19,
            20, 21, 22, 20, 22, 23,
            24, 25, 26, 24, 26, 27,
            12, 13, 17,
            12, 17, 16,
        ], dtype="uint32")
        self._carIBuffer.setData(indices)
        self.carVAO = gr.VertexArray(self._carVBuffer, self._carIBuffer, [
            gr.VertexElement(3, gr.FLOAT)
        ])

        # CONE VAO
        # Create all the necessary components for the "collision field of view" of the vehicles as indicated by the green triangular cones.
        # If the cones are unwanted, they can either be fully removed, or their transparency can be disabled in the "cone_color" frag shader
        cone_height = 10
        self._coneVBuffer = gr.Buffer(gr.VERTEX_BUFFER, gr.STATIC_DRAW)
        vertices = np.array([
            0.0, 0.0, 0.0,
            -np.tan(22.5)*cone_height, cone_height, 0.0,
            np.tan(22.5)*cone_height, cone_height, 0.0,
        ], dtype = "f")
        self._coneVBuffer.setData(vertices)
        self._coneIBuffer = gr.Buffer(gr.INDEX_BUFFER, gr.STATIC_DRAW)
        indices = np.array([
            0, 1, 2
        ], dtype = "uint32")
        self._coneIBuffer.setData(indices)
        self.coneVAO = gr.VertexArray(self._coneVBuffer, self._coneIBuffer, [
            gr.VertexElement(3, gr.FLOAT)
        ])

        # LINE VAO
        self._lineVBuffer = gr.Buffer(gr.VERTEX_BUFFER, gr.STATIC_DRAW)
        vertices = np.array([
            -0.02, 0.0, 0.0,
            -0.02, 1.0, 0.0,
            0.02, 1.0, 0.0,
            0.02, 0.0, 0.0,
        ], dtype="f")
        self._lineVBuffer.setData(vertices)
        self._lineIBuffer = gr.Buffer(gr.INDEX_BUFFER, gr.STATIC_DRAW)
        indices = np.array([
            0, 1, 2, 0, 2, 3
        ], dtype="uint32")
        self._lineIBuffer.setData(indices)
        self.lineVAO = gr.VertexArray(self._lineVBuffer, self._lineIBuffer, [
            gr.VertexElement(3, gr.FLOAT)
        ])

        # Wall
        # ----
        self._wallVBuffer = gr.Buffer(gr.VERTEX_BUFFER, gr.STATIC_DRAW)
        self._wallIBuffer = gr.Buffer(gr.INDEX_BUFFER, gr.STATIC_DRAW)
        self.wallVAO = gr.VertexArray(self._wallVBuffer, self._wallIBuffer, [
            gr.VertexElement(2, gr.FLOAT)
        ])

        # Exit separating lines
        self._EBWVBuffer = gr.Buffer(gr.VERTEX_BUFFER, gr.STATIC_DRAW)
        self._EBWIBuffer = gr.Buffer(gr.INDEX_BUFFER, gr.STATIC_DRAW)
        self.EBWVAO = gr.VertexArray(self._EBWVBuffer, self._EBWIBuffer, [
            gr.VertexElement(2, gr.FLOAT)
        ])

        # Location marker
        self._locMarkBuffer = gr.Buffer(gr.VERTEX_BUFFER, gr.STATIC_DRAW)

        self._visitedBuffer = gr.Buffer(gr.VERTEX_BUFFER, gr.STATIC_DRAW)

        # Create shaders
        # --------------
        # The header contains all the buffer bindings and will be prepended to all the shaders
        header = ''
        with open(self.shaderPathPrefix+"header.glsl") as f:
            header = f.read()

        # Car drawing program
        # Draws vehicle as red 'H'
        shaders = []
        with open(self.shaderPathPrefix+"graphics/car.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/red.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.carProgram = gr.ShaderProgram(shaders)

        # Car filling program
        # Fill in vehicle
        shaders = []
        with open(self.shaderPathPrefix+"graphics/car.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/darkred.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.carFillingProgram = gr.ShaderProgram(shaders)

        # Cone filling program
        # fill in cone showing approximate "collision field of view"
        shaders = []
        with open(self.shaderPathPrefix+"graphics/cone.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/cone_color.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.coneFillingProgram = gr.ShaderProgram(shaders)

        # Steering angle program
        # Draws a yellow line at the front of the vehicle
        shaders = []
        with open(self.shaderPathPrefix+"graphics/angle.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/yellow.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.angleProgram = gr.ShaderProgram(shaders)

        # Velocity program
        # Draws a green line at the center of the vehicle
        shaders = []
        with open(self.shaderPathPrefix+"graphics/velocity.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/green.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.velocityProgram = gr.ShaderProgram(shaders)

        # Desired velocity program
        # Draws a blue line at the center of the vehicle
        shaders = []
        with open(self.shaderPathPrefix+"graphics/dvelocity.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/blue.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.dVelocityProgram = gr.ShaderProgram(shaders)

        # Obstacle program
        # Draws yellow lines as walls
        shaders = []
        with open(self.shaderPathPrefix+"graphics/wall.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/blue.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.wallProgram = gr.ShaderProgram(shaders)


        # Obstacle program
        # Draws dotted blue lines as road markers
        shaders = []
        with open(self.shaderPathPrefix+"graphics/exitline.vert") as f:
            shaders.append(gr.Shader(header + f.read(), gr.VERTEX_SHADER))
        with open(self.shaderPathPrefix+"graphics/dottedblue.frag") as f:
            shaders.append(gr.Shader(header + f.read(), gr.FRAGMENT_SHADER))
        self.EBWProgram = gr.ShaderProgram(shaders)


        # Distance calculation program
        # Creates a N+M,N sized matrix with distances between vehicle i and object j
        shaders = []
        with open(self.shaderPathPrefix+"distance.comp") as f:
            shaders.append(gr.Shader(header + f.read(), gr.COMPUTE_SHADER))
        self.distanceProgram = gr.ShaderProgram(shaders)

        # Vehicle movement program
        # Calculates the velocity and steering angle from a desired velocity 
        shaders = []
        with open(self.shaderPathPrefix+"vehiclemovement.comp") as f:
            shaders.append(gr.Shader(header + f.read(), gr.COMPUTE_SHADER))
        self.moveProgram = gr.ShaderProgram(shaders)

        # Precalc program
        # Calculates the normals on each obstacle
        shaders = []
        with open(self.shaderPathPrefix+"precalc.comp") as f:
            shaders.append(gr.Shader(header + f.read(), gr.COMPUTE_SHADER))
        self.precalcProgram = gr.ShaderProgram(shaders)

        # Precalc program for the road markers. Ideally implemented in other precalc program
        # Calculates the normals on the road markers
        shaders = []
        with open(self.shaderPathPrefix+"precalcroadmarkers.comp") as f:
            shaders.append(gr.Shader(header + f.read(), gr.COMPUTE_SHADER))
        self.roadMarkerPrecalcProgram = gr.ShaderProgram(shaders)

        self.header = header

    """ Draw vehicles and obstacles
    """
    def _drawObjects(self):
        # Draw approximate "collision cone"
        self.coneFillingProgram.use()
        self.coneVAO.bind()
        gr.drawInstanced(6, self.N)

        # Draw car
        self.carVAO.bind()
        self.carFillingProgram.use()
        gr.drawInstanced(6, self.N, 42)
        self.carProgram.use()
        gr.drawInstanced(42, self.N)

        # Draw steering angle vector
        self.angleProgram.use()
        self.lineVAO.bind()
        gr.drawInstanced(6, self.N)

        # Draw velocity vector
        self.velocityProgram.use()
        gr.drawInstanced(6, self.N)

        # Draw desired velocity vector
        self.dVelocityProgram.use()
        gr.drawInstanced(6, self.N)

        # Draw walls
        self.wallProgram.use()
        self.wallVAO.bind()
        gl.glDrawElements(gl.GL_LINES, self._wallIBuffer.length//4, gl.GL_UNSIGNED_INT, ctypes.c_void_p(0))

        # Draw exit separating dotted lines
        self.EBWProgram.use()
        self.EBWVAO.bind()
        gl.glDrawElements(gl.GL_LINES, self._EBWIBuffer.length//4, gl.GL_UNSIGNED_INT, ctypes.c_void_p(0))

    """ Render pass callback
    """
    def _renderPass(self):
        # Update globalSettingsBuffer
        self.globalSettingsBuffer.setData(self.globalSettings)

        self._bindBuffers()

        self.distanceProgram.dispatch(self.N, self.N+self.M+self.P)
        gl.glMemoryBarrier(gl.GL_SHADER_STORAGE_BARRIER_BIT)

        self.algoPass(self)
        gl.glMemoryBarrier(gl.GL_SHADER_STORAGE_BARRIER_BIT)

        self.moveProgram.dispatch(self.N)
        gl.glMemoryBarrier(gl.GL_SHADER_STORAGE_BARRIER_BIT)

        if self.rendering:
            # Draw objects to screen
            self._drawObjects()

            # Draw GUI
            self.guiPass(self)

        # Increase step count
        self.stepCount += 1
        self.time += self.globalSettings[1][0][0]
        if self.steps>0 and self.stepCount==self.steps:
            # self.window.close()
            self.window.softclose()
            

        # Run dataPass after period
        if self.dataPassPeriod>0 and self.stepCount%self.dataPassPeriod == 0:
            self.dataPass(self)

        if self.inReset:
            self.inReset = False
            imgui.render()
            self._reset()

    """ Window resize callback
    """
    def _resize(self, window):
        ratio = float(self.window.width)/float(self.window.height)
        self.globalSettings[0] = glm.ortho(-ratio*50, ratio*50, -50.0, 50.0).to_list()

    """ Window event callback
    """
    def _onEvent(self, window):
        # Move camera
        moveDir = glm.vec3(0.0)
        if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS:
            moveDir += glm.vec3(0.0, -1.0, 0.0) # 0.0, -1.0, 0.0
        if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS:
            moveDir += glm.vec3(0.0, 1.0, 0.0) # 0.0, 1.0, 0.0
        if glfw.get_key(window, glfw.KEY_A) == glfw.PRESS:
            moveDir += glm.vec3(1.0, 0.0, 0.0) # 1.0, 0.0, 0.0
        if glfw.get_key(window, glfw.KEY_D) == glfw.PRESS:
            moveDir += glm.vec3(-1.0, 0.0, 0.0) # -1.0, 0.0, 0.0
        if glm.length(moveDir)>0:
            moveDir = glm.normalize(moveDir)
            
            # Fix for camera movement.
            translation_matrix = glm.translate(glm.mat4(1.0), 0.2*CAMSPEED*moveDir)
            self.globalSettings[0][3][0] += translation_matrix[3][0] 
            self.globalSettings[0][3][1] += translation_matrix[3][1]
        
        # Zoom camera
        zoom = 1.0
        if glfw.get_key(window, glfw.KEY_Q) == glfw.PRESS:
            zoom *= 1+ZOOMSPEED
            self.zoomLevel *= 1-ZOOMSPEED
        if glfw.get_key(window, glfw.KEY_E) == glfw.PRESS:
            zoom *= 1-ZOOMSPEED
            self.zoomLevel *= 1+ZOOMSPEED
        if zoom!=1.0:
            self.globalSettings[0] = glm.scale(self.globalSettings[0], glm.vec3(zoom))

        if glfw.get_key(window, glfw.KEY_ESCAPE):
            self.window.close()

    """ Reset simulation
    """
    def reset(self):
        self.inReset = True

    """ Internal reset function
    """
    def _reset(self):
        # Initialize algorithm by calling algoInit function
        wallVertices, simState, EBWVertices, locationMarkerCoords, visitedList = self.algoInit(self)
        # Create obstacle vertex and index buffers
        self._wallVBuffer.setData(wallVertices)
        wallIndices = np.arange(0, len(wallVertices), 1, dtype="uint32")
        self._wallIBuffer.setData(wallIndices)
        self.M = len(wallVertices)//4           # M = amount of obstacles

        self._EBWVBuffer.setData(EBWVertices)
        EBWIndices = np.arange(0, len(EBWVertices), 1, dtype = "uint32")
        self._EBWIBuffer.setData(EBWIndices)
        self.P = len(EBWVertices)//4            # P = amount of fake walls

        self._visitedBuffer.setData(visitedList)

        self._locMarkBuffer.setData(locationMarkerCoords)
        # self.L = 4

        print("N = %d, M = %d"%(self.N, self.M))

        # Set global settings
        self._bindBuffers()
        self.globalSettings[1][0][1] = float(self.N)
        self.globalSettings[1][0][2] = float(self.M)
        self.globalSettings[1][2][2] = float(self.P)
        self.globalSettings[1][2][3] = float(self.L)
        self.globalSettingsBuffer.setData(self.globalSettings)
        gl.glMemoryBarrier(gl.GL_UNIFORM_BARRIER_BIT)

        # Reserve space for M dependent buffers
        self.distanceBuffer.reserveData(4*2*self.N*(self.N+self.M+self.P))
        self.wallInfoBuffer.reserveData(4*1*self.M)
        self.roadMarkingBuffer.reserveData(4*1*self.P)

        # Zero out simStateBuffer
        #simState = np.zeros([self.N, 4], dtype="uint32")
        self.simStateBuffer.setData(simState)

        # Execute precalc shader
        # This will calculate obstacle normals
        self.precalcProgram.dispatch(self.M)
        gl.glMemoryBarrier(gl.GL_SHADER_STORAGE_BARRIER_BIT)

        self.roadMarkerPrecalcProgram.dispatch(self.P)
        gl.glMemoryBarrier(gl.GL_SHADER_STORAGE_BARRIER_BIT)

        # Run the simulation
        self.stepCount = 0
        self.time = 0.0
        self.window.run()