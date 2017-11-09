

import numpy as np
import matplotlib.pyplot as plt
from RoverFSMStates import *
import socket
import struct

class RoverAI :

    def __init__( self, agent ) :

        self.agent = agent
        self.m_roverData = None

    def update( self, dt, roverData ) :
        self.m_roverData = roverData


class RoverAI_FSM( RoverAI ) :

    ST_LOOKING_FOR_PATH = 'lookingForPath'
    ST_FORWARD          = 'forward'
    ST_BRAKING          = 'braking'
    ST_REACHING_ROCK    = 'reachingRock'
    ST_PICKING_ROCK     = 'pickingRock'
    ST_TEST             = 'test'

    def __init__( self, agent ) :
        super( RoverAI_FSM, self ).__init__( agent )
        self.m_states = {}
        self.m_states[RoverAI_FSM.ST_LOOKING_FOR_PATH]  = STLookingForPath( self )
        self.m_states[RoverAI_FSM.ST_FORWARD]           = STForward( self )
        self.m_states[RoverAI_FSM.ST_BRAKING]           = STBraking( self )
        self.m_states[RoverAI_FSM.ST_REACHING_ROCK]     = STReachingRock( self )
        self.m_states[RoverAI_FSM.ST_PICKING_ROCK]      = STPickingRock( self )
        self.m_states[RoverAI_FSM.ST_TEST]              = STTest( self )
        
        self.m_currentState = None
        self.m_currentStateId = ''


        self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

    def setCurrentState( self, stateId ) :
        if ( self.m_currentState != None ) :
            self.m_currentState.onExit()

        self.m_currentState = self.m_states[stateId]
        self.m_currentStateId = stateId
        self.m_currentState.onEnter()

    def update( self, dt, roverData ) :
        super( RoverAI_FSM, self ).update( dt, roverData )
        ## print( 'RoverAI_FSM::update> ', dt, ' currentState: ', self.m_currentStateId )

        if ( self.m_currentState != None ) :
            self.m_currentState.update( dt, roverData )
            if ( self.m_currentState.state == RoverFSMState.ST_FINISHED ) :
                
                # go to next state
                if self.m_currentStateId == RoverAI_FSM.ST_LOOKING_FOR_PATH :
                    if self.m_currentState.status == 'found_navigable_area' :
                        self.setCurrentState( RoverAI_FSM.ST_FORWARD )
                    else :
                        self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

                elif self.m_currentStateId == RoverAI_FSM.ST_FORWARD :
                    if self.m_currentState.status == 'no_navigable_area' :
                        self.setCurrentState( RoverAI_FSM.ST_BRAKING )
                    elif self.m_currentState.status == 'rock_in_area' :
                        self.setCurrentState( RoverAI_FSM.ST_REACHING_ROCK )

                elif self.m_currentStateId == RoverAI_FSM.ST_BRAKING :
                    if self.m_currentState.status == 'fully_stopped' :
                        self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

                elif self.m_currentStateId == RoverAI_FSM.ST_REACHING_ROCK :
                    if self.m_currentState.status == 'rock_reachable' :
                        self.setCurrentState( RoverAI_FSM.ST_PICKING_ROCK )
                    elif self.m_currentState.status == 'rock_out_of_range' :
                        self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

                elif self.m_currentStateId == RoverAI_FSM.ST_PICKING_ROCK :
                    if self.m_currentState.status == 'rock_picked' :
                        self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )


class RoverAI_BT( RoverAI ) :

    def __init__( self, agent ) :
        super( RoverAI_BT, self ).__init__( agent )


class PIDController :

    def __init__( self, Kp = 5.0, Kd = 4.0, Ki = 0.001 ) :

        self.epv = 0.0
        self.eiv = 0.0
        self.edv = 0.0

        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def reset( self ) :
        self.epv = 0.0
        self.eiv = 0.0
        self.edv = 0.0

    def calculate( self, x, xRef, verbose = False ) :
        _epv = x - xRef
        self.edv = _epv - self.epv
        self.epv = _epv
        self.eiv += _epv
        _u = -( self.Kp * self.epv + self.Kd * self.edv + self.Ki * self.eiv )
        if ( verbose ) :
            print( 'x,xRef: ', x, xRef, ' u: ', _u )

        return _u

class RoverMotionController :

    def __init__( self ) :

        self.m_speedController = PIDController()
        self.m_steerController = PIDController( 15.0, 15.0, 0.0 )
        self.ai = RoverAI_FSM( self )

    def update( self, dt, roverData ) :
        self.ai.update( dt, roverData )

    def restartNavigationController( self ) :
        self.m_speedController.reset()

    def restartSteerController( self ):
        self.m_steerController.reset()

    def navigationController( self, v, theta, vRef, thetaRef ) :
        u_throttle = self.m_speedController.calculate( v, vRef, False )
        u_brake = 0
        if u_throttle < 0 :
            u_brake = np.clip( -u_throttle ,0, 10 )

        u_throttle = np.clip( u_throttle, 0, 0.2 )
        u_steer = np.clip( thetaRef, -15, 15 )

        return [u_throttle,u_brake,u_steer]

    def steerController( self, theta, thetaRef ) :
        u_steer = self.m_steerController.calculate( theta, thetaRef, True )
        u_steer = np.clip( u_steer, -15, 15 )

        return [0,0,u_steer]

    def positionController( self, xRef, yRef ) :
        u_throttle = 0
        u_steer = 0
        return [u_throttle,u_brake,u_steer]

g_roverController = RoverMotionController()

"""
g_socket = socket.socket()
g_host = socket.gethostname()
g_port = 4571
g_status = 'idle'
g_socket.connect( ( g_host, g_port ) )
"""

def onBroadcast( dataPacket ) :
    global g_socket, g_host, g_port, g_status
    print( 'broadcasting' )
    g_socket.send( dataPacket )
    
    

TYPE_STR = type('')
TYPE_INT = type(0)
TYPE_FLOAT = type(.0)

def encode( packet ) :
    _res = b''
    for q in range( len( packet ) ) :
        if type( packet[q] ) == TYPE_STR :
            _res += packet[q].encode( 'utf-8' )
        elif type( packet[q] ) == TYPE_INT :
            print( 'encoding ', packet[q], ' floats' )
            _res += chr( packet[q] ).encode( 'utf-8' )
        elif type( packet[q] ) == TYPE_FLOAT :
            _res += struct.pack( 'f', packet[q] )
    return _res

def encodeData() :
    global g_roverState
    _packet = ['s']

    if g_roverState == None:
        _encoded_str += 'xxx'
        return _encoded_str

    _packet.append( 'o' )
    _packet.append( 'k' )

    if g_roverState.navigationPath :
        encodeNavPathData( g_roverState.navigationPath, _packet )
    if g_roverState.navigationMesh :
        encodeNavMeshData( g_roverState.navigationMesh, _packet )

    return encode( _packet )

def encodeNavPathData( navPath, workingpacket ) :
    workingpacket.append( 'n' )
    _pts = navPath.getPoints()
    _numPoints = len( _pts )
    workingpacket.append( _numPoints * 2 )
    for q in range( _numPoints ) :
        workingpacket.append( _pts[q][0] )
        workingpacket.append( _pts[q][1] )

def encodeNavMeshData( navMesh, workingpacket ) :
    pass

g_roverState = None

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    global g_roverController, g_socket, g_roverState

    g_roverState = Rover

    ### _dataPacket = encodeData()
    ### onBroadcast( _dataPacket )

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    g_roverController.update( Rover.time_struct['delta'], Rover )

    """
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    """
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

