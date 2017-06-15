

import numpy as np


class RoverFSMState :

    def __init__( self, parent ) :
    	self.parent = parent

    def onEnter( self ) :

    	pass

    def onExit( self ) :

    	pass

    def update( self, dt, roverData ) :

        pass


class STLookingForPath ( RoverFSMState ) :

    def __init__( self, parent ) :
    	super( STLookingForPath, self ).__init__( parent )

    def update( self, dt, roverData ) :
    	print( 'STLookingForPath::update> dt: ', dt )


class STForward ( RoverFSMState ) :

	def __init__( self, parent ) :
		super( STForward, self ).__init__( parent )

	def update( self, dt, roverData ) :

		pass

class STBraking ( RoverFSMState ) :

	def __init__( self, parent ) :
		super( STBraking, self ).__init__( parent )


	def update( self, dt, roverData ) :

		pass



class STReachingRock ( RoverFSMState ) :
	
	def __init__( self, parent ) :
		super( STReachingRock, self ).__init__( parent )
		

	def update( self, dt, roverData ) :

		pass


class STPickingRock ( RoverFSMState ) :

	def __init__( self, parent ) :
		super( STPickingRock, self ).__init__( parent )
		

	def update( self, dt, roverData ) :

		pass