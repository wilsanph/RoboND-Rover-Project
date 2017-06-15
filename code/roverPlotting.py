

import matplotlib.pyplot as plt
import socket


class NavigationPlot :


    def __init__( self ) :

        self.m_fig = plt.figure()
        self.m_ax = self.m_fig.add_subplot( 1, 1, 1 )
        self.m_ax.set_xlim( -200, 200 )
        self.m_ax.set_ylim( -200, 200 )
        self.m_ax.hold( True )

        self.blocked = False

        print( 'initialized plot ' )

    def updateNavPath( self, navPathData ) :
        _pathPts, _lastPt = self.getPointsFromData( navPathData )
        self.m_ax.plot( _pathPts, 'o' )

    def getPointsFromData( self, navPathData ) :
        return navPathData['path'], navPathData['lastPoint']

    def updateNavMesh( self, navMeshData ) :

        pass

    def updateData( self, roverData ) :
        self.blocked = True
        # clear the current axes
        self.m_ax.clear()
        self.updateNavPath( roverData['navPathData'] )
        ## self.updateNavMesh( roverData['navMeshData'] )
        print( 'plotting' )
        plt.show()
        self.blocked = False


g_navigationPlot = NavigationPlot()
