

import numpy as np
import socket
import struct
import time
import sys

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

class CPlotWindow( QtGui.QMainWindow ) :

    def __init__( self, parent = None ) :
        super( CPlotWindow, self ).__init__( parent )

        self.m_view = pg.GraphicsLayoutWidget()
        self.setCentralWidget( self.m_view )
        
        self.setWindowTitle( 'Rover plotting' )

        self.m_plotArea = self.m_view.addPlot()
        self.m_plotArea.setXRange( -200, 200 )
        self.m_plotArea.setYRange( -200, 200 )

        self.m_scatterPlot = pg.ScatterPlotItem( size=10, 
                                                 pen=pg.mkPen( None ), 
                                                 brush=pg.mkBrush( 255, 255, 255, 120 ) )
        self.m_plotArea.addItem( self.m_scatterPlot )

        self.m_timer = QtCore.QTimer()
        self.connect( self.m_timer, QtCore.SIGNAL( 'timeout()' ), self.onUpdateGraph )
        self.m_timer.start( 10 )

        self.show()

    def decode( self, strData, bytesStream ) :
        _res = []
        print( 'decoding: ', bytesStream )
        if strData[2] == 's' \
            and strData[3] == 'o'\
            and strData[4] == 'k' :
            # received good packet
            _numComps = bytesStream[4]
            print( 'decoding ', _numComps, ' floats' )
            for q in range( _numComps ) :
                _floatBStr = bytesStream[(5 + 4 * q):(5 + 4 * q + 4 )]
                _res.append( struct.unpack( 'f', _floatBStr )[0] )

        return _res

    def onUpdateGraph( self ) :
        data = c.recv( 1024 )
        if data :
            decodedData = self.decode( str( data ), data )
            self.updateData( decodedData, None )


    def updateNavPath( self, navPathPoints ) :
        _pathPts = []
        _numPts = int( len( navPathPoints ) / 2 )
        _xx = []
        _yy = []
        for q in range( _numPts ) :
            _pathPts.append( ( navPathPoints[2 * q],
                               navPathPoints[2 * q + 1] ) )
            _xx.append( navPathPoints[2 * q] )
            _yy.append( navPathPoints[2 * q + 1] )

        self.m_scatterPlot.addPoints( x = _xx, y = _yy )

    def updateNavMesh( self, navMeshGraph ) :
        pass

    def updateData( self, navPathPoints, navMeshGraph = None ) :
        self.m_scatterPlot.clear()

        self.updateNavPath( navPathPoints )
        self.updateNavMesh( navMeshGraph )

s = socket.socket()
host = socket.gethostname()
port = 4571
s.bind( ( host, port ) )

print( 'waiting for someone...' )
s.listen( 1 )
c, addr = s.accept()
print( 'Got connection from : ', addr )

app = QtGui.QApplication( sys.argv )

mWindow = CPlotWindow()

app.exec_()

