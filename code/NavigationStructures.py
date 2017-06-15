
import numpy


class RoverNavigationMap :

    def __init__( self ) :

        pass

    def update( self, roverData ) :

        pass

class RoverNavigationMesh :

    def __init__( self ) :

        pass

class RoverNavigationPath :

    SAMPLE_TIME = 2.0
    DIST_THRESHOLD = 10

    def __init__( self ) :
        self.m_sampleTimer = 0.0
        self.m_ptsPath = []
        self.m_lastPt = None

    def encode( self ) :
        _data = { 'path': [], 'lastPoint': None }
        for q in range( len( self.m_ptsPath ) ) :
            _pt = ( self.m_ptsPath[q][0], self.m_ptsPath[q][1] )
            _data['path'].append( _pt )
        if self.m_lastPt != None :
            _data['lastPoint'] = ( self.m_lastPt[0], self.m_lastPt[1] )

        return _data


    def getPoints( self ) :
        return self.m_ptsPath

    def dist( self, p1, p2 ) :
        return ( ( p1[0] - p2[0] ) ** 2 + ( p1[1] - p2[1] ) ** 2 )

    def update( self, dt, roverData ) :

        self.m_sampleTimer += dt
        if ( self.m_sampleTimer > RoverNavigationPath.SAMPLE_TIME ) :
            self.m_sampleTimer = 0

            _newPt = ( roverData.pos[0], roverData.pos[1] )
            if self.m_lastPt == None :
                self.m_lastPt = ( _newPt[0], _newPt[1] )
                self.m_ptsPath.append( _newPt )
            else :
                if self.dist( self.m_lastPt, _newPt ) > RoverNavigationPath.DIST_THRESHOLD :
                    self.m_lastPt = ( _newPt[0], _newPt[1] )
                    self.m_ptsPath.append( _newPt )





