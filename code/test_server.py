

import socket
import struct

s = socket.socket()
host = socket.gethostname()
port = 4570
s.bind( ( host, port ) )

print( 'waiting for someone...' )
s.listen( 1 )
c, addr = s.accept()
print( 'Got connection from : ', addr )

def decode( strData, bytesStream ) :
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
            _res.append( struct.unpack( 'f', _floatBStr ) )

    return _res

while True :

    data = c.recv( 1024 )

    if data :
        print( 'data: ', data )
        print( 'type(data): ', type( data ) )
        print( 'dataStr: ', str( data ) )
        decodedData = decode( str( data ), data )
        print( 'decodedData: ', decodedData )
c.close()