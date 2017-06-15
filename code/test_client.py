
import socket
import time
import struct

s = socket.socket()
host = socket.gethostname()
port = 4570

print( 'connecting...' )
s.connect( ( host, port ) )
print( 'connected!' )

sample_packet = ['s', 'o', 'k', 'n', 4, 1.23, 2.4, 1.34, 2.7]

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

_sStr = encode( sample_packet )

while True :
    ## s.send( ( 'snp' + chr( 62 ) ).encode( 'utf-8' ) )
    s.send( _sStr )
    print( 'sent : ', str( _sStr ) )
    print( 'nfloats: ', _sStr[4] )
    time.sleep( 2 )

s.close()
