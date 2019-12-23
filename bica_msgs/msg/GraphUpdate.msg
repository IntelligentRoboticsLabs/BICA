uint8 ADD=0
uint8 REMOVE=1
uint8 SYNC=2
uint8 REQSYNC=3

uint8 NODE=0
uint8 EDGE=1
uint8 GRAPH=2

builtin_interfaces/Time stamp
string node_id

uint8 operation_type
uint8 element_type

# seq is only used for GRAPH 
uint32 seq
string object

