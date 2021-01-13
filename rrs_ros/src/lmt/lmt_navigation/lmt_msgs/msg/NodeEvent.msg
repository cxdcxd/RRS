##
## Event type constants
##
byte NODE_STARTED=1
byte NODE_DIED=2
byte NODE_PAUSED=4
byte NODE_CONTINUED=8
byte NODE_SEGFAULT=16
##
## Fields
##
Header header
byte event_type
string node_name
string message
