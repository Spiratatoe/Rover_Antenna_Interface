# Rover_Antenna_Interface


The RAI PIF specifies how the BAC and RAC, and on-board software interact through UART links and/or abstract software layers. 

The BAC and RAC interact with the production (rover) or testing system through respective adapters via UART links. Information exchange takes place over UART as full-duplex byte streams. PIF also specifies standards regarding the protocol used. 

Lower layers of PIF do not distinguish between the base station and the rover - the protocol stack implementation on the two parties are identical. Different applications drive the protocol stack to send/receive different data. 

PIF assumes the underlying transport layers have enough reliability, however in reality, data/packet loss may happen. PIF is designed to be insensitive to occasional data loss, but in corner cases we must assume PIF is best-effort rather than reliable. 

RAI PIF is organized as 4 layers:
UART layer: This layer sends and receives raw bytes (octet-synchronized stream). 
Data link layer: This layer organizes data chunks into structured frames, and searches valid frames from inbound byte streams. 
Network layer: This layer generates and parses structured packets that are intended to represent different types of data. 
Application layer: The end application utilizes layers below to implement application functionality. 

PIF defines 1 built-in application and expects 3 external applications:
Session manager (software, built-in): Reporting network connection status between the rover and the base station. 
Base station (firmware): Reporting base station coordinates and discovering rover coordinates. 
Rover (firmware): Reporting rover coordinates and discovering base station coordinates. 
UI (software): Collecting rover coordinates and system status for display (HTTP API). 


PIF layer top-down overview

Layer Name
Takes in what
Transmits as what
Application layer
Application input (GPS, etc)
Message ID & payload
Network layer
Message ID (number), payload (byte array)
Data chunk (byte array)
Data link layer (LFP)
Data chunk (byte array)
Raw bytes (frame)
UART layer
A byte (8 bits)
Electrical signals



PIF layer bottom-up overview

Layer Name
Takes in what
Converts as what
UART layer
Electrical signals
A byte
Data link layer (LFP)
Stream of bytes
Data chunk (byte array)
Network layer
Data chunk (byte array)
Message ID (number), payload (byte array
Application layer
Message ID & payload
Application actions (motor moving, etc)


