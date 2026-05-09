# Status of Connection with Sim Server, in Initsim.Request / Recv

uint8 READY = 0      # simulation is ready
uint8 RUNNING = 1    # simulation is running
uint8 DONE = 2       # simulation is done
uint8 BUSY = 3       # simulation is preparing
uint8 ERROR = 4      # sim server response with error
uint8 FAILURE = 5    # simulation is close with error
uint8 MAX = 6        # invalid boundary

uint8 status
string<=256 msg
