# Status of Connection with Sim Server, in Initsim.Request / Recv

uint8 STATUS_READY = 0      # simulation is ready
uint8 STATUS_RUNNING = 1    # simulation is running
uint8 STATUS_DONE = 2       # simulation is done
uint8 STATUS_BUSY = 3       # simulation is preparing
uint8 STATUS_ERROR = 4      # sim server response with error

uint8 status
string<=256 msg