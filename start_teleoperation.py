import time
from utils.processes.fsm import FSM
time_EXIT = 5000
FSM.activate_threads(FSM)
time.sleep(time_EXIT)
FSM.deactivate_threads(FSM)
