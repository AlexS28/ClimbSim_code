# Author: Alexander Schperberg, aschperb@gmail.com
# Date: 2023-03-27

import multiprocessing
import utils.processes.pybullet_process as pybullet
import time

# initialize multiprocessing events
event_shutdown_PYBULLET = multiprocessing.Event()

class FSM(object):
    @staticmethod
    def start_PYBULLET():
        p_LSE = multiprocessing.Process(target=pybullet.activate, args=(event_shutdown_PYBULLET,))
        p_LSE.start()
        print('PYBULLET ACTIVATED')
        return p_LSE

    @staticmethod
    def stop_PYBULLET(p_PYBULLET):
        event_shutdown_PYBULLET.set()
        time.sleep(0.5)
        p_PYBULLET.terminate()
        print('PYBULLET DEACTIVATED')

    @staticmethod
    def activate_threads(self):
        self.p_PYBULLET = FSM.start_PYBULLET()

    @staticmethod
    def deactivate_threads(self):
        try:
            FSM.stop_PYBULLET(self.p_PYBULLET)
        except:
            pass