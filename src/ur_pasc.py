from threading import Thread, Condition, Lock, Event
import logging
import struct
import socket
import time
import numpy as np

__author__ = 'Kamil Skop'
__copyright__ = 'Copyright 2022-2023, Kamil Skop'
__license__ = 'MIT'
# const 
PORT  = 30002 

class ParsingException(Exception):
    def __init__(self,*args):
        super().__init__(*args)
        
class PrimSecoClinetException(Exception):
    def __init__(self,*args):
        super().__init__(*args)

class Package_Parser():
    def __init__(self,version:tuple):
        self.logger = logging.getLogger('ur_pasc')
        self.version = version
        self._param_dict = self._init_dict()
        
    def _init_dict(self) -> None:
        # RobotModeData
        if self.version == (1, 8):
            robot_mode_data = {'packageSize':None, 'packageType':None, 'timestamp':None, 'isRobotConnected':None, 'isRealRobotEnabled':None, 'isPowerOnRobot':None, 'isEmergencyStopped':None, 'isSecurityStopped':None, 'isProgramRunning':None, 'isProgramPaused':None, 'robotMode':None, 'speedFraction':None}
        elif self.version == (3,7):
            robot_mode_data = {'packageSize':None, 'packageType':None, 'timestamp':None, 'isRealRobotConneted':None,'isRealRobotEnabled':None, 'isRobotPowerOn':None, 'isEmergencyStopped':None, 'isProtectiveStopped':None, 'isProgramRunning':None,'isProgramPaused':None, 'robotMode':None, 'controlMode':None, 'targetSpeedFraction':None, 'speedScaling':None, 'targetSpeedFractionLimit':None}
        # JointData
        joint_data = {'packageSize':None, 'packageType':None}
        for i in range(0, 6):
            joint_data.update({('q_actual%s' % i):None, ('q_target%s' % i):None, ('qd_actual%s' % i):None, ('I_actual%s' % i):None, ('V_actual%s' % i):None, ('T_motor%s' % i):None, ('T_micro%s' % i):None, ('jointMode%s' % i):None})
        # CartesianInfo   
        cartesianinfo = {'packageSize':None, 'packageType':None, 'X':None, 'Y':None, 'Z':None, 'Rx':None, 'Ry':None, 'Rz':None}
        # MasterboardData
        masterboard_data = {'packageSize':None, 'packageType':None, 'digitalInputBits':None, 'digitalOutputBits':None, 'analogInputRange0':None, 'analogInputRange1':None, 'analogInput0':None, 'analogInput1':None, 'analogOutputDomain0':None, 'analogOutputDomain1':None, 'analogOutput0':None, 'analogOutput1':None, 'masterBoardTemperature':None, 'robotVoltage48V':None, 'robotCurrent':None, 'masterIOCurrent':None} 
        # ToolData
        tool_data = {'packageSize':None, 'packageType':None, 'analoginputRange2':None, 'analoginputRange3':None, 'analogInput2':None, 'analogInput3':None, 'toolVoltage48V':None, 'toolOutputVoltage':None, 'toolCurrent':None, 'toolTemperature':None, 'toolMode':None}
        
        # Prepared dict
        builded_dict = {'RobotModeData':robot_mode_data,'JointData':joint_data,'CartesianInfo':cartesianinfo, 'MasterboardData':masterboard_data, 'ToolData':tool_data} 
        return builded_dict
    
    def parser (self, pdata, ptype):
        if ptype == 0:
            pack_type = 'RobotModeData'
            if self.version == (1, 8):
                vars_type = '!iBQ???????Bd'
            elif self.version == (3,7):
                vars_type = '!iBQ???????BBddd'
        elif ptype == 1:
            pack_type = 'JointData'
            vars_type = '!iB dddffffB dddffffB dddffffB dddffffB dddffffB dddffffB'
        elif ptype == 4:  
            pack_type = 'CartesianInfo'
            vars_type = '!iBdddddd'
        elif ptype == 3:
            pack_type = 'MasterboardData'
            if self.version == (1, 8):
                vars_type = 'iBhhbbddbbddffffBBb'
            elif self.version == (3, 7):
                vars_type = 'iBiibbddbbddffffBBb'
        elif ptype == 2:
            pack_type = 'ToolData'
            vars_type = 'iBbbddfBffB'
        else:
            pack_type = None
            vars_type = None
            
        # unpacking data depend on 'packageType' info
        if pack_type and vars_type:    
            self._unpack_data(pdata, vars_type, pack_type)
            
        #self.logger.debug('Updated package:{}'.format(self._param_dict))
        return self._param_dict.copy()
    
    def _unpack_data(self, data, unpack_key:str, dict_key:str):
        cor_index = 0
        for i, des in enumerate(self._param_dict[dict_key]):
            while unpack_key[i + cor_index] in ('@','=','<','>','!', ' '):
                 cor_index += 1
            var_size = struct.calcsize(unpack_key[i+cor_index])
            if len(data) < var_size:
                raise ParsingException('Error, length of parsing data smaller than expected dict_key:{} psize:{}'.format(des,len(data)))
    
            self._param_dict[dict_key][des] = struct.unpack('!'+unpack_key[i+cor_index], data[0:var_size])[0]
            data = data[var_size:]
        #self.logger.debug('Decode data:{}'.format(tmp_dict))       
    
class UR_PrimSecoClient(Thread):
    def __init__(self,HOST:str, version:tuple):
        super().__init__()
        self._version = version
        self.logger = logging.getLogger('ur_pasc')
        self._recvqueue = bytes()
        self._param_dict_lock = Lock()
        self._prog_wait_event = Event()
        self._status_counter = 0
        self._progqueue_lock = Condition()
        self._encode_data  = {}
        self._prog_list = []
        self.event = Event()
        self._parser = Package_Parser(self._version)
        
        self._scon = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self._scon.settimeout(5)   # 5 seconds
        try:
            self._scon.connect((HOST, PORT))
        except socket.error as err:
            raise PrimSecoClinetException('Error, _scon.connect((HOST, PORT)):{}'.format(err))
               
    def _analyze_header(self, data):
        '''
        Function which unpack first 5 bytes (package header) and return overall length and type of package
        '''
        if len(data) < 5:
            raise ParsingException('_analyze_header:Error! Package size: {} smaller then declarated'.format(len(data)))
        msg_size, msg_type = struct.unpack('!iB', data[0:5])
        
        return msg_size, msg_type
        
    def _find_main_header(self, data): 
        # Only the message type 'ROBOT_STATE'is documented (messageType value is 16).
        # So we have to find pakage with this header
        # Message fomating:
        #    4 bytes (int) - Length of overall package 
        #    1 byte (uchar) - Robot MessageType: MESSAGE_TYPE_ROBOT_STATE = 16
        msg_size, msg_type = self._analyze_header(data)
        #self.logger.debug('Readed data - msg_size:{}, msg_type:{}'.format(msg_size, msg_type))
        if msg_type == 16 and msg_size == len(data):
            #self.logger.debug('_find_main_header:Complete package founded:msg_size:{},msg_type:{}'.format(msg_size, msg_type))
            return data[5:]
        else:
            self.logger.warning('_find_main_header:Package not found in data')
            return None 
         
    def _get_subpackage(self):
        while True:
            data = self._find_main_header(self._scon.recv(1024))
            if data:
                #self.logger.debug('Start parsing package, data:{}'.format(len(data)))
                while len(data) > 5:
                    sub_size, sub_type = self._analyze_header(data)
                    #self.logger.debug('Readed header: sub_size:{}, sub_type:{}'.format(sub_size, sub_type))
                    with self._param_dict_lock:
                        self._encode_data = self._parser.parser(data[:sub_size],sub_type)
                    data = data[sub_size:]
                return self._encode_data
            
    def add_to_sendqueue(self,prog:str) -> None:
        prog.strip()
        prog = prog.encode('ascii')
           
        with self._progqueue_lock:
            self._prog_list.append(prog)
            self._progqueue_lock.wait()
        self._prog_wait_event.wait()
        self._prog_wait_event.clear()
        
    def close(self)->None:
        self.event.set()
        self.join()
        self._scon.close()
  
    def run(self):
        MAX_CNT = 2
        while not self.event.is_set():
            with self._progqueue_lock:
                if not self._prog_list == []:
                    self.logger.debug('Sended package:{}'.format(self._prog_list))
                    self._scon.send((self._prog_list.pop(0)))
                    self._prog_wait_event.clear()
                    self._progqueue_lock.notify_all()     
            self._get_subpackage()
            with self._param_dict_lock:
                status = self._encode_data['RobotModeData']['isProgramRunning']
                if self._prog_wait_event and not status:
                    self._status_counter += 1
                else:
                    self._status_counter = 0
                if self._status_counter >= MAX_CNT:
                    self._status_counter = 0
                    self._prog_wait_event.set()
            
    def get_all_data(self):
        with self._param_dict_lock:
            return self._encode_data.copy()
        
    def get_joint_pose(self, mode = 'rad') -> list:
        with self._param_dict_lock:
            tmp = []
            for i in range(6):
                if mode == 'deg':
                    q = np.rad2deg(self._encode_data['JointData']['q_actual{}'.format(i)])
                else:
                    q = self._encode_data['JointData']['q_actual{}'.format(i)]
                tmp.append(q)
        return tmp.copy()
    
    def get_xyz_pose(self, mode:str = 'rad') -> list:
        with self._param_dict_lock:
            tmp = []
            for i in self._encode_data['CartesianInfo']:
                if i not in ('packageSize', 'packageType'):
                    if i in ('Rx','Ry','Rz') and mode == 'deg':
                        q = np.rad2deg(self._encode_data['CartesianInfo'][i])
                    else:
                        q = self._encode_data['CartesianInfo'][i]
                    tmp.append(q)
        return tmp.copy()
    def _binary_repr(self, data, len) -> list:
        binary = []
        for _ in range(18):
            binary.append(data&1)
            data>>=1 
        return binary 
     
    def get_digital_in(self) -> list:
        with self._param_dict_lock:
            return self._binary_repr(self._encode_data['MasterboardData']['digitalInputBits'],18)
        
    def get_digital_out(self) -> list:
        with self._param_dict_lock:
            return self._binary_repr(self._encode_data['MasterboardData']['digitalOutputBits'],18)
        
    def get_analog_out(self) -> list:
        analog_out_list = []
        with self._param_dict_lock:
            for i in range(2): 
                analog_out_list.append([self._encode_data['MasterboardData']['analogOutput{}'.format(i)], self._encode_data['MasterboardData']['analogOutputDomain{}'.format(i)]])
        return analog_out_list
    
    def get_analog_in(self):
        analog_out_list = []
        with self._param_dict_lock:
            for i in range(2): 
                analog_out_list.append([self._encode_data['MasterboardData']['analogInput{}'.format(i)], self._encode_data['MasterboardData']['analogInputRange{}'.format(i)]])
        return analog_out_list
    
    def get_is_program(self):
        with self._param_dict_lock:
            return self._encode_data['RobotModeData']['isProgramRunning']
    
        
if __name__ == '__main__':
    # logging.getLogger().setLevel(logging.DEBUG)

  
    