import logging
import time
import json
import os
import re
import cv2 as cv
import numpy as np
import math
from ur_pasc import UR_PrimSecoClient 
from threading import Thread, Event, Lock
from queue import Queue
import socket, select

__author__ = "Kamil Skop"
__copyright__ = "Copyright 2022-2023, Kamil Skop"
__license__ = "MIT"

class UR_Exception(Exception):
    def __init__(self,*args):
        super().__init__(*args)   
        
class UR_robot:
    def __init__(self, hostname, version = (3,7)):
        self.logger = logging.getLogger('ur_sc')  
        self.HOST = hostname   
        self.version = version
        self.terminal = UR_PrimSecoClient(self.HOST,self.version)
        self.terminal.start()
        self.poses_dict = {}
        # variables for tcp_client_listener()
        self._client_socket = socket.socket()
        self._client_socket_lock = Lock()
        self._socket_event = Event()
        # variables for digital_in_listener()
         
    def _thread_creator(fn):
        def wrapper(*args, **kwargs):
            thread = Thread(target=fn, args=args, kwargs=kwargs)
            thread.start()
        return wrapper
    
    @_thread_creator
    def tcp_client_listener(self, HOST, PORT, queue:Queue):
        self._client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        MAX_CNT_ERROR = 3
        try:
            self._client_socket.connect((HOST, PORT))
            cnt_error = 0
            while not self._socket_event.is_set():
                ready = select.select([self._client_socket],[],[])
                if ready:
                    with self._client_socket_lock:
                        data = self._client_socket.recv(1024) 
                 
                if data == b'' : cnt_error += 1 
                if cnt_error >= MAX_CNT_ERROR: self.reconnect(HOST,PORT,queue)
                
                self.logger.debug('tcp_client_listener:Received data:{}'.format(data))
                # Saving received data to Queue structure
                if data:
                    queue.put(data)
                    
            self._client_socket.close()
            
        except socket.error as err:
            raise TypeError('Error, _scon.connect((HOST, PORT)):{}'.format(err))
    
    def tcp_client_send_msg(self, msg:str) -> None:
        try:
            with self._client_socket_lock:
                self._client_socket.sendall(msg.encode('ascii'))
                self.logger.debug('tcp_client_send_msg:Sended msg:{}'.format(msg))
        except:
            raise TypeError('tcp_client_send_msg: Error msg:{} cannot be send'.format(msg))   
                          
    def reconnect (self,HOST:str, PORT:int, queue:Queue) -> None:
        self.logger.debug('reconnect: Try to reconnect')
        self.socket_close()
        self.tcp_client_listener(HOST, PORT, queue)
        
    def socket_close(self) -> None:
        self._socket_event.set()
        
    @_thread_creator
    def digital_in_listener(self, nr:int, event:Event, time_interval = 1) -> None:
        # signal from Mobile Omron  DI_in_2
        if not nr > 17:
            while True:
                tab = self.terminal.get_digital_in()
                if (tab[nr]):
                    event.set()
                time.sleep(time_interval)
                    
    def close (self):
        self.terminal.close()
        self._client_socket.close()
        
    def get_poses_dict(self):
        return self.poses_dict.copy()
    
    def add_currrent_pose (self, key:str, mode = 'p') -> None:
        '''
        Add new pose to 'poses_dict' container
        '''
        if key not in self.poses_dict:
            if mode == 'j':
                pose = self.terminal.get_joint_pose()
                pose.append('j')
            else:   
                pose = self.terminal.get_xyz_pose()
                pose.append('p')
            if pose:
                self.poses_dict[key] = pose
            else:
                self.logger.warning('add_current_pose: funtion not received data:{}'.format(pose) )
    
    def add_pose (self, key:str, data:list) -> None:
        '''
        Add pose directly
        '''
        if len(data) == 7 and data[-1] in ('j','p'):
            self.poses_dict[key] = data
    def update_pose (self, key:str, data:list) -> None:
        '''
        Update indicated value in defined pose 
        '''
        if key in self.poses_dict and len(data) == 7 and data[-1] in ('j','p'):
            self.poses_dict[key] = data
        
    def remove_pose (self, key:str) -> None:
        '''
        Remove indicated pose from 'poses_dict' container
        '''
        if key in self.poses_dict:
            return self.poses_dict.pop(key)
        else:
            self.logger.info('remove_pose: Indicated pose:{} does not exist'.format(key))
            
    def save_poses_dict (self, path:str) -> None:
        '''
        Save builded dictionary with positions to .json file.
        
        Save format:
        {
            P1:[x, y, z, Rx, Ry, Rz, 'p']
            or 
            P1:[q0, q1, q2, q3, q4, q5, 'j']
            
            x, y, z, Rx, Ry, Rz  - pose and orientation (to inverse kinematics)
            q0, q1, q2, q3, q4, q5 - joint angles (precision point)
        '''
        path = path.strip()
        tmp = path.split('/')
        folder = ''
        for i in tmp[:-1]:
            folder += i+'/'   
        if os.path.exists(folder):
            if re.search('.json',path).end() != len(path):
                path += '.json'
            with open(path, 'w') as poses_file:
                json.dump(self.poses_dict, poses_file, sort_keys=True, indent=4)
        else:
            self.logger.error('save_poses_dict: Indicated path:{} does not exist'.format(path))
    def load_poses_file(self, path:str) -> None:
        '''
        Loading previous saved file with poses
        '''
        if os.path.exists(path):
            with open(path, 'r') as poses_file:
                self.poses_dict = json.load(poses_file)
        else:
            self.logger.error('load_poses_file: Indicated path:{} does not exist'.format(path))

        
    # Usefull 'URScript' methods 
    
    def set_digital_out (self,n:int,b:bool) -> None:
        '''
        URScript syntax:
            set digital out(n, b)
        
        Description:
            Set digital output signal level
        
        Parameters:
            n: The number (id) of the output. (int)
            b: The signal level. (boolean)
        '''
        if n > 9 or n < 0:
            raise ValueError('fcn - set_digital_out: the number (id) invalid value')
        
        b = 'True' if b else 'False'
       
        tmp_msg = "set_digital_out({},{})\n".format(n,b)
        #self.logger.debug('Msg to send:{}'.format(tmp_msg))
        self.terminal.add_to_sendqueue(tmp_msg)
        
    def movej (self, q, a:float=3, v:float=0.75, t:float=0, r:float=0)->None:
        '''
        URScript syntax:
            movej(q, a=3, v=0.75, t=0, r=0)
        
        Description:
            Move to position (linear in joint-space) When using this command, the
            robot must be at standstill or come from a movej og movel with a blend.
            The speed and acceleration parameters controls the trapezoid speed
            profile of the move. The $t$ parameters can be used in stead to set the
            time for this move. Time setting has priority over speed and acceleration
            settings. The blend radius can be set with the $r$ parameters, to avoid
            the robot stopping at the point. However, if he blend region of this mover
            overlaps with previous or following regions, this move will be skipped, and
            an 'Overlapping Blends' warning message will be generated.

        Parameters:
            q: joint positions (q can also be specified as a pose, then
            inverse kinematics is used to calculate the corresponding
            joint positions)
            a: joint acceleration of leading axis [rad/s^2]
            v: joint speed of leading axis [rad/s]
            t: time [S]
            r: blend radius [m]
        '''
        mode = '' if q[-1] == 'j' else 'p'
        tmp_msg = "movej({}[{},{},{},{},{},{}], a={}, v={}, r={})\n".format(mode,*q[:6], a, v, r)
        #self.logger.debug('Msg to send:{}'.format(tmp_msg))
        self.terminal.add_to_sendqueue(tmp_msg)
        
    def movel (self, q, a:float=3, v:float=0.75, t:float=0, r:float=0) -> None:
        '''
        URScript syntax:
            movej(q, a=3, v =0.75, t=0, r =0
        
        Description:
            Move to position (linear in tool-space)
            See movej.
        
        Parameters:
            pose: target pose (pose can also be specified as joint
            positions, then forward kinematics is used to calculate
            the corresponding pose)
            a: tool acceleration [m/sˆ2]
            v: tool speed [m/s]
            t: time [S]
            r: blend radius [m]
        '''
        mode = '' if q[-1] == 'j' else 'p'
        tmp_msg = "movej({}[{},{},{},{},{},{}], a={}, v={}, r={})\n".format(mode,*q[:6], a, v, r)
        self.terminal.add_to_sendqueue(tmp_msg)
        
    def sleep (self,t:int) -> None:
        '''
        URScript syntax:
            sleep(t)
        
        Description:
            Sleep for an amount of time
            
        Parameters:
            t: time [s]
        '''
        tmp_msg = "sleep({})\n".format(t)
        self.terminal.add_to_sendqueue(tmp_msg)
    def textmsg (self, s:str) -> None:
        '''
        URScript syntax:
            textmsg(s)
        
        Description:
            Send text message
            Send message to be shown on the GUI log-tab

        Parameters:
            s: message string
        '''
        tmp_msg = "textmsg(\"{}\")\n".format(s)
        self.terminal.add_to_sendqueue(tmp_msg)
        
    def popup(self, s ,title:str ='Popup', warning:bool = False, error:bool = False) -> None:
        '''
        URScript syntax:
            popup(s, title='Popup', warning=False, error =False)
        
        Description:
            Display popup on GUI
            Display message in popup window on GUI.

        Parameters:
            s: message string
            title: title string
            warning: warning message?
            error: error message?
        '''
        tmp_msg = "popup(\"{}\",\"{}\",{},{})\n".format(s, title, warning, error)        
        self.terminal.add_to_sendqueue(tmp_msg)
        
    def get_actual_tcp_pose(self) -> list:
        '''
        URScript syntax:
            popup(s, title='Popup', warning=False, error =False)
        
        Description:
            Returns the current measured tool pose
            Returns the 6d pose representing the tool position and orientation
            specified in the base frame. The calculation of this pose is based on the
            actual robot encoder readings

        Parameters:
            ---
        Returned Value:
            The current actual TCP vector : ([X, Y, Z, Rx, Ry, Rz])
        
        '''
        return self.terminal.get_xyz_pose()
    
    def pose_trans(self, p_from:list, p_from_to:list) -> list:
        '''
        URScript syntax:
            pose trans(p_from, p_from to)
            
        Description:
            The first argument, p from, is used to transform the second argument,
            p from to, and the result is then returned. This means that the result is the
            resulting pose, when starting at the coordinate system of p from, and
            then in that coordinate system moving p from to.
            This function can be seen in two different views. Either the function
            transforms, that is translates and rotates, p from to by the parameters of
            p from. Or the function is used to get the resulting pose, when first making
            a move of p from and then from there, a move of p from to.
            If the poses were regarded as transformation matrices, it would look like:
            T world->to = T world->from * T from->to
            T x->to = T x->from * T from->to
            
        Parameters:
            p from: starting pose (spatial vector)
            p from to: pose change relative to starting pose (spatial
            vector)
            
        Return Value
            resulting pose (spatial vector)
        '''
        if p_from[-1] != 'j':
            new_pose = p_from.copy()
            if len(p_from) == 7 and len(p_from_to) == 6:
                for i, val in enumerate(p_from_to):
                    new_pose[i] += val/1000
                    
                return new_pose
            else:
                raise ValueError('axis_offset: incorect lenght of input vectors, exceptations: len(p_from) = 7, len(p_from) = 6, received: {},{}'.format(len(p_from),len(p_from_to)))
        else:
            raise ValueError('axis_offset:Cannot trans \'Joint\' pose')


    def pose_rot(self, p_from, rot_vec):
        if p_from[-1] != 'j':
            R_from, _ = cv.Rodrigues(np.array(p_from[-4:-1]))  
            alfa, fi, theta  = rot_vec
    
            rot_z = np.matrix([
                                    [math.cos(theta), -math.sin(theta), 0],
                                    [math.sin(theta), math.cos(theta), 0],
                                    [0, 0, 1]
                                    ])

            rot_y = np.matrix([
                                    [math.cos(fi), 0, math.sin(fi)],
                                    [0, 1, 0],
                                    [-math.sin(fi), 0, math.cos(fi)]
                                    ])

            rot_x = np.matrix([
                                    [1, 0, 0],
                                    [0, math.cos(alfa), -math.sin(alfa)],
                                    [0, math.sin(alfa), math.cos(alfa)]
                                    ])
            R_from = np.asmatrix (R_from)
            R_from_to = rot_z * rot_y * rot_x
            R_result = R_from*R_from_to
            new_rvec = cv.Rodrigues(R_result)
            new_rvec = np.resize(new_rvec[0],(1,3))
            new_pose = p_from.copy()
            new_pose[-4:-1] = new_rvec.tolist()[0]
            
            return new_pose
        else:
            raise ValueError('pose_rot: Cannot rotate \'Joint-pose\'')

    def axis_offset(self, pose:list, offset:int, axis:int = 2):
        '''
        Add offset value [mm] to indicated xyz coordinate
        '''
        if pose[-1] != 'j':
            new_pose = pose.copy()
            new_pose[axis] += (offset/1000)
            return new_pose
        else:
            raise ValueError('axis_offset:Cannot add \'XYZ-offset\' to position saved as:\'Joint pose\'')


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)
    
    sc = UR_robot('192.168.0.50',(1,8))
    time.sleep(2)
    sc.load_poses_file(r'path to file')
    
    p = sc.get_poses_dict()
    sc.movej(p['p_mobile_out'])
  
    # print(sc.pose_trans(p['l_b'],[0,0,50,0,0,0]))
    # print(sc.axis_offset(p['l_b'],50,2))
    # rx = np.deg2rad(30)
    # #new_pose = sc.pose_rot(p['l_b'],[0,0,rx])
    # #sc.movej(new_pose)
    
