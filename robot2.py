# rcj_soccer_player controller - ROBOT B2

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841
                i=0
                f=0
                p=0
                fl=1
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])

                # Write scripts of the robot here
                class PID() :
                
                     def __init__ (self) :
                        self.kp=20000
                        self.ki=0
                        self.kd=0
                        self.T=1
                        
                     def ctrl(self,e,im,fm) :

                         i1=im+self.ki*self.T*e  #Backward Dfference
                         d=(e-fm)/self.T         #Backward Dfference
                         f1=e
                         v1=-1*(self.kp*e+i1+self.kd*d)
                         
                         return [v1,i1,f1]
                        
                #
                def motorspeed(v):
                     vmotor=v
                     if v >= 10 :
                          vmotor=9.9
                   
                     elif v <= -10:
                          vmotor =-9.9
                          
                     if -0.15 <= v <= 0.15 :
                          vmotor=0
                     
                     return vmotor
                #
                def motor(v,a):
                     
                     if a==0:
                        self.left_motor.setVelocity(motorspeed(v))
                        self.right_motor.setVelocity(motorspeed(v))
                     elif a==1:
                        self.left_motor.setVelocity(-motorspeed(v))
                        self.right_motor.setVelocity(motorspeed(v))
                     elif a==-1:
                        self.left_motor.setVelocity(motorspeed(v))
                        self.right_motor.setVelocity(-motorspeed(v))
                #
                def true(e,d):
                     
                     if -d <= e <= d :
                         true=1
                     else:
                         true=0
                         
                     return true
                #
                def sign(a) :
                    
                   if a>=0 :
                      return 1
                   else :
                      return -1
                #
                
                strengthdesired=160
                xd=0
                yd=0.7
                
                rp=self.get_gps_coordinates()
                ph=self.get_compass_heading()
                print(rp)
                from math import atan2              
                pd=atan2(xd-rp[0],rp[1]-yd)
                
                bd=ball_data["direction"]
                phball=atan2(bd[1],bd[0])
                #print(ball_data["strength"])
                phda=atan2(xd-rp[0],rp[1]-yd)
                if rp[1]<=0 : fl=0
                
                if fl==0 :
                           
                     if (true(ph-phda,0.1)==0):
                           e=ph-phda
                           ct1=PID()
                           self.kp=2
                           self.ki=20
                           self.kd=0
                           self.T=1
                           v=ct1.ctrl(e,i,f)[0]
                           m=ct1.ctrl(e,i,f)[1]
                           f=ct1.ctrl(e,i,f)[2]
                           i=m
                           motor(v,1)
                     else :
                     
                           if true(rp[1]-yd,0.1)==0 : 
                             e=rp[1]-yd
                             ct1=PID()
                             self.kp=2
                             self.ki=20
                             self.kd=0
                             self.T=1
                             v=ct1.ctrl(e,i,f)[0]
                             m=ct1.ctrl(e,i,f)[1]
                             f=ct1.ctrl(e,i,f)[2]
                             i=m
                             motor(-v,0) 
                           else:
                             fl=1


       
                
                else:
                  
                  if ball_data["strength"] >= strengthdesired-1 :
                       if true(pd-ph,0.1)==0 :
                           e=pd-ph
                           ct1=PID()
                           self.kp=2
                           self.ki=20
                           self.kd=0
                           self.T=1
                           v=ct1.ctrl(e,i,f)[0]
                           m=ct1.ctrl(e,i,f)[1]
                           f=ct1.ctrl(e,i,f)[2]
                           i=m
                           motor(v,1)
                                         
                       if true(pd-ph,0.05)==1 :   
                           e=-0.7-rp[1]
                           ct2=PID()
                           self.kp=2
                           self.ki=20
                           self.kd=0
                           self.T=1
                           v=ct2.ctrl(e,i,f)[0]
                           m=ct2.ctrl(e,i,f)[1]
                           f=ct2.ctrl(e,i,f)[2]
                           i=m             
                           motor(v,0)

                  else:
                  
        
                       if true(phball,0.1)==0 :
                           e=phball
                           ct3=PID()
                           self.kp=20
                           self.ki=0
                           self.kd=0
                           self.T=1
                           v=ct3.ctrl(e,i,f)[0]
                           m=ct3.ctrl(e,i,f)[1]
                           f=ct3.ctrl(e,i,f)[2]
                           i=m
                           motor(v,1)
 
 
 
 
 
 
 
