import numpy as np
from qpsolvers import solve_qp
from RobotRaconteur.Client import *
import sys, time, yaml, argparse, traceback
from importlib import import_module
from laser_detection import laser_detection
import cv2
sys.path.append('../toolbox/')
from general_robotics_toolbox import *
from vel_emulate_sub import EmulatedVelocityControl
from autodiscovery import autodiscover


new_val=False
def pointInRect(point,rect):
    x1, y1, x2, y2 = rect
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            return True
    return False

def normalize_dq(q):
	q[:-1]=q[:-1]/(np.linalg.norm(q[:-1])) 
	return q 

def ImageToMat(image):

	frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

	return frame2
current_frame=None



def new_frame(pipe_ep):
	global current_frame, now
	# print('fps= ', 1/(time.time()-now))
	now=time.time()
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame=ImageToMat(image)


def move(n, robot_def,vel_ctrl,vd,R):
	try:
		w=10000000000000
		Kq=.01*np.eye(n)    #small value to make sure positive definite
		KR=np.eye(3)        #gains for position and orientation error

		q_cur=vel_ctrl.joint_position()
		J=robotjacobian(robot_def,q_cur)        #calculate current Jacobian
		Jp=J[3:,:]
		JR=J[:3,:] 
		H=np.dot(np.transpose(Jp),Jp)+Kq+w*np.dot(np.transpose(JR),JR)

		H=(H+np.transpose(H))/2

		robot_pose=fwdkin(robot_def,q_cur.reshape((n,1)))
		R_cur = robot_pose.R
		ER=np.dot(R_cur,np.transpose(R))
		k,theta = R2rot(ER)
		k=np.array(k,dtype=float)
		s=np.sin(theta/2)*k         #eR2
		wd=-np.dot(KR,s)  
		f=-np.dot(np.transpose(Jp),vd)-w*np.dot(np.transpose(JR),wd)
		qdot=0.2*normalize_dq(solve_qp(H, f))
		vel_ctrl.set_velocity_command(qdot)

	except:
		traceback.print_exc()
	return




def connect_failed(s, client_id, url, err):
	print ("Client connect failed: " + str(client_id.NodeID) + " url: " + str(url) + " error: " + str(err))

def main():
	global new_val, p_realsense, R_realsense
	#Accept the names of the webcams and the nodename from command line
	parser = argparse.ArgumentParser(description="RR plug and play client")
	parser.add_argument("--robot-name",type=str,default='tormach',help="Robot name")
	args, _ = parser.parse_known_args()

	robot_name=args.robot_name
	robot_url=autodiscover("com.robotraconteur.robotics.robot.Robot",robot_name)
	#########read in yaml file for robot client

	url='rr+tcp://localhost:59823?service=camera'
	if (len(sys.argv)>=2):
		url=sys.argv[1]

	#Startup, connect, and pull out the camera from the objref    
	c=RRN.ConnectService(url)

	#Connect the pipe FrameStream to get the PipeEndpoint p
	p=c.frame_stream.Connect(-1)

	#Set the callback for when a new pipe packet is received to the
	#new_frame function
	p.PacketReceivedEvent+=new_frame
	try:
		c.start_streaming()
	except: 
		traceback.print_exc()
		pass


	###robot part
	robot_sub=RRN.SubscribeService(robot_url)
	####get client object
	robot=robot_sub.GetDefaultClientWait(1)
	
	####get subscription wire

	##robot wire
	cmd_w = robot_sub.SubscribeWire("position_command")
	state_w = robot_sub.SubscribeWire("robot_state")
	####connection fail callback
	robot_sub.ClientConnectFailed += connect_failed

	##########Initialize robot constants
	robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
	halt_mode = robot_const["RobotCommandMode"]["halt"]
	jog_mode = robot_const["RobotCommandMode"]["jog"]   
	position_mode = robot_const["RobotCommandMode"]["position_command"]
	robot.command_mode = halt_mode
	time.sleep(0.1)
	robot.command_mode = jog_mode 
	##########Initialize robot parameters   #need modify
	num_joints=len(robot.robot_info.joint_info)
	P=np.array(robot.robot_info.chains[0].P.tolist())
	length=np.linalg.norm(P[1])+np.linalg.norm(P[2])+np.linalg.norm(P[3])
	H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
	robot_def=Robot(H,np.transpose(P),np.zeros(num_joints))
	#always facing 0 config orientation
	R=np.eye(3)

	#######move to start point
	print("ready, jogging to start pose")

	
	vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w, 0.01)
	robot.jog_freespace([-5.21293594e-05,  2.71570638e-01,  5.46115274e-01, -9.20868630e-06,  6.40457320e-01, -1.22748139e-04],0.2*np.ones(num_joints), True)


	robot.command_mode = halt_mode 
	robot.command_mode = position_mode 
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()


	while True:
		now=time.time()
		#show image
		if (not current_frame is None):
			cv2.imshow("Image",current_frame)
			if cv2.waitKey(50)!=-1:
				break
			try:
				centroid=laser_detection(current_frame)			
			except:
				vel_ctrl.set_velocity_command(np.zeros(num_joints))
				# move(num_joints, robot_def,vel_ctrl,np.zeros(3),R)
				# traceback.print_exc()
				continue
			####map dot vector to base frame
			pose=state_w.InValue.kin_chain_tcp[0]
			position=list(pose['position'])
			q=list(pose['orientation'])
			R=q2R(q)

			vd=np.zeros(3)
			vd[1]=-0.00001*(centroid[0]-len(current_frame[0])/2.)
			vd[2]=-0.00001*(centroid[1]-len(current_frame)/2.)

			vd_base=np.dot(R,vd)
			vd_base[-1]=0.

			next_p=np.array(position)+vd_base
			if (not pointInRect(tuple(next_p[:2]),(0.25,-0.35,0.75,0.35))) or np.linalg.norm(vd)<0.00001*20:
				vel_ctrl.set_velocity_command(np.zeros(num_joints))
				continue


			move(num_joints, robot_def,vel_ctrl,vd_base,R)
		print(time.time()-now)
		

	cv2.destroyAllWindows()
    
		

	p.Close()
	c.stop_streaming()



if __name__ == '__main__':
	main()