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

current_frame_rgb=None
current_frame_depth=None
new_val=False


def normalize_dq(q):
	q[:-1]=q[:-1]/(np.linalg.norm(q[:-1])) 
	return q 

def ImageToMat(image):

	frame2=image.data.reshape([image.image_info.height, image.image_info.width, int(len(image.data)/(image.image_info.height*image.image_info.width))], order='C')

	return frame2


#This function is called when a new pipe packet arrives
def new_frame_rgb(pipe_ep):
	global current_frame_rgb, new_val
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):

		#Receive the packet
		image=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame_rgb=ImageToMat(image)
		new_val=True

		return
def new_frame_depth(pipe_ep):
	global current_frame_depth
	#Loop to get the newest frame
	while (pipe_ep.Available > 0):
		#Receive the packet
		
		depth_data=pipe_ep.ReceivePacket()
		#Convert the packet to an image and set the global variable
		current_frame_depth=depth_data.data.view(dtype=np.uint16).reshape([depth_data.image_info.height, depth_data.image_info.width], order='C')

		return

def move(n, robot_def,vel_ctrl,vd,R):
	try:
		w=10
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
	parser.add_argument("--robot-name",type=str,help="Robot name")
	args, _ = parser.parse_known_args()

	robot_name=args.robot_name
	robot_url=autodiscover("com.robotraconteur.robotics.robot.Robot",robot_name)
	#########read in yaml file for robot client

	url='rr+tcp://localhost:25415?service=Multi_Cam_Service'


	#Startup, connect, and pull out the camera from the objref    
	Multi_Cam_obj=RRN.ConnectService(url)


	#Connect the pipe FrameStream to get the PipeEndpoint p
	cam_rgb=Multi_Cam_obj.get_cameras(0)
	# cam_depth=Multi_Cam_obj.get_cameras(1)


	p_rgb=cam_rgb.frame_stream.Connect(-1)
	# p_depth=cam_depth.frame_stream.Connect(-1)
	#Set the callback for when a new pipe packet is received to the
	#new_frame function
	p_rgb.PacketReceivedEvent+=new_frame_rgb
	# p_depth.PacketReceivedEvent+=new_frame_depth


	try:
		cam_rgb.start_streaming()
		# cam_depth.start_streaming()
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
	robot.jog_freespace(np.zeros(num_joints),np.ones(num_joints), True)


	robot.command_mode = halt_mode 
	robot.command_mode = position_mode 
	#enable velocity mode
	vel_ctrl.enable_velocity_mode()


	while True:
		#show image
		if (not current_frame_rgb is None):
			cv2.imshow("Image",current_frame_rgb)
			centroid=laser_detection(current_frame_rgb)
			try:
				centroid[0]

			except:
				vel_ctrl.set_velocity_command(np.zeros(num_joints))
				# move(num_joints, robot_def,vel_ctrl,np.zeros(3),R)
				continue
			vd=np.zeros(3)
			vd[1]=-0.0001*(centroid[0]-640)
			vd[2]=0.0001*(centroid[1]-360)
			move(num_joints, robot_def,vel_ctrl,vd,R)
		if cv2.waitKey(50)!=-1:
			break

			
		

	p_rgb.Close()
	p_depth.Close()
	cam_rgb.stop_streaming()
	cam_depth.stop_streaming()



if __name__ == '__main__':
	main()