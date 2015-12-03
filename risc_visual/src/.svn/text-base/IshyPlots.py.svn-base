import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

#plt.rc(r'text', usetex=True)
#plt.rc(r'font', family='serif')
mpl.rcParams['legend.fontsize'] = 12

def pl3d(Traj_name,x_act,y_act,z_act):
        x_act=list(np.array(x_act).reshape(-1))
	y_act=list(np.array(y_act).reshape(-1))
	z_act=list(np.array(z_act).reshape(-1))
	fig = plt.figure(Traj_name)
	ax = fig.gca(projection='3d')
	ax.plot(x_act, y_act, z_act,'k-', label='Cortex')
	ax.legend()
	ax.set_title(Traj_name + ' Trajectory', fontsize=16)
	ax.set_xlabel(r'X (m)', fontsize=14)
	ax.set_ylabel(r'Y (m)', fontsize=14)
	ax.set_zlabel(r'Z (m)', fontsize=14)
	ax.set_xlim([-2, 2])
	ax.set_ylim([-2, 2])
	ax.set_zlim([0, 2])

def pl_phi(Traj_name,time,phi_traj,phi_sens,phi_cort):
	
	
	fig = plt.figure(Traj_name + '_Phi')
	plt.plot(time, phi_traj,'k-', label='Actual')
	plt.plot(time, phi_sens,'k--', label='Sensor')
	plt.plot(time, phi_cort,'k:', label='Cortex')
	plt.legend()
	plt.title(Traj_name + ' Roll Angle', fontsize=16)
	plt.xlabel(r'Time (s)', fontsize=14)
	plt.ylabel(r'$\phi$ (rad)}', fontsize=14)

def pl_theta(Traj_name,time,theta_traj,theta_sens,theta_cort):
	

	fig = plt.figure(Traj_name + '_Theta')
	plt.plot(time, theta_traj,'k-', label='Actual')
	plt.plot(time, theta_sens,'k--', label='Sensor')
	plt.plot(time, theta_cort,'k:', label='Cortex')
	plt.legend()
	plt.title(Traj_name + ' Pitch Angle', fontsize=16)
	plt.xlabel(r'Time (s)', fontsize=14)
	plt.ylabel(r'$\theta$ (rad)', fontsize=14)


def pl_psi(Traj_name,time,psi_traj,psi_sens,psi_cort):
	

	fig = plt.figure(Traj_name + '_Psi')
	plt.plot(time, psi_traj,'k-', label='Actual')
	plt.plot(time, psi_sens,'k--', label='Sensor')
	plt.plot(time, psi_cort,'k:', label='Cortex')
	plt.legend()
	plt.title(Traj_name + ' Yaw Angle', fontsize=16)
	plt.xlabel(r'Time (s)', fontsize=14)
	plt.ylabel(r'$\psi$ (rad)', fontsize=14)

def pl_perror(Traj_name,time,x_err,y_err,z_err):
	
	
	fig = plt.figure(Traj_name + '_Position_Error')
	plt.plot(time, x_err,'k-', label='X Error')
	plt.plot(time, y_err,'k--', label='Y Error')
	plt.plot(time, z_err,'k:', label='Z Error')
	plt.legend()
	plt.title(Traj_name + ' Position Errors', fontsize=16)
	plt.xlabel(r'Time (s)', fontsize=14)
	plt.ylabel(r'Error (m)', fontsize=14)
	
def pl_verror(Traj_name,time,u_err,v_err,w_err,u_err_sens,v_err_sens,w_err_sens):
	

	fig = plt.figure(Traj_name + '_Velocity_Error')
	plt.plot(time, u_err,'k-',lw=3, label='X Cortex')
	plt.plot(time, v_err,'k--',lw=3, label='Y Cortex')
	plt.plot(time, w_err,'k:',lw=3, label='Z Cortex')
	plt.plot(time, u_err_sens,'k-', label='X Sensor')
	plt.plot(time, v_err_sens,'k--', label='Y Sensor')
	plt.plot(time, w_err_sens,'k:', label='Z Sensor')
	plt.legend()
	plt.title(Traj_name + ' Velocity Errors', fontsize=16)
	plt.xlabel(r'Time (s)', fontsize=14)
	plt.ylabel(r'Error (m/s)', fontsize=14)
	

