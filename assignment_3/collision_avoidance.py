import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import time
from tf.transformations import euler_from_quaternion

ANG_MAX = math.pi/18
VEL_MAX = 0.15

class collision_avoid

    def __init__(self):
        self.start_time=0
        self.VEL_MAX = VEL_MAX
        self.bot_diameter = 0.15

        self.bot_pose = [0.0, 0.0, 0.0] #x,y,theta of the bot
        self.bot_v = [0.0, 0.0] #current velocity of the bot
        self.bot_omega = 0.0: #current angular velocity of the bot

        self.obs_pose_x = []
        self.obs_pose_y = []
        self.obs_v_x = []
        self.obs_v_y = []

        self.goal = [5, 0]

        self.path_x = []
        self.path_y = []
        self.time = []

        self.start_time = time.time()
        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
        self.r = rospy.Rate(30)
        self.obs_data = ObsData()
        self.odom = Odometry()
        
        rospy.Subscriber('/obs_data', ObsData, callback_obs) #topic name fixed
        rospy.Subscriber('/bot_1/odom', Odometry, callback_odom) #topic name fixed

        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
        self.r = rospy.Rate(30)
        
		self.f=open('output.txt','w')
		self.f.write('{:.5f} , {:.5f} \n'.format(self.bot_pose[0],self.bot_pose[1]))

		#plot
		self.bot_path=[[],[]]
		self.time_elapsed=[0]
	


   def velocity_convert(x, y, theta, vel_x, vel_y):
       '''
       Robot pose (x, y, theta)  Note - theta in (0, 2pi)
       Velocity vector (vel_x, vel_y)
       '''

       gain_ang = 1 #modify if necessary
    
       ang = math.atan2(vel_y, vel_x)
       if ang < 0:
           ang += 2 * math.pi
    
       ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

       v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
       v_ang = gain_ang * ang_err
       return v_lin, v_ang


   def callback_obs(data):
       '''
       Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
       '''
       n=len(data.obstacles)			
		self.obs_pose_x = np.zeros(self.n)
        self.obs_pose_y = np.zeros(self.n)
        self.obs_v_x = np.zeros(self.n)
        self.obs_v_y = np.zeros(self.n)
		 
		for i in range(data.obstacles):
			self.obs_pose_x[i] = data.obstacles[i].pose_x
            self.obs_pose_y[i] = data.obstacles[i].pose_y
            self.obs_v_x[i] = data.obstacles[i].vel_x
            self.obs_v_y[i] = data.obstacles[i].vel_y
       


   def callback_odom(data):
       '''
       Get robot data
       '''
        self.bot_pose[0] = data.pose.pose.position.x
        self.bot_pose[1] = data.pose.pose.position.y
        self.bot_pose[2] = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.bot_v[0] = data.twist.twist.linear.x
        self.bot_v[1] = data.twist.twist.linear.y
        self.bot_omega = data.twist.twist.angular.z
      
       
        self.f.write('{:.5f} , {:.5f} \n'.format(self.pose[0],self.pose[1]))
	    self.time_elapsed.append(time()-self.start_time)
		self.bot_path[0].append(self.bot_pose[0])
		self.bot_path[1].append(self.bot_pose[1])
       
def distance(self,a,b):
		# print(sqrt((a[0]-b[0])**2+(a[1]-b[1])**2))
		return sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

def angleBetweenVectors(self, v, w):
        v = np.array(v)
        w = np.array(w)
        angle = 0
        if np.linalg.norm(v)*np.linalg.norm(w) != 0.0:
            unit_vector_1 = v / np.linalg.norm(v)
            unit_vector_2 = w / np.linalg.norm(w)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            if abs(dot_product) > 1:
                dot_product = np.sign(dot_product)*1
            angle = np.arccos(dot_product)
        return angle
        
def velocityObstacleCone(self, idxObs):
        ratio = self.bot_diameter/self.distance(self.bot_pose,[i.pose_x,i.pose_y])
        if abs(ratio) > 1:
            ratio = np.sign(ratio)*1
        sweptAngle = np.arcsin(ratio)
        return sweptAngle

if __name__ == '__main__':
   
    rospy.init_node('collision_avoidance', anonymous = True)
    

   

    while True: #replace with destination reached?
    #calculate collision cone below

    #calculate v_x, v_y as per either TG, MV, or ST strategy
    #Make sure your velocity vector is feasible (magnitude and direction)

    #convert velocity vector to linear and angular velocties using velocity_convert function given above

    #publish the velocities below
    vel_msg = Twist()
    # vel_msg.linear.x = v_lin
    # vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)
    
    #store robot path with time stamps (data available in odom topic)
    plt.plot(Solver.waypoints[0],Solver.waypoints[1])
	plt.title('Plot of robot path')
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.show()
	plt.plot(Solver.time_elapsed,Solver.waypoints[0])
	plt.title('Plot of robot’s x-coordinate vs time')
	plt.ylabel('pos_x')
	plt.xlabel('time in seconds	')
	plt.show()
