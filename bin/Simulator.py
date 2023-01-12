import pygame
from shapely import affinity
from shapely.geometry import Polygon
import numpy as np
import math
import cubic_splines as cs
import Controller

def control_block(px, py, pyaw):
	#xs and ys are both lists of x & y coords respectively

	#Time parameterizing the coordinates
	#px, py, pyaw, pk, ps = cs.calc_spline_course(xs, ys, ds=Controller.P.d_dist)
	
	sp = Controller.calc_speed_profile(px, py, pyaw, Controller.P.target_speed)
	
	param_path = Controller.PATH(px, py, pyaw)
	node = Controller.Node(x=px[0], y=py[0], yaw=pyaw[0], v=0.0)

	time = 0.0
	
	x = [node.x]
	y = [node.y]
	yaw = [node.yaw]
	v = [node.v]
	t = [0.0]
	d = [0.0]
	a = [0.0]
	
	delta_opt, a_opt = None, None
	a_exc, delta_exc = 0.0, 0.0
	
	while time < Controller.P.time_max:
		z_ref, target_ind = Controller.calc_ref_trajectory_in_T_step(node, param_path, sp)
		
		z0 = [node.x, node.y, node.v, node.yaw]
		
		a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = Controller.linear_mpc_control(z_ref, z0, a_opt, delta_opt)
		
		if delta_opt is not None:
			delta_exc, a_exc = delta_opt[0], a_opt[0]
			
		
		node.update(a_exc, delta_exc, 1.0)
		time += Controller.P.dt
		
		x.append(node.x)
		y.append(node.y)
		yaw.append(node.yaw)
		v.append(node.v)
		t.append(time)
		d.append(delta_exc)
		a.append(a_exc)
		
		dist = math.hypot(node.x - px[-1], node.y - py[-1])

		if dist < Controller.P.dist_stop and abs(node.v) < Controller.P.speed_stop:
			break

		dy = (node.yaw - yaw[-2]) / (node.v * Controller.P.dt)
		steer = Controller.pi_2_pi(-math.atan(Controller.P.WB * dy))


def run_sim(scenario, path_points, path_yaw):
	# initialise real-time plot with pygame
	pixel_scaling = 600/20 # 1 meter is 1000/20 pixels. Hence a 20*20 window would be 1000*1000 pixels
	pygame.init() # start pygame
	window = pygame.display.set_mode((scenario.width*pixel_scaling, scenario.height*pixel_scaling)) # keeping the window proportional to the scenario dims
	window.fill((255,255,255)) # white background
	xc, yc = window.get_rect().center # window center
	pygame.display.set_caption('Path Following Sim w/ path from RRT* and a local controller')

	font = pygame.font.Font('freesansbold.ttf', 12) # printing text font and font size
	text = font.render('Vehicle Sim - RRT*', True, (0, 0, 0), (255, 255, 255)) # printing text object
	textRect = text.get_rect()
	textRect.topleft = (10, 10) # printing text position with respect to the top-left corner of the window

	clock = pygame.time.Clock() # initialise clock
	FPS = int(100) # refresh rate


	# controller = Controller()
	vehicle_length, vehicle_width = scenario.vehicle_length, scenario.vehicle_width
	obstacles = scenario.obstacles

	# wait for key press
	run = True
	while run:
		for event in pygame.event.get(): # interrupt function
			if event.type == pygame.KEYUP:
				if event.key == ord('s'): # enter the main loop after 's' is pressed
					run = False

	# MAIN LOOP
	run = True
	for i in range(len(path_points)):
		for event in pygame.event.get(): # interrupt function
			if event.type == pygame.QUIT: # force quit with closing the window
				run = False
			elif event.type == pygame.KEYUP:
				if event.key == ord('q'): # force quit with q button
					run = False


		# x_pos, y_pos, yaw = controller.x_pos, controller.y_pos, controller.yaw
		x_pos, y_pos, yaw = path_points[i][0], path_points[i][1], path_yaw[i]
		# real-time plotting
		window.fill((255,255,255)) # clear window

		# Draw Obstacles
		for obstacle in obstacles:
			obstacle = affinity.scale(obstacle, xfact=pixel_scaling, yfact=pixel_scaling, origin='centroid')
			xs, ys = obstacle.exterior.coords.xy
			xs = np.add(xs, xc)
			ys = np.add(ys, yc)
			pygame.draw.polygon(window, (0,0,0), list(zip(xs,ys)))

		# Draw vehicle position
		car_footprint = Polygon([[(x_pos-vehicle_length/2),y_pos+vehicle_width/2],[x_pos+vehicle_length/2,y_pos+vehicle_width/2],[x_pos+vehicle_length/2,y_pos-vehicle_width/2],[x_pos-vehicle_length/2,y_pos-vehicle_width/2]])
		car_footprint = affinity.scale(car_footprint, xfact=pixel_scaling, yfact=pixel_scaling, origin='centroid')
		car_footprint = affinity.rotate(car_footprint, yaw, origin='centroid', use_radians=True) # Rotating the rectangle to capture orientation
		xs, ys = car_footprint.exterior.coords.xy
		xs = np.add(xs, xc)
		ys = np.add(ys, yc)
		pygame.draw.polygon(window, (255,0,0), list(zip(xs,ys)))

		# Draw Globally Planned Path

		# Draw Currently Executed Path
		
		text = font.render("FPS = " + str( round( clock.get_fps() ) ), True, (0, 0, 0), (255, 255, 255))
		window.blit(text, textRect)
		
		pygame.display.flip() # update display


		# Update State
		# controller.update_state()


		# increase loop counter
		i = i + 1
			
		# try to keep it real time with the desired step time
		clock.tick(FPS)
		
		if i >= len(path_points):
			run = False
		if run == False:
			break

	pygame.quit() # stop pygame
