import pygame
from shapely import affinity
from shapely.geometry import Polygon
import numpy as np
class Controller:
	def __init__():
		# Set up member variables
		self.x_pos = 0
		self.y_pos = 0
		self.yaw = 0

	def update_state():
		'''
		For every path point and yaw value, the controller outputs the vehicle's control variables. These can be any physical quantity
		and are not strictly tied to the simulator. 

		The only thing that the run_sim function expects to get is the position and orientation of the vehicle at a given point in time. 
		For now, the local_controller simply relays its path_point and yaw_value inputs to the run_sim function
		'''


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
