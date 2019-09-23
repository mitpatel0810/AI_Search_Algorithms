#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Chirav Dave"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import numpy as np
import os, sys
class Maze:

	def copy_empty_world(self):
		parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
		#File containing empty world description
		f_in = open(parent_dir+'/worlds/empty_world.sdf', 'r')
		#File to save your maze
		f_out = open(parent_dir+'/worlds/maze.sdf', 'w')
		#Copying empty world description into output maze
		for line in f_in:
			f_out.write(line)
		f_in.close()
		return f_out

	#Method to add wall description surrounding the maze
	def add_walls_description(self, f_out):
		for i in range(1, 5):
			f_out.write('<model name=\'wall{}\'>\n'.format(i))
			f_out.write('<static>1</static>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n')
			f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
			f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n</geometry>\n<material>\n<script>\n')
			f_out.write('<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
			f_out.write('<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')

	#Method to place walls around the maze
	def add_walls(self, f_out, length):
		scale = (length+2)/7.5
		wall_dimensions = [(-1, length/2, -1.55905, scale, 1), (length/2, length+1, 0, scale, 1), (length+1, length/2, -1.55905, scale, 1), (length/2, -1, 0, scale, 1)]
		for i in range(4):
			f_out.write('<model name=\'wall{}\'>\n'.format(i+1))
			f_out.write('<pose frame=\'\'>{} {} 0 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			f_out.write('<scale>{} {} 0.03</scale>\n'.format(wall_dimensions[i][3], wall_dimensions[i][4]))
			f_out.write('<link name=\'link\'>\n')
			f_out.write('<pose frame=\'\'>{} {} 0.42 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')

	#Method to add description for cans
	def add_can_description(self, f_out, coords):
		for i in coords:
			x, y = i
			f_out.write('<model name=\'can{}{}\'>\n'.format(x, y))
			f_out.write('<link name=\'link\'>\n<pose frame=\'\'>0 0 0.115 0 -0 0</pose>\n<inertial>\n<mass>0.39</mass>\n<inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n</inertial>\n<collision name=\'collision\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n')
			f_out.write('<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n<visual name=\'visual\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n<material>\n<script>\n<uri>model://beer/materials/scripts</uri>\n<uri>model://beer/materials/textures</uri>\n<name>Beer/Diffuse</name>\n</script>\n</material>\n</visual>\n')
			f_out.write('<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>0.888525 -2.58346 0 0 -0 0</pose>\n</model>\n')
			f_out.write('<gui fullscreen=\'0\'>\n<camera name=\'user_camera\'>\n<pose frame=\'\'>5 -5 2 0 0.275643 2.35619</pose>\n<view_controller>orbit</view_controller>\n<projection_type>perspective</projection_type>\n</camera>\n</gui>\n')

	#Method to place cans inside the maze
	def add_can(self, f_out, x, y):
		f_out.write('<model name=\'can{}{}\'>\n'.format(x, y))
		f_out.write('<pose frame=\'\'>{} {} -2e-06 1e-06 0 -9.5e-05</pose>\n'.format(x, y))
		f_out.write('<scale>0.5 0.5 1</scale>\n<link name=\'link\'>\n')
		f_out.write('<pose frame=\'\'>{} {} 0.114998 1e-06 0 -9.5e-05</pose>\n'.format(x, y))
		f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -3.822 0 -0 0</wrench>\n</link>\n</model>\n')

	#Method to add description for the goal can
	def add_goal_description(self, f_out, coord):
		f_out.write('<model name=\'goal\'>\n<link name=\'link\'><pose frame=\'\'>0 0 0.115 0 -0 0</pose>\n<inertial>\n<mass>0.0005</mass><inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n')
		f_out.write('<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n</inertial>\n<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n')
		f_out.write('<visual name=\'visual\'>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n</cylinder>\n</geometry>\n<material>\n<script>\n<uri>model://beer/materials/scripts</uri>\n<uri>model://beer/materials/textures</uri>\n')
		f_out.write('<name>Beer/Diffuse</name>\n</script>\n<ambient>1 0 0 1</ambient>\n<diffuse>1 0 0 1</diffuse>\n<specular>0 0 0 1</specular>\n<emissive>0 0 0 1</emissive>\n<shader type=\'vertex\'>\n')
		f_out.write('<normal_map>__default__</normal_map>\n</shader>\n</material>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<cast_shadows>1</cast_shadows>\n<transparency>0</transparency>\n</visual>\n')
		f_out.write('<collision name=\'collision\'>\n<laser_retro>0</laser_retro>\n<max_contacts>10</max_contacts>\n<pose frame=\'\'>0 0 0 0 -0 0</pose>\n<geometry>\n<cylinder>\n<radius>0.055</radius>\n<length>0.23</length>\n')
		f_out.write('</cylinder>\n</geometry>\n<surface>\n<friction>\n<ode>\n<mu>1</mu>\n<mu2>1</mu2>\n<fdir1>0 0 0</fdir1>\n<slip1>0</slip1>\n<slip2>0</slip2>\n</ode>\n<torsional>\n<coefficient>1</coefficient>\n')
		f_out.write('<patch_radius>0</patch_radius>\n<surface_radius>0</surface_radius>\n<use_patch_radius>1</use_patch_radius>\n<ode>\n<slip>0</slip>\n</ode>\n</torsional>\n</friction>\n<bounce>\n')
		f_out.write('<restitution_coefficient>0</restitution_coefficient>\n<threshold>1e+06</threshold>\n</bounce>\n<contact>\n<collide_without_contact>0</collide_without_contact>\n<collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n')
		f_out.write('<collide_bitmask>1</collide_bitmask>\n<ode>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n<kd>1</kd>\n<max_vel>0.01</max_vel>\n<min_depth>0</min_depth>\n</ode>\n')
		f_out.write('<bullet>\n<split_impulse>1</split_impulse>\n<split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n')
		f_out.write('<kd>1</kd>\n</bullet>\n</contact>\n</surface>\n</collision>\n</link>\n<static>0</static>\n<allow_auto_disable>1</allow_auto_disable>\n')
		f_out.write('<pose frame=\'\'>{} {} 0 0 -0 0</pose>\n</model>\n'.format(coord, coord))

	#Method to place the goal can
	def add_goal(self, f_out, coord):
		f_out.write('<model name=\'goal\'>\n')
		f_out.write('<pose frame=\'\'>{} {} -9e-06 -1e-06 -4e-06 0</pose>\n'.format(coord, coord))
		f_out.write('<scale>1 1 1</scale>\n<link name=\'goal::link\'>\n')
		f_out.write('<pose frame=\'\'>{} {} 0.114991 -1e-06 -4e-06 0</pose>\n'.format(coord, coord))
		f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -9.8 0 -0 0</wrench>\n</link>\n</model>\n')

	#Method to generate maze
	def generate_maze(self, grid_dimension, n_obstacles, seed, scale=0.5):
		np.random.seed(seed)
		blocked_edges = set()
		coords = []
		f_out = self.copy_empty_world()
		self.add_walls(f_out, grid_dimension*scale)
		count = 1
		while(count <= n_obstacles):
			x = scale*np.random.randint(0, grid_dimension+1)
			y = scale*np.random.randint(0, grid_dimension+1)
			if x == 0 and y == 0:  # do not block edges starting at origin
				continue
			#flag is used to decide if we want to block the edge (x, y) and (x+1, y) or (x, y) and (x, y+1) 
			flag = np.random.randint(0, 2)
			if(flag == 0 and ((x+scale) <= grid_dimension*scale) and ((x, y, x+scale, y) not in blocked_edges)):
				blocked_edges.add((x, y, x+scale, y))
				#Adding obstacle in the middle with some offset value of the edge to be blocked
				offset = np.random.uniform(0, 0.07*scale)
				coords.append((x+scale/2+offset, y))
				self.add_can(f_out, x+scale/2+offset, y)
				count += 1
			elif(flag == 1 and ((y+scale) <= grid_dimension*scale) and ((x, y, x, y+scale) not in blocked_edges)):
				blocked_edges.add((x, y, x, y+scale))
				#Adding obstacle in the middle with some offset value of the edge to be blocked
				offset = np.random.uniform(0, 0.07*scale)
				coords.append((x, y+scale/2-offset))
				self.add_can(f_out, x, y+scale/2-offset)
				count += 1
		self.add_goal(f_out, grid_dimension*0.5)
		f_out.write('</state>')
		self.add_goal_description(f_out, grid_dimension*0.5)
		self.add_walls_description(f_out)
		self.add_can_description(f_out, coords)
		f_out.write('</world>\n</sdf>')
		f_out.close()
		maze = [(0, grid_dimension, 'EAST', scale), blocked_edges]
		return maze
