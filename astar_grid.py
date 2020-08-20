import pygame
import math
from queue import PriorityQueue
import datetime
import csv

pygame.init()

#The width of the canvas
WIDTH = 800
#Window object
WIN = pygame.display.set_mode((WIDTH, WIDTH))
#Title
pygame.display.set_caption("A* Path Finding Algorithm")

#Colors and font
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
BROWN = (165, 42, 42)
NEON = (80, 100, 0)
FONT = pygame.font.SysFont('comicsans', 30, True)

class Node:
	def __init__(self, col, row, width, total_cols):
		self.col = col
		self.row = row
		self.x = (col + 0.5) * width
		self.y = (row + 0.5) * width
		self.color = WHITE
		self.neighbors = []

		#width of a square grid
		self.width = width
		self.total_cols = total_cols

		#grouping node into groups to repsent the obstacles they are in
		self.group = []
		self.start = False
		self.end = False

		#edge for visibility graph
		self.edge = False
		self.g_score = float("inf")
		self.f_score = float("inf")

	def get_pos(self):
		return self.col, self.row
	
	def get_coordinate(self):
		return self.x, self.y
	
	def get_group(self):
		return self.group

	def get_g(self):
		return self.g_score

	def get_f(self):
		return self.f_score
	
	def set_x(self, x):
		self.x = x

	def set_y(self, y):
		self.y = y

	def set_group(self, g):
		self.group = g

	def is_closed(self):
		return self.color == RED

	def is_open(self):
		return self.color == GREEN

	def is_barrier(self):
		return self.color == BLACK

	def is_start(self):
		return self.start

	def is_end(self):
		return self.end

	def is_edge(self):
		return self.edge

	def reset(self):
		self.color = WHITE
		self.start = self.end = self.edge = False

	def make_edge(self):
		self.color = BROWN
		self.edge = True

	def make_temp(self):
		self.color = NEON

	def make_start(self):
		self.color = ORANGE
		self.start = True

	def make_closed(self):
		self.color = RED

	def make_open(self):
		self.color = GREEN

	def make_barrier(self):
		self.color = BLACK

	def make_end(self):
		self.color = TURQUOISE
		self.end = True

	def make_path(self):
		self.color = PURPLE

	#visibile from start
	def vis_from_start(self):
		self.color = YELLOW

	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.col * self.width, self.row * self.width, self.width, self.width))

	def find_visible_neighbor(self, grid, edge_node):
		self.neighbors = []
		for node in edge_node:
			if self.check_v(grid, node):
				self.neighbors.append(node)

	#check if node is visible from self
	def check_v(self, grid, node):
		if node.neighbors.count(self) != 0:
			return True

		col, row = self.get_pos()
		col1, row1 = node.get_pos()

		if col == col1 and row == row1:
			return False

		deltay = row1 - row
		deltax = col1 - col

		if deltax != 0: #if not vertical
			slope = deltay * 1.0 / deltax

			if deltax > 0:
				step = - 0.5
				n = 1
			else:
				step = 0.5
				n = -1
			temp = col - n


			while temp != col1 + n:
				y = slope * step + row
				deci_y = y % 1
	
				if (self.is_start() or self.is_end()) and temp == col - n:
					pass
				elif temp == col1 and (len(node.get_group()) > 1 or node.is_end or node.is_start):
					break
				else:
					if deci_y == 0.5:
						if deltay > 0:
							if (grid[temp][int(math.ceil(y))].is_barrier() and grid[temp + n][int(math.floor(y))].is_barrier()) or grid[temp + n][int(math.ceil(y))].is_barrier():
								return False
						else:
							if (grid[temp][int(math.floor(y))].is_barrier() and grid[temp + n][int(math.ceil(y))].is_barrier()) or grid[temp + n][int(math.floor(y))].is_barrier():
								return False
					else:
						if grid[temp][int(round(y))].is_barrier() or grid[temp + n][int(round(y))].is_barrier():
							return False
				temp = temp + n
				step = step + n


		if deltay != 0: #if not horizontal
			in_slope = deltax * 1.0 / deltay

			if deltay > 0:
				step = - 0.5
				n = 1
			else:
				step = 0.5
				n = -1
			temp = row - n
			while temp != row1 + n:
				x = in_slope * step + col
				
				if (self.is_start() or self.is_end()) and temp == row - n:
					pass
				elif temp == row1 and (len(node.get_group()) > 1 or node.is_end or node.is_start):
					break
				else:
					if x % 1 != 0.5 and (grid[int(round(x))][temp].is_barrier() or grid[int(round(x))][temp + n].is_barrier()):
						return False

				temp = temp + n
				step = step + n

		return True

	def update_neighbors(self, grid):
		self.neighbors = []

		not_bottom = self.col < self.total_cols - 1
		not_top = self.col > 0
		not_right = self.row < self.total_cols - 1
		not_left = self.row > 0

		if  not_bottom and not grid[self.col + 1][self.row].is_barrier(): # DOWN
			self.neighbors.append(grid[self.col + 1][self.row])

		if  not_top and not grid[self.col - 1][self.row].is_barrier(): # UP
			self.neighbors.append(grid[self.col - 1][self.row])

		if  not_right and not grid[self.col][self.row + 1].is_barrier(): # RIGHT
			self.neighbors.append(grid[self.col][self.row + 1])

		if  not_left and not grid[self.col][self.row - 1].is_barrier(): # LEFT
			self.neighbors.append(grid[self.col][self.row - 1])

		if not_top and not_left and not grid[self.col - 1][self.row - 1].is_barrier() and not (grid[self.col - 1][self.row].is_barrier() and grid[self.col][self.row - 1].is_barrier()): #UP-LEFT
			self.neighbors.append(grid[self.col - 1][self.row - 1])

		if not_top and not_right and not grid[self.col - 1][self.row + 1].is_barrier()and not (grid[self.col - 1][self.row].is_barrier() and grid[self.col][self.row + 1].is_barrier()): #UP-RIGHT
			self.neighbors.append(grid[self.col - 1][self.row + 1])

		if not_bottom and not_left and not grid[self.col + 1][self.row - 1].is_barrier() and not (grid[self.col + 1][self.row].is_barrier() and grid[self.col][self.row - 1].is_barrier()): #DOWN-LEFT
			self.neighbors.append(grid[self.col + 1][self.row - 1])

		if not_bottom and not_right and not grid[self.col + 1][self.row + 1].is_barrier() and not (grid[self.col + 1][self.row].is_barrier() and grid[self.col][self.row + 1].is_barrier()): #DOWN-RIGHT
			self.neighbors.append(grid[self.col + 1][self.row + 1])

	def __lt__(self, other):
		return False

def distance(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	result = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
	return result

def h(p1, p2):
	return distance(p1, p2)

#Finish drawing path
def reconstruct_path(came_from, current):
	length = 0.0
	coord_list = []
	while current in came_from:
		length = length + distance(current.get_pos(), came_from[current].get_pos())
		coord = [came_from[current].get_coordinate(), current.get_coordinate()]
		coord_list.append(coord)
		current = came_from[current]
		current.make_path()
	return length, coord_list

#Find edge node for visibility graph
def find_edge(grid, barrier, start, end):
	edge_node = []
	edge_node.append(start)
	edge_node.append(end)
	for node in barrier:
		candidates = []
		col, row = node.get_pos()
		top = down = left = right = True
		top_left = top_right = down_left = down_right = True

		not_bottom = node.col < node.total_cols - 1
		not_top = node.col > 0
		not_right = node.row < node.total_cols - 1
		not_left = node.row > 0

		if not_top:
			top = grid[col - 1][row].is_barrier()
			if not_left:
				top_left = grid[col - 1][row - 1].is_barrier()
			if not_right:
				top_right = grid[col - 1][row + 1].is_barrier()
		if not_bottom:
			down = grid[col + 1][row].is_barrier()
			if not_left:
				down_left = grid[col + 1][row - 1].is_barrier()
			if not_right:
				down_right = grid[col + 1][row + 1].is_barrier()
		if not_left:
			left = grid[col][row - 1].is_barrier()
		if not_right:
			right = grid[col][row + 1].is_barrier()


		if (top_left and down_right) or (top_right and down_left) or (top and down) or (left and right):
			continue
		else:
			if not top_left and not top_right and not top:
				candidates.append(grid[col - 1][row])
				grid[col - 1][row].set_group(grid[col - 1][row].get_group() + node.get_group())
			if not down_left and not down_right and not down:
				candidates.append(grid[col + 1][row])
				grid[col + 1][row].set_group(grid[col + 1][row].get_group() + node.get_group())
			if not top_left and not down_left and not left:
				candidates.append(grid[col][row - 1])
				grid[col][row - 1].set_group(grid[col][row - 1].get_group() + node.get_group())
			if not top_right and not down_right and not right:
				candidates.append(grid[col][row + 1])
				grid[col][row + 1].set_group(grid[col][row + 1].get_group() + node.get_group())

		for node in candidates:
			if edge_node.count(node) == 0:
				edge_node.append(node)

	return edge_node

#Run A* algorithm with visibility graph
def visibility(grid, start, end, print_edge, print_length):
	barrier = []
	group = 0
	for row in grid:
		for node in row:
			if node.is_barrier():
				barrier.append(node)
				col, row = node.get_pos()
				if len(grid[col + 1][row].get_group()) != 0:
					node.set_group(grid[col + 1][row].get_group())
				elif len(grid[col - 1][row].get_group()) != 0:
					node.set_group(grid[col - 1][row].get_group())
				elif len(grid[col + 1][row].get_group()) != 0:
					node.set_group(grid[col + 1][row].get_group())
				elif len(grid[col - 1][row].get_group()) != 0:
					node.set_group(grid[col - 1][row].get_group())
				else:
					node.set_group([group])
					group = group + 1
			
	start_time = datetime.datetime.now()
	edge_node = find_edge(grid, barrier, start, end)
	if print_edge:
		print("number of obstacles: " + str(len(barrier)))
		print("number of edges: " + str(len(edge_node)))

	for node in edge_node:
		node.make_edge()
		node.find_visible_neighbor(grid, edge_node)
	print('begin astar')
	return astar(grid, start, end, print_length, start_time)

def algorithm(grid, start, end, print_length):
	start_time = datetime.datetime.now()
	for col in grid:
		for node in col:
			node.update_neighbors(grid)
	return astar(grid, start, end, print_length, start_time)

def astar(grid, start, end, print_length, start_time):
	open_set = PriorityQueue()
	open_set.put((start.get_f(), start))
	
	#keep ttrack of the origin of each node
	came_from = {}

	start.g_score = 0
	open_set_hash = {start}

	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

		current = open_set.get()[1]
		open_set_hash.remove(current)

		if current == end:
			delta = datetime.datetime.now() - start_time
			compute_time = int(delta.total_seconds() * 1000)
			length, coord_list = reconstruct_path(came_from, end)
			if print_length:
				print("Length: " + str(length))
			end.make_end()
			start.make_start()
			return compute_time, coord_list

		for neighbor in current.neighbors:
			temp_g_score = current.g_score + h(neighbor.get_pos(), current.get_pos())

			if temp_g_score < neighbor.g_score:
				came_from[neighbor] = current
				neighbor.g_score = temp_g_score
				neighbor.f_score = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					open_set.put((neighbor.f_score, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()
		if current != start:
			current.make_closed()
	return -1, []

def make_grid(cols, width):
	grid = []
	gap = width // cols
	for i in range(cols):
		grid.append([])
		for j in range(cols):
			node = Node(i, j, gap, cols)
			grid[i].append(node)

	return grid


def draw_grid(win, cols, width):
	gap = width // cols
	for i in range(cols):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(cols):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, coord_list, cols, width, compute_time):
	win.fill(WHITE)

	text = FONT.render('Compute Time: ' + str(compute_time) + ' milliseconds', 1, (0,0,0))

	for col in grid:
		for node in col:
			node.draw(win)
	
	for pair in coord_list:
		origin = pair[0]
		destination = pair[1]
		pygame.draw.line(win, YELLOW, origin, destination,3)

	draw_grid(win, cols, width)

	win.blit(text, (20, 20))
	pygame.display.update()


def get_clicked_pos(pos, cols, width):
	gap = width // cols
	x, y = pos

	col = x // gap
	row = y // gap

	return col, row


def main(win, width):
	colS = 50
	compute_time = 0
	grid = make_grid(colS, width)
	coord_list = []

	start = None
	end = None

	run = True


	while run:
		draw(win, grid, coord_list, colS, width, compute_time)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False
				
			if pygame.mouse.get_pressed()[0]: # LEFT
				pos = pygame.mouse.get_pos()
				col, row = get_clicked_pos(pos, colS, width)
				node = grid[col][row]
				if not start:
					start = node
					start.make_start()

				elif not end and node != start:
					end = node
					end.make_end()

				elif node != end and node != start:
					node.make_barrier()

			elif pygame.mouse.get_pressed()[2]: # RIGHT
				pos = pygame.mouse.get_pos()
				col, row = get_clicked_pos(pos, colS, width)
				node = grid[col][row]
				node.reset()
				if node == start:
					start = None
				elif node == end:
					end = None

			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					coord_list = []
					for row in grid:
						for node in row:
							if not node.is_barrier() and not node.is_end() and not node.is_start():
								node.reset()
					compute_time, coord_list = algorithm(grid, start, end, True)
					draw(win, grid, coord_list, colS, width, compute_time)
					print(compute_time)

				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(colS, width)
					coord_list = []

				if event.key == pygame.K_v and start and end:
					coord_list = []
					for row in grid:
						for node in row:
							if not node.is_barrier() and not node.is_end() and not node.is_start():
								node.reset()
					compute_time, coord_list = visibility(grid, start, end, True, True)
					"""
					#Check if algorithm is running correctly. Will high light node visible from start YELLOW.
					for node in start.neighbors:
						node.vis_from_start()
					"""
					draw(win, grid, coord_list, colS, width, compute_time)

				if event.key == pygame.K_d:
					with open('/Users/a212807082/Desktop/new_grid.csv', 'w', newline='') as new_file:
						csv_writer = csv.writer(new_file)
						csv_writer.writerow(['col', 'row'])
						x, y = start.get_pos()
						csv_writer.writerow([x, y])
						x, y = end.get_pos()
						csv_writer.writerow([x, y])
						for row in grid:
							for node in row:
								if node.is_barrier():
									x, y = node.get_pos()
									csv_writer.writerow([x, y])

				if event.key == pygame.K_l:
					with open('/Users/a212807082/Desktop/test_grid.csv','r') as csv_file:
						csv_reader = csv.reader(csv_file)
						next(csv_reader)
						x, y = next(csv_reader)
						start = grid[int(x)][int(y)]
						start.make_start()
						x, y = next(csv_reader)
						end = grid[int(x)][int(y)]
						end.make_end()
						for line in csv_reader:
							node = grid[int(line[0])][int(line[1])]
							node.make_barrier()

				if event.key == pygame.K_t and start and end:
					count = 100
					with open('/Users/a212807082/Desktop/test_result.csv', 'w', newline='') as new_file:
						csv_writer = csv.writer(new_file)
						csv_writer.writerow(['reg_time', 'vis_time'])
						while count > 0:
							coord_list = []
							for row in grid:
								for node in row:
									if not node.is_barrier() and not node.is_end() and not node.is_start():
										node.reset()
							reg_time, coord_list = algorithm(grid, start, end, count == 1)

							for row in grid:
								for node in row:
									if not node.is_barrier() and not node.is_end() and not node.is_start():
										node.reset()
							vis_time, coord_list = visibility(grid, start, end, count == 1, count == 1)
							count = count - 1
							csv_writer.writerow([reg_time, vis_time])

	pygame.quit()

main(WIN, WIDTH)