from vpython import *
import numpy as np
import math



class ROBOT:

    def __init__(self, dt = 0.001, initX = 0, initY = 0, initZ = 0, gene = np.zeros([4, 3])):
        
        self.gravity = 9.8
        self.k = 10000
        self.k_c = 100000
        self.T = 0
        self.dt = dt
        self.friction_coefficient = 0.9
        self.damping_constant = 0.99
        self.initX = initX
        self.initY = initY
        self.initZ = initZ

        # self.reset_dots_springs()

        self.dots = self.generate_dots()
        self.springs = self.generate_springs()
        self.gene = gene
        
    # def reset_dots_springs(self):

    #     self.dots = self.generate_dots()
    #     self.springs = self.generate_springs()

    def generate_dots(self):

        #! the initiated height
        off = 0.15
        og_mass_positions = [
                            vector(self.initX + 0.1, self.initY + off, self.initZ + 0.1), 
                            vector(self.initX + 0.1, self.initY + off, self.initZ - 0.1), 
                            vector(self.initX - 0.1, self.initY + off, self.initZ - 0.1), 
                            vector(self.initX - 0.1, self.initY + off, self.initZ + 0.1),
                            vector(self.initX + 0.1, self.initY + 0.1 + off, self.initZ + 0.1), 
                            vector(self.initX + 0.1, self.initY + 0.1 + off, self.initZ - 0.1), 
                            vector(self.initX - 0.1, self.initY + 0.1 + off, self.initZ - 0.1), 
                            vector(self.initX - 0.1, self.initY + 0.1 + off, self.initZ + 0.1),
                            vector(self.initX + 0.15, self.initY - 0.1 + off, self.initZ + 0.1), 
                            vector(self.initX + 0.15, self.initY - 0.1 + off, self.initZ - 0.1), 
                            vector(self.initX - 0.15, self.initY - 0.1 + off, self.initZ - 0.1), 
                            vector(self.initX - 0.15, self.initY - 0.1 + off, self.initZ + 0.1)
                            ]
        cube_dots = []
        for pos in og_mass_positions:
            cube_dots.append(self.Mass(m=0.1, p=pos, v=vector(0, 0, 0), a=vector(0, 0, 0)))

        return cube_dots

    def generate_springs(self):
        
        cube_springs = []
        break_flag = False
        for i in range(len(self.dots)):

            for j in range(i + 1, len(self.dots)):

                if i == 8:
                    break_flag = True
                    break

                posdiff = self.dots[j].pos - self.dots[i].pos
                dist = mag(posdiff)

                if j >= 8 and dist > 0.270:
                    continue

                #! generate some special springs to move
                # if dist > 0.287 or 0.101 < dist < 0.13:
                #     continue

                cube_springs.append(self.Spring(L_0=dist, m1=self.dots[i], m2=self.dots[j], i1=i, i2=j, k=self.k))
            if break_flag == True:
                break
        
        #! generate some special springs to move
        posdiff = self.dots[8].pos - self.dots[0].pos
        dist = mag(posdiff)
        cube_springs.append(self.Spring(L_0=dist, m1=self.dots[i], m2=self.dots[j], i1=0, i2=8, k=self.k))
        cube_springs.append(self.Spring(L_0=dist, m1=self.dots[i], m2=self.dots[j], i1=1, i2=9, k=self.k))
        cube_springs.append(self.Spring(L_0=dist, m1=self.dots[i], m2=self.dots[j], i1=2, i2=10, k=self.k))
        cube_springs.append(self.Spring(L_0=dist, m1=self.dots[i], m2=self.dots[j], i1=3, i2=11, k=self.k))

        return cube_springs

    def robotupdate(self):

        robot_forces = [[0]*3 for _ in range(len(self.springs))]
        # print(robot_forces)

        #! L_0 = L_0 + self.gene[0]*sin(self.gene[1]*t+self.gene[2])
        # self.gene[0][0] = 0.05
        # self.gene[0][1] = 3
        # self.gene[0][2] = 0
        # print(self.T)
        if self.T > 0.001:
            # self.springs[-1].L_0 = 0.12241 + self.gene[0][0] * math.sin(self.gene[0][1] * self.T + self.gene[0][2])
            # self.springs[-2].L_0 = 0.12241 + self.gene[0][0] * math.sin(self.gene[0][1] * self.T + self.gene[0][2] + math.pi / 2)
            # self.springs[-3].L_0 = 0.12241 + self.gene[0][0] * math.sin(self.gene[0][1] * self.T + self.gene[0][2] + math.pi / 2)
            # self.springs[-4].L_0 = 0.12241 + self.gene[0][0] * math.sin(self.gene[0][1] * self.T + self.gene[0][2])
            self.springs[-1].L_0 = 0.12241 + self.gene[0][0] * math.sin(self.gene[0][1] * self.T + self.gene[0][2])
            self.springs[-2].L_0 = 0.12241 + self.gene[1][0] * math.sin(self.gene[1][1] * self.T + self.gene[1][2])
            self.springs[-3].L_0 = 0.12241 + self.gene[2][0] * math.sin(self.gene[2][1] * self.T + self.gene[2][2])
            self.springs[-4].L_0 = 0.12241 + self.gene[3][0] * math.sin(self.gene[3][1] * self.T + self.gene[3][2])

        self.T += self.dt

        #! calculate the spring
        for i in range(len(self.springs)):
            mass1 = self.dots[self.springs[i].i1]
            mass2 = self.dots[self.springs[i].i2]
            # print(mass2.pos, mass1.pos)
            posdiff = mass2.pos - mass1.pos
            L = mag(posdiff)
            force = self.springs[i].k * abs(self.springs[i].L_0 - L)
            direction = posdiff / L
            # print("this is dir", direction)
            
            if L > self.springs[i].L_0:
                robot_forces[self.springs[i].i1][0] = robot_forces[self.springs[i].i1][0] + direction.x * force
                robot_forces[self.springs[i].i1][1] = robot_forces[self.springs[i].i1][1] + direction.y * force
                robot_forces[self.springs[i].i1][2] = robot_forces[self.springs[i].i1][2] + direction.z * force
                robot_forces[self.springs[i].i2][0] = robot_forces[self.springs[i].i2][0] - direction.x * force
                robot_forces[self.springs[i].i2][1] = robot_forces[self.springs[i].i2][1] - direction.y * force
                robot_forces[self.springs[i].i2][2] = robot_forces[self.springs[i].i2][2] - direction.z * force
            elif L < self.springs[i].L_0:
                robot_forces[self.springs[i].i1][0] = robot_forces[self.springs[i].i1][0] - direction.x * force
                robot_forces[self.springs[i].i1][1] = robot_forces[self.springs[i].i1][1] - direction.y * force
                robot_forces[self.springs[i].i1][2] = robot_forces[self.springs[i].i1][2] - direction.z * force
                robot_forces[self.springs[i].i2][0] = robot_forces[self.springs[i].i2][0] + direction.x * force
                robot_forces[self.springs[i].i2][1] = robot_forces[self.springs[i].i2][1] + direction.y * force
                robot_forces[self.springs[i].i2][2] = robot_forces[self.springs[i].i2][2] + direction.z * force

        new_pos = []

        for i in range(len(self.dots)):

            #! calculate the gravity
            robot_forces[i][1] = robot_forces[i][1] - self.dots[i].m * self.gravity

            if self.dots[i].pos.y < 0:
                # print(f'this is the y {self.dots[i].pos.y}')
                # print(f'this is the forces {robot_forces[i][1]}')
                robot_forces[i][1] = robot_forces[i][1] + self.k_c * abs(self.dots[i].pos.y)

                #! we can add the friction here
                F_horizontal = math.sqrt(robot_forces[i][0] ** 2 + robot_forces[i][2] ** 2)
                F_vertical =self.k_c * abs(self.dots[i].pos.y)
                V_horizontal = math.sqrt(self.dots[i].vel.x ** 2 + self.dots[i].vel.z ** 2)
                # print(f'this is the hori {F_horizontal}')
                # print(f'this is the ver {F_vertical}')
                if F_horizontal < F_vertical * self.friction_coefficient:
                    robot_forces[i][0] = 0
                    robot_forces[i][2] = 0
                    self.dots[i].vel.x = 0
                    self.dots[i].vel.z = 0
                else:
                    if V_horizontal != 0:
                        robot_forces[i][0] = robot_forces[i][0] - F_vertical * self.friction_coefficient * self.dots[i].vel.x / V_horizontal
                        robot_forces[i][2] = robot_forces[i][2] - F_vertical * self.friction_coefficient * self.dots[i].vel.z / V_horizontal

            self.dots[i].acc.x = robot_forces[i][0] / self.dots[i].m
            self.dots[i].acc.y = robot_forces[i][1] / self.dots[i].m
            self.dots[i].acc.z = robot_forces[i][2] / self.dots[i].m

            self.dots[i].vel.x = self.damping_constant * (self.dots[i].vel.x + self.dots[i].acc.x * self.dt)
            self.dots[i].vel.y = self.damping_constant * (self.dots[i].vel.y + self.dots[i].acc.y * self.dt)
            self.dots[i].vel.z = self.damping_constant * (self.dots[i].vel.z + self.dots[i].acc.z * self.dt)

            self.dots[i].pos.x = self.dots[i].pos.x + self.dots[i].vel.x * self.dt
            self.dots[i].pos.y = self.dots[i].pos.y + self.dots[i].vel.y * self.dt
            self.dots[i].pos.z = self.dots[i].pos.z + self.dots[i].vel.z * self.dt

            new_pos.append(self.dots[i].pos)
        
        # print(self.dots[11].pos.x)
        # print(self.T)
        
        return new_pos

    class Mass:
        def __init__(self, m, p, v, a):
            self.m = m
            self.pos = p
            self.vel = v
            self.acc = a

    class Spring:
        def __init__(self, L_0, m1, m2, i1, i2, k):
            self.k = k
            self.L_0 = L_0
            self.axis = m2.pos - m1.pos
            # self.L = sqrt(self.axis.x ** 2 + self.axis.y ** 2 + self.axis.z ** 2)
            self.m1 = m1
            self.m2 = m2
            # self.pos = m1.pos
            # keep track of indices of associated masses
            self.i1 = i1
            self.i2 = i2

class MULTI_SIMULATION:

    def __init__(self):

        self.population_size = 8
        self.evaluations = 2
        self.sim_time = 50
        self.dt = 0.001
        self.sim_flag = True

        self.step()

    def genereate_robots(self, population_gene):

        robots = []
        factor = []
        X = 0.5
        Z = 0.5
        for i in range(1, self.population_size + 1):
            if self.population_size % i == 0:
                factor.append(i)

        if len(factor) % 2 == 0:
            configuration = [factor[int(len(factor) / 2 - 1)], factor[int(len(factor) / 2)]]
        else:
            configuration = [factor[int((len(factor) - 1) / 2)], factor[int((len(factor) - 1) / 2)]]

        # print(population_gene[i])
        i = 0
        for a in range(configuration[0]):
            for b in range(configuration[1]):
                robots.append(ROBOT(dt=self.dt, initX=a*X, initY=0, initZ=b*Z, gene=population_gene[i]))
                i += 1

        return robots

    def generate_gene(self):

        population_gene = []

        for i in range(self.population_size):
            gene_per_robot = []
            #! L=L_0+A*sin(B*t+C)
            for j in range(4):
                A = np.random.uniform(-0.06, 0.06)
                B = np.random.uniform(-math.pi, math.pi)
                C = np.random.uniform(-math.pi * 2, math.pi * 2)
                gene_per_robot.append([A, B, C])
            population_gene.append(gene_per_robot)

        population_gene = np.asarray(population_gene)
        # print(population_gene)

        return population_gene

    def vpython_render(self, robots, spheres, rods):

        T = 0
        counter = 0

        if self.sim_flag == True:

            while T < self.sim_time:

                counter += 1
                #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                dots_pos = []
                for i in range(self.population_size):
                    dots_pos.append(robots[i].robotupdate())
                #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                # print(f'this is dots pos {dots_pos[0][0]}')
                # print(dots_pos)
                # spheres[0][1][0].pos = dots_pos[0][1]

                if counter % 10 == 0:
                    for j in range(self.population_size):

                        for i in range(len(dots_pos[j])):
                            spheres[j][i][0].pos = dots_pos[j][i]

                        for i in range(len(rods[j])):
                            rod = rods[j][i][0]
                            spring = rods[j][i][1]
                            rod.pos = dots_pos[j][spring.i1]
                            rod.axis = dots_pos[j][spring.i2] - dots_pos[j][spring.i1]
                T += self.dt

        else:
            while T < self.sim_time:
                dots_pos = []
                for i in range(self.population_size):
                    # print(f'this is robot {i}')
                    dots_pos.append(robots[i].robotupdate())
                T += self.dt
        
        return dots_pos

    def calculate_fitness(self, dots_pos, robots, num):

        population_fitness = []
        # print(f'this is dots pos {dots_pos}')
        for i in range(self.population_size):

            x_distance = (dots_pos[i][0].x + dots_pos[i][1].x + dots_pos[i][2].x + dots_pos[i][3].x) / 4 - robots[i].initX
            z_distance = (dots_pos[i][0].z + dots_pos[i][1].z + dots_pos[i][2].z + dots_pos[i][3].z) / 4 - robots[i].initZ
            # print(x_distance)
            # print(z_distance)

            distance = math.sqrt(x_distance ** 2 + z_distance ** 2)
            population_fitness.append(distance)

        average_fitness = np.mean(population_fitness)
        max_fitness = np.max(population_fitness)
        arg = np.argmax(population_fitness)

        # print(f'this is all fitness {population_fitness}')
        # print(f'In evaluation {num}, the max fitness is {max_fitness}.')

        return population_fitness, max_fitness, robots[arg].gene

    def selection(self, population_fitness, robots):

        population_gene = []
        # population_fitness = -np.sort((-np.asarray(population_fitness)))
        population_index = np.argsort((-np.asarray(population_fitness)))

        for i in range(int(self.population_size / 2)):
            population_gene.append(robots[population_index[i]].gene)
        population_gene = np.asarray(population_gene)
        # print(population_gene)

        return population_gene
    
    def crossover(self, population_gene):

        #! pair the parents
        used_list = []
        new_population_gene = []
        for i in range(int(population_gene.shape[0] / 2)):

            parent_index = np.random.choice(population_gene.shape[0], 2, replace=False)
            while parent_index[0] in used_list or parent_index[1] in used_list:
                parent_index = np.random.choice(population_gene.shape[0], 2, replace=False)
            print(f'this is parent index {parent_index}')
            print(f'this is used index {used_list}')
            used_list.append(parent_index[0])
            used_list.append(parent_index[1])

            parent_1 = population_gene[parent_index[0]].reshape(-1)
            parent_2 = population_gene[parent_index[1]].reshape(-1)
            cross_point = np.random.choice(len(parent_1) - 1) + 1
            child_1 = np.concatenate([parent_1[:cross_point], parent_2[cross_point:]])
            child_2 = np.concatenate([parent_2[:cross_point], parent_1[cross_point:]])

            new_population_gene.append(parent_1.reshape(4,3))
            new_population_gene.append(parent_2.reshape(4,3))
            new_population_gene.append(child_1.reshape(4,3))
            new_population_gene.append(child_2.reshape(4,3))

        new_population_gene = np.asarray(new_population_gene)
        # print(new_population_gene)
        return new_population_gene

    def mutation():

        pass

    def step(self):

        max_fitness = -np.inf
        best_gene = 0

        if self.sim_flag == True:

            population_gene = self.generate_gene()

            robots = self.genereate_robots(population_gene)
            # print(f'this is the number of robots: {len(robots)}')

            def make_grid(xmax, dx, scale=15):
                    for x in range(-xmax, xmax + dx, dx):  # Create vertical lines
                        curve(pos=[vector(x / scale, 0, xmax / scale), vector(x / scale, 0, -xmax / scale)], radius=0.001)
                    for z in range(-xmax, xmax + dx, dx):  # Create horizontal lines
                        curve(pos=[vector(xmax / scale, 0, z / scale), vector(-xmax / scale, 0, z / scale)], radius=0.001)

            scene.height = 720
            scene.width = 1280
            make_grid(xmax=30, dx=1)
            lamp = local_light(pos=vector(1,1,1), color=color.white)

            # Plot masses
            def display_masses(mass_array):
                spheres = []
                for mass in mass_array:
                    pt = sphere(pos=mass.pos, radius=0.01, color=color.green)
                    spheres.append([pt, mass])
                return spheres # returns a list of Sphere objects that are plotted in vpython
            spheres = []
            for i in range(self.population_size):
                spheres.append(display_masses(robots[i].dots))

            # Plot springs
            def display_springs(cube_springs):
                lines = []
                for spring in cube_springs:
                    cyl = cylinder(pos=spring.m1.pos, axis=spring.axis, color=color.blue, radius=0.001)
                    lines.append([cyl, spring])
                return lines # returns a list of Cylinder objects that are plotted in vpython
            rods = []
            for i in range(self.population_size):
                rods.append(display_springs(robots[i].springs))
        
        else:
            spheres = 0
            rods = 0
        
        population_gene = self.generate_gene() #! this is numpy array
        robots = self.genereate_robots(population_gene)
        print(f'this is the number of robots: {len(robots)}')


        for i in range(self.evaluations):

            dots_pos = self.vpython_render(robots, spheres, rods)

            population_fitness, fitness, gene = self.calculate_fitness(dots_pos, robots, i)

            if fitness > max_fitness:
                max_fitness = fitness
                best_gene = gene
                print(f'In evaluation {i}, the max fitness is {max_fitness}.')
                print(f'In evaluation {i}, the best gene is\n {best_gene}.')

            parent_population_gene = self.selection(population_fitness, robots)

            new_population_gene = self.crossover(parent_population_gene)

            robots = self.genereate_robots(new_population_gene)





if __name__ == "__main__":

    multi_simulation = MULTI_SIMULATION()
    
    # GUI = False
    
    # if GUI == True:

    #     def make_grid(xmax, dx, scale=15):
    #         for x in range(-xmax, xmax + dx, dx):  # Create vertical lines
    #             curve(pos=[vector(x / scale, 0, xmax / scale), vector(x / scale, 0, -xmax / scale)], radius=0.001)
    #         for z in range(-xmax, xmax + dx, dx):  # Create horizontal lines
    #             curve(pos=[vector(xmax / scale, 0, z / scale), vector(-xmax / scale, 0, z / scale)], radius=0.001)

    #     scene.height = 720
    #     scene.width = 1280
    #     make_grid(xmax=30, dx=1)
    #     # lamp = local_light(pos=vector(1,1,1), color=color.white)
    #     # color = color.green

    #     # Plot masses
    #     def display_masses(mass_array):
    #         spheres = []
    #         for mass in mass_array:
    #             pt = sphere(pos=mass.pos, radius=0.01, color=color)
    #             spheres.append([pt, mass])
    #         return spheres # returns a list of Sphere objects that are plotted in vpython
    #     spheres = display_masses(robot.dots)
    #     # print(spheres)

    #     # Plot springs
    #     def display_springs(cube_springs):
    #         lines = []
    #         for spring in cube_springs:
    #             cyl = cylinder(pos=spring.m1.pos, axis=spring.axis, color=color, radius=0.001)
    #             lines.append([cyl, spring])
    #         return lines # returns a list of Cylinder objects that are plotted in vpython
    #     rods = display_springs(robot.springs)

    #     while 1:

    #         counter += 1
    #         #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #         dots_pos = robot.robotupdate()
    #         #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #         # print(dots_pos)

    #         if counter % 10 == 0:
    #             for i in range(len(dots_pos)):
    #                 spheres[i][0].pos = dots_pos[i]

    #             for i in range(len(rods)):
    #                 rod = rods[i][0]
    #                 spring = rods[i][1]
    #                 rod.pos = dots_pos[spring.i1]
    #                 rod.axis = dots_pos[spring.i2] - dots_pos[spring.i1]
    # else:

    #     print('over')
    #     # while 1:

    #     #     counter += 1

    #     #     dots_pos = robot.robotupdate()
    #     #     print(dots_pos)