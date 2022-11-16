from vpython import *
import numpy as np
import math
import matplotlib.pyplot as plt


class ROBOT:

    def __init__(self, initX = 0, initY = 0, initZ = 0):
        
        self.gravity = 9.8
        self.k = 5000
        self.k_c = 100000
        self.T = 0
        self.dt = 0.00005
        self.friction_coefficient = 0.6
        self.damping_constant = 0.9999

        self.dots = self.generateDots(initX, initY, initZ)
        self.springs = self.generateSprings()
        

    def generateDots(self, initX, initY, initZ):

        #! the initiated height
        off = 4.0
        og_mass_positions = [
                            vector(initX + 0.1, initY + off, initZ + 0.1), 
                            vector(initX + 0.1, initY + off, initZ - 0.1), 
                            vector(initX - 0.1, initY + off, initZ - 0.1), 
                            vector(initX - 0.1, initY + off, initZ + 0.1),
                            vector(initX + 0.1, initY + 0.1 + off, initZ + 0.1), 
                            vector(initX + 0.1, initY + 0.1 + off, initZ - 0.1), 
                            vector(initX - 0.1, initY + 0.1 + off, initZ - 0.1), 
                            vector(initX - 0.1, initY + 0.1 + off, initZ + 0.1),
                            vector(initX + 0.15, initY - 0.1 + off, initZ + 0.1), 
                            vector(initX + 0.15, initY - 0.1 + off, initZ - 0.1), 
                            vector(initX - 0.15, initY - 0.1 + off, initZ - 0.1), 
                            vector(initX - 0.15, initY - 0.1 + off, initZ + 0.1)
                            ]
        cube_dots = []
        for pos in og_mass_positions:
            cube_dots.append(self.Mass(m=0.1, p=pos, v=vector(0, 0, 0), a=vector(0, 0, 0)))

        return cube_dots

    def generateSprings(self):
        
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

        #! L_0 = L_0 + A*sin(B*t+C)
        A = 0.03
        B = 3
        C = 0
        # print(self.T)

        target = np.array([[2.40564349e-02, 1.03368988e+01, 6.16093071e+00],
 [-1.89864519e-01, -1.02317037e+01, -6.25861556e+00],
 [ 1.92398550e-01, -1.06078955e+01, -9.41945465e-01],
 [-2.65968028e-02, -1.22321816e+01, -1.43523920e+00]])

        if self.T > 0.001:
            # self.springs[-1].L_0 = 0.1118 + target[0][0] * math.sin(target[0][1] * self.T + target[0][2])
            # self.springs[-2].L_0 = 0.1118 + target[1][0] * math.sin(target[1][1] * self.T + target[1][2])
            # self.springs[-3].L_0 = 0.1118 + target[2][0] * math.sin(target[2][1] * self.T + target[2][2])
            # self.springs[-4].L_0 = 0.1118 + target[3][0] * math.sin(target[3][1] * self.T + target[3][2])
            # self.springs[-1].L_0 = 0.1118 + A * math.sin(B * self.T + C)
            # self.springs[-2].L_0 = 0.1118 + A * math.sin(B * self.T + C + math.pi / 2)
            # self.springs[-3].L_0 = 0.1118 + A * math.sin(B * self.T + C + math.pi / 2)
            # self.springs[-4].L_0 = 0.1118 + A * math.sin(B * self.T + C)
            pass
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
                # print('aaaaaaaaaaaaaaaaaaaaaaaa')
                # print(f'this is the y {self.dots[i].pos.y}')
                # print(f'this is the forces {robot_forces[i][1]}')
                robot_forces[i][1] = robot_forces[i][1] + self.k_c * abs(self.dots[i].pos.y)
                print(f'this is the forces {robot_forces[i][1]}')

                #! we can add the friction here
                F_horizontal = math.sqrt(robot_forces[i][0] ** 2 + robot_forces[i][2] ** 2)
                F_vertical =self.k_c * abs(self.dots[i].pos.y)
                # print(f'this is the hori {F_horizontal}')
                # print(f'this is the ver {F_vertical}')
                if F_horizontal < F_vertical * self.friction_coefficient:
                    robot_forces[i][0] = 0
                    robot_forces[i][2] = 0
                    self.dots[i].vel.x = 0
                    self.dots[i].vel.z = 0
                else:
                    robot_forces[i][0] = robot_forces[i][0] + F_vertical * self.friction_coefficient
                    pass


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

class SIMULATION:

    def __init__(self):
        
        pass

    def step(self):
        
        pass


if __name__ == "__main__":


    robot = ROBOT()
    print(robot.dots[1].pos.z)
    counter = 0 #! determine the frequency of the simulation
    
    GUI = True

    
    if GUI == True:

        def make_grid(xmax, dx, scale=15):
            for x in range(-xmax, xmax + dx, dx):  # Create vertical lines
                curve(pos=[vector(x / scale, 0, xmax / scale), vector(x / scale, 0, -xmax / scale)], radius=0.001)
            for z in range(-xmax, xmax + dx, dx):  # Create horizontal lines
                curve(pos=[vector(xmax / scale, 0, z / scale), vector(-xmax / scale, 0, z / scale)], radius=0.001)

        scene.height = 720
        scene.width = 1280
        make_grid(xmax=30, dx=1)
        # floor = box(pos=vec(0, -0.1, 0), size=vec(10, 0.001, 10), color=color.purple)
        lamp = local_light(pos=vector(1,1,1), color=color.white)

        # Plot masses
        def display_masses(mass_array):
            spheres = []
            for mass in mass_array:
                pt = sphere(pos=mass.pos, radius=0.01, color=color.green)
                spheres.append([pt, mass])
            return spheres # returns a list of Sphere objects that are plotted in vpython
        spheres = display_masses(robot.dots)
        # print(spheres)

        # Plot springs
        def display_springs(cube_springs):
            lines = []
            for spring in cube_springs:
                cyl = cylinder(pos=spring.m1.pos, axis=spring.axis, color=color.blue, radius=0.001)
                lines.append([cyl, spring])
            return lines # returns a list of Cylinder objects that are plotted in vpython
        rods = display_springs(robot.springs)

        

        while 1:

            counter += 1
            dots_pos = robot.robotupdate()
            # print(dots_pos)

            if counter % 10 == 0:
                for i in range(len(dots_pos)):
                    spheres[i][0].pos = dots_pos[i]

                for i in range(len(rods)):
                    rod = rods[i][0]
                    spring = rods[i][1]
                    rod.pos = dots_pos[spring.i1]
                    rod.axis = dots_pos[spring.i2] - dots_pos[spring.i1]

    dt = 0.0002
    T = 0
    velocity = []
    terminate_time = 15
    start_time = 1

    while T < terminate_time:

        T += dt

        dots_pos = robot.robotupdate()
        # print(dots_pos)
        if T > start_time:
            x_distance = (dots_pos[0].x + dots_pos[1].x + dots_pos[2].x + dots_pos[3].x) / 4
            z_distance = (dots_pos[0].z + dots_pos[1].z + dots_pos[2].z + dots_pos[3].z) / 4
            # print(x_distance)
            # print(z_distance)

            distance = math.sqrt(x_distance ** 2 + z_distance ** 2)
            velocity.append(distance / T)

    # print(velocity)
    del(velocity[-1])
    T_plt = np.linspace(start_time, terminate_time, int((terminate_time - start_time)/0.0002))
    plt.plot(T_plt, velocity, label='aa')
    plt.xlabel("Time", fontsize=24, fontweight='heavy')
    plt.ylabel("Velocity", fontsize=24, fontweight='heavy')
    plt.xticks(size=25)
    plt.yticks(size=25)
    plt.title("Velocity for the Best Robot", fontsize=30, fontweight='bold')
    # plt.legend(loc='best', fontsize=20)
    plt.show()
