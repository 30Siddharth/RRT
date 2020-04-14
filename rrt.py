import numpy as np 
from matplotlib import pyplot as plt
import math

class rrt_star(object):
    def __init__(self,start, goal, xlim, ylim,tree,obss):
        self.start = start
        self.goal = goal
        self.xlim = xlim
        self.ylim = ylim 
        self.sample = [0,0]
        self.tree = tree
        self.nrstpt = start
        self.obss = obss
        self.goal_flag = False
        self.last = goal

    def sampling(self):
        # returns a state x that is sampled uniformly from the domain
        self.sample[0] = np.random.uniform(self.xlim[0],self.xlim[1])
        self.sample[1] = np.random.uniform(self.ylim[0],self.ylim[1])


    def steer(self,x1,x2):
        # returns the optimal control trajectory of travelling from x1 to x2
        # and the cost
        # pass
        x_steer = [0,0]  # It is the reachable point
        '''
        Notes about the steer: the actually next step can be found as xt' = xt + u*del_t
        The input is fixed to not exceed 1. The biggest step hence is fixed by the step size in
        time. xt' = xt + del_t. The current setup is holonomic, hence we can freely steer to any point. 
        A bigger question that arises here is the step size in time. 
        If the sampling point is not within a circle from the nearest node with a radius of del_t then 
        the best we can do is travel with max speed, i.e. 1 towards the sampled point and reach the new point.
        This point will lie on the edge of the circle with radius del_t. 
        If the sampled point is within the circle then we need not travel with max speed. This assumption is based
        on the idea that the control frequency is fixed and that one input is implemented in a single time step.
        Hence we will not travel with max speed always. We will assume that travelling between all the point takes del_t, 
        and hence if the point lies within the circle then the we will travel with a speed less than the max speed. 
        Basically we are not choosing bang-bang control and we will achieve our output in terms of the del_t. We can fix the del_t later.
        '''
        if np.linalg.norm(x1-x2) < 1: 
            x_steer = x2
        else:
            # slope = /(x1[0] - x2[0])
            theta = math.atan2((x2[1] - x1[1]),(x2[0] - x1[0]))
            x_steer[0] = x1[0] + math.cos(theta)
            x_steer[1] = x1[1] + math.sin(theta)

        x_cost = np.linalg.norm(x_steer - x1)
        return x_steer, x_cost


    def is_in_collision(self,p1,p2):
        # returns True if a path between p1 and p2 of the robot is in collision with any of the obstacles
        red_flag = False
        m = (p2[1]-p1[1])/(p2[0]-p1[0])
        c = p1[1] - m*p1[0]
        xs = np.arange(p1[0],p2[0]+0.01,0.01)  # sample ponits at an interval along x
        ys = np.zeros_like(xs)
        for i in range(len(xs)):
            ys[i] =  m*xs[i] + c
        
        for i in range(len(self.obss)): #check for all obstacles
            xl = self.obss[i][0] 
            xu = self.obss[i][1]
            yl = self.obss[i][2]
            yu = self.obss[i][3]
            #If it is within the bounds in any direction then there is collision
            if (xl <= xs.any() <= xu) and (yl <= ys.any() <= yu):
                red_flag = True
                
        return red_flag

        

    def nearest(self):
        # finds a node in the tree that is closest to the state x
        # (Closest in what metric?) ---> Looking @ Eucledian distance
        # Store all the keys (i.e. nodes of the tree in a point)
        arr = []
        for k in self.tree.keys():
            arr = arr + [list(k)]
        nodes = np.array(arr)
        # print(self.sample*np.ones((len(nodes),1))-nodes)
        print(self.sample*np.ones((len(nodes),1))-nodes)
        dist = np.linalg.norm(self.sample*np.ones((len(nodes),1))-nodes, axis = 1)
        # dist = np.linalg.norm(self.sample*np.ones((1,len(nodes)))-nodes, axis = 1)
        min_idx = np.argmin(dist)
        self.nrstpt = nodes[min_idx]


    def connect(self,x_new,parent,new_cost):
        # add state x to the tree
        # cost of state x = cost(parent) + cost of steer(parent,x)
        new_cost_to_go = self.tree[tuple(parent)][1] +  new_cost
        self.tree[tuple(x_new)] = (tuple(parent),new_cost_to_go)


    def rewire(self,x):
    #rewire all the nodes in the tree with the O(gamma*log n/n)^{1/d}) ball
    # near the state x, update the costs of all rewired neighbours
        pass
    
    def check_goal(self,p1):
        # Given the latest added point, check if we can reach the goal
        goal = np.array(self.goal)
        p1arr = np.array(p1)
        d = np.linalg.norm(goal - p1arr)
        if d < 1:
            # self.connect(self.goal,p1,d)
            self.last = p1
            self.goal_flag = True


    def solid(self):
        self.sampling()
        self.nearest()
        x_nearest = self.nrstpt
        x_new, new_cost = self.steer(x_nearest,self.sample)
        collision_flag = self.is_in_collision(x_nearest,x_new)
        if collision_flag is True:
            print('Collision detected!')
            # break
        else:
            self.connect(x_new,x_nearest,new_cost)
            self.check_goal(x_new)

        

# def get_obs_lines(vertcies):
#     #Given the vertices of the obstacles, get the equations of the lines. 
#     obs_dic = 
#     for i in range(len(vertcies)):


# def get_lin(p1,p2):
#     # Given two points, get the equation of the line passing through the points.
#     m = (p2[1]-p1[1])/(p2[0]-p1[0])
#     c = p1[1] - m*p1[0]

#     return tuple(m,c)


if __name__ == "__main__":
    # pass
    start = [0,0]
    goal = [8,8]
    xlimits = [-10,10]
    ylimits = [-10,10]
    tree = {tuple(start):(tuple(start),0)}
    obs1 = [4,5,-4,9]
    obs2 = [-6,0,-5,-4]
    obss = [obs1,obs2]
    # obs1_eqns = get_obs_lines(obs1)
    # obs2_eqns = get_obs_lines(obs2)
    graph = rrt_star(start,goal,xlimits, ylimits,tree,obss)
    N = 1000
    i = 0
    # for i in range(N):
    while i < N and graph.goal_flag is False:
        graph.solid() 
        i = i+1

        # Till this point we have done the follwoing things:
        # 1. Sampled a point
        # 2. Found the nearest point in the tree 
        # 3. Idenitfied the reachable point
        # 4. Checked for a collision free path and accordingly added the point in the tree or skipped the point

    fig = plt.figure(0)
    x,y = [],[]
    start_flag = False
    if graph.goal_flag is True:
        print('Yay!')
    for k in tree.keys():
        l = [[tree[k][0][0],k[0]],[tree[k][0][1],k[1]]]
        plt.plot(l[0],l[1], 'go--',linewidth=2, markersize=3)
    
    

    print('Done Plotting')

    k = tuple(graph.last)
    path = []
    xpath = []
    ypath = []
    while start_flag is False and graph.goal_flag is True:
        path = path + [k]
        # print(tree[k])
        print(k)
        if list(tree[k][0]) == start:
            start_flag = True
            # path = path + [tree[k][0]]
            # print(tree[k][0])
            xpath = xpath + [path[-1][0]]
            ypath = ypath + [path[-1][1]]           
        else:
            k = tree[k][0] 
            # path = path + [k]
            xpath = xpath + [path[-1][0]]
            ypath = ypath + [path[-1][1]]

            # plot()
        # print(xpath)
    xpath = [goal[0]] + xpath + [start[0]]
    ypath = [goal[1]] + ypath + [start[1]]
    plt.plot(xpath,ypath,'ro--')
    plt.plot(start[0],start[1],'cp',markersize=3)
    plt.plot(goal[0],goal[1],'bp',markersize=5)
    plt.grid()
    plt.show()
 


