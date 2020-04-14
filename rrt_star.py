import numpy as np 
from matplotlib import pyplot as plt 
import math
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle

class rrt_star(object):
    def __init__(self,start,goal,x_lim,y_lim,obss,tree):
        self.xlim = x_lim
        self.ylim = y_lim
        self.obss = obss
        self.tree = tree

        self.goal = goal
        self.near_goal = None
        self.goal_flag = False
    
    def sampling(self):
        x = np.array([0,0],dtype=float)
        x[0] = np.random.uniform(self.xlim[0],self.xlim[1])
        x[1] = np.random.uniform(self.xlim[0],self.xlim[1])

        return x

    def is_in_collision(self,x):
        col_flag = False
        for i in range(len(obss)):
            if i == 0:
                xl = self.obss[i][0] - 0.6
                xu = self.obss[i][1] + 0.6
                yl = self.obss[i][2] - 0.6
                yu = self.obss[i][3] + 0.6
                if xl <= x[0] <= xu:
                    if yl <= x[1] <= yu:
                        col_flag = True
                        break
                    else:
                        col_flag =  False
            
            elif i == 1:
                xl = self.obss[i][0] - 0.6
                xu = self.obss[i][1] + 0.6
                yl = self.obss[i][2] - 0.6
                yu = self.obss[i][3] + 0.6

                if xl <= x[0] <= xu:
                    if yl <= x[1] <= yu:
                        col_flag = True
                        break
                    else:
                        col_flag =  False
        
        return col_flag


    def nearest(self,x):
        arr = []
        for k in self.tree.keys():
            arr = arr + [list(k)]
        nodes = np.array(arr)
        dist = np.linalg.norm(np.ones((len(nodes),2))*x - nodes,axis = 1)
        min_idx = np.argmin(dist)
        return nodes[min_idx]

    def steer(self,x_nearest, x_sample):
        d = np.linalg.norm(x_nearest - x_sample)
        if d <= 1:
            return x_sample,d
        else:
            theta = math.atan2(x_sample[1] - x_nearest[1] , x_sample[0] - x_nearest[0])
            xnew = np.array([0,0],dtype = float)
            xnew[0] = x_nearest[0] + math.cos(theta)
            xnew[1] = x_nearest[1] + math.sin(theta)
            d = np.linalg.norm(x_nearest - xnew)
            return xnew,d

    def connect(self,new,parent,cost):
        # print(tuple(parent[0]))
        self.tree[tuple(new)] = (tuple(parent),cost)

    def Optimal_circle(self,xnew,parent):
        gamma = 10
        N = len(self.tree)
        radius = gamma*(np.log(N)/N)**0.5
        xmin = parent        
        costmin = self.tree[tuple(parent)][1] + np.linalg.norm(xnew - list(parent))
        for k in self.tree.keys():
            if tuple(xnew) != k:
                d = np.linalg.norm(xnew - self.tree[k][0])
                if d < radius:
                    cost_to_go = d + self.tree[k][1]
                    if cost_to_go < costmin and np.linalg.norm(xnew - self.tree[k][0]) <1:
                        xmin = k
                        costmin = cost_to_go

        self.tree[tuple(xnew)] = (tuple(xmin),costmin)

    def check_goal(self,x):
        if self.near_goal is None:
            if np.linalg.norm(np.array(x) - np.array(self.goal)) < 1:
                self.near_goal = x
                self.goal_flag = True
        
        elif np.linalg.norm(np.array(x) - np.array(self.goal)) < 1:
            # print(self.tree[tuple(x)][1] , self.tree[tuple(self.near_goal)][1])
            if self.tree[tuple(x)][1] < self.tree[tuple(self.near_goal)][1]:
                print(self.tree[tuple(x)][1] , self.tree[tuple(self.near_goal)][1])
                self.near_goal = x
                self.goal_flag = True


    def loop(self):
        x_sample = self.sampling()
        
        if self.is_in_collision(x_sample):
            # print("Sampled in Obstacle!")
            pass
        else:
            x_nearest = self.nearest(x_sample)
            x_reachable,new_cost = self.steer(x_nearest,x_sample)

            col_flag = False
            if x_nearest[0] > x_reachable[0] :
                xs = np.arange(x_reachable[0],x_nearest[0],0.01)
                m = (x_reachable[1]- x_nearest[1])/(x_reachable[0]- x_nearest[0])
                c = x_nearest[1] - m*x_nearest[0]
            else:
                xs = np.arange(x_nearest[0],x_reachable[0],0.01)
                m = (x_reachable[1]- x_nearest[1])/(x_reachable[0]- x_nearest[0])
                c = x_nearest[1] - m*x_nearest[0]
            for _,x in enumerate(xs):
                y = m*x + c
                col_flag = self.is_in_collision([x,y])

            if col_flag is True:
                pass
            else:
                self.connect(x_reachable,x_nearest,new_cost)
                self.check_goal(x_reachable)
                self.Optimal_circle(x_reachable,x_nearest)
                         



if __name__ == "__main__":
    start = [0,0]
    goal = [8,8]
    x_lim = [-10,10]
    y_lim = [-10,10]

    obs1 = [4,5,-4,9]
    obs2 = [-6,0,-5,-4]
    # obs2 = [4,10,-5,-4]
    obss = [obs1,obs2]
    # obss = [obs1]

    tree = {tuple(start):(tuple(start),0)}
    # tree = {tuple(start):(tuple(start),0), tuple(goal):(tuple(goal),0)}

    search_obj = rrt_star(start,goal,x_lim, y_lim,obss,tree)

    N = 1000

    for i in range(N):
        search_obj.loop()
        if i%100 ==0 and i!=0:
            fig,ax = plt.subplots()
            path =[]
            for k in search_obj.tree.keys():
                xs = [search_obj.tree[k][0][0],k[0]]
                ys = [search_obj.tree[k][0][1],k[1]]

                plt.plot(xs,ys,'go--',markersize = 5)

            plt.plot(start[0],start[1],'cp',markersize=2)
            plt.plot(goal[0],goal[1],'bp',markersize=2)
            # plt.plot([goal[0],search_obj.near_goal[0]],[goal[1],search_obj.near_goal[1]],'ro--',markersize=2)

            if search_obj.goal_flag is True:
                path = []
                pointer = tuple(search_obj.near_goal)
                final_cost = search_obj.tree[pointer][1]
                start_flag = False
                while start_flag is False:
                    path = path + [pointer]
                    pathx = search_obj.tree[pointer][0][0]
                    pathy = search_obj.tree[pointer][0][1]
                    plt.plot([pointer[0],pathx],[pointer[1],pathy],'ro--',markersize=2)
                    if list(pointer) == start:
                        start_flag = True
                    else:
                        pointer = search_obj.tree[pointer][0]

                print("The path is:",path)
                print("the cost of the optimal trajectory is: ",final_cost)

            patches = []
            rect1 = mpatches.Rectangle([4,-4],1,13,ec="none")
            patches.append(rect1)
            rect2 = mpatches.Rectangle([-6,-5],6,1,ec="none")
            patches.append(rect2)
            goal_circle = Circle(tuple(goal),0.5)
            patches.append(goal_circle)

            colors = np.linspace(0, 1, len(patches))
            collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.7)
            collection.set_array(np.array(colors))
            ax.add_collection(collection)

            patches2 = []
            rect1if = mpatches.Rectangle([4 -0.6 ,-4 - 0.6],1 + 1.2,13+1.2,ec="none")
            patches2.append(rect1if)
            rect2if = mpatches.Rectangle([-6 - 0.6,-5 - 0.6],6 + 1.2,1 + 1.2,ec="none")
            patches2.append(rect2if)
            collection2 = PatchCollection(patches2, cmap=plt.cm.hsv, alpha=0.3)
            collection2.set_array(np.array(colors))
            ax.add_collection(collection2)

            plt.axis('equal')
            plt.tight_layout()
            plt.grid()
            plt.show()
            # plt.close('all')
            print('!')
        