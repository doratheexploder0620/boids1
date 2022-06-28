from p5 import*
import random as ra
import numpy as np
class Boid():

    def __init__(self, x, y,radiusvel=50 ,radiuscohe=50,rradiusrep=10,width=640, height=360, radiuskura=10):
    
        self.position = Vector(x, y)
        angle=(ra.uniform(0,2*3.14))
        self.velocity=Vector(5*cos(angle),5*sin(angle))
        self.acceleration=Vector(0,0)
        self.rv=radiusvel
        self.rc=radiuscohe
        self.rr=rradiusrep
        self.maxvel=50
        self.maxacc=10
        self.width=width
        self.rku=radiuskura
        self.height=height
        self.bright=ra.uniform(0,1)
        self.frequency=1
        self.phase=ra.uniform(0,2*3.14)

    def borders(self):
        if self.position.x>self.width:
            self.position.x=0
        if self.position.x<0:
            self.position.x=self.width
        if self.position.y>self.width:
            self.position.y=0
        if self.position.y<0:
            self.position.y=self.height

            
        

    def newphase_using_kuramoto(self, boids):
        b=0
        count=0
        for boid in boids:
            d=(self.position-boid.position).magnitude
            
            if d<self.rku and d!=0:
                b+=np.sin(boid.phase-self.phase)
                count=count+1
        if count!=0:
            b=b/count
            return(b)
        else :
            return(b)    


        


    def alignacc(self,boidset):
        v=Vector(0,0)
        count=0
        steer= Vector(0,0)
        for boid in boidset:
            if (self.position- boid.position).magnitude<self.rv:
                v+=boid.velocity
                count+=1
        if count>0 and (v-self.velocity).magnitude!=0:    
            v=(v/count)
            direction= ((v-self.velocity)/(v-self.velocity).magnitude)  
             
            steer+=(direction*(self.maxacc))
            
        return steer     

    def seperate(self,boids):
        steer=Vector(0,0)
        desveldir=Vector(0,0)
        for boid in boids :
            d=(self.position - boid.position ).magnitude 
            if d<self.rr and d!=0 :
                direction=(self.position- boid.position).normalize()
                
                desveldir+= direction/d
        if desveldir!=Vector(0,0):
            desveldir=desveldir.normalize()
            desveldir*=self.maxvel
            steer+=((desveldir - self.velocity).normalize())*self.maxacc
        
            return steer         
        else:
            return steer          

        

    def cohesionacc(self,boids):
        agg= Vector(0,0)
        count=0
        steer=Vector(0,0)
        for boid in boids :
            d=(self.position-boid.position).magnitude 
            if d<self.rc:
                agg+=boid.position
                count+=1

        if count!=0 and (agg-self.position).magnitude!=0:
            avgpos=agg/count
            dessteerdir= (avgpos- self.position).normalize()
            dessteerdir*=self.maxvel
            steer+=(dessteerdir-self.velocity).normalize()*self.maxacc

            
            
            return steer 
        else:
            return steer
    

    def flockeffect(self,boidset,dt=.1,paramalign=1,paramseperate=4,paramcohes=1,):
        a=self.alignacc(boidset)*paramalign
        b=self.cohesionacc(boidset)*paramcohes

        c=self.seperate(boidset)*paramseperate
        d=self.newphase_using_kuramoto(boidset)
        self.frequency=1+d
        self.acceleration+=a
        self.acceleration+=b
        self.acceleration+=c
        
        
    def update(self,dt=.1):
        self.velocity+=self.acceleration*dt
        
        self.velocity.limit(self.maxvel)
        self.phase=self.phase+self.frequency*dt
        
        
        self.position+=self.velocity*dt
        self.acceleration=Vector(0,0)
    def run(self, boids,dt=.15):
        self.flockeffect(boids,dt)
        self.borders()
        self.update(dt)

            
class Flock:
    def __init__(self, count=150, width=300, height=300):
        self.width = width
        self.height = height
        self.boids = []
        for i in range(count):
            boid = Boid(ra.uniform(0,width), ra.uniform(0,height))
            boid.width = width
            boid.height = height
            self.boids.append(boid)

    def run(self):
        for boid in self.boids:
            
            boid.run(self.boids)

    def cohesion(self, boids):
        P = np.zeros((len(boids),2))
        for i, boid in enumerate(self.boids):
            P[i] = boid.cohesionacc(self.boids)
        return P

        

            
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


n=30
flock = Flock(n)
P = np.zeros((n,2))
j=np.zeros((n,1))


def make_update(k):
    x=np.zeros((n*k))
    y=np.zeros((n*k))
    z=np.zeros((n*k))
    for i in range (k):

        flock.run()
        for j,boid in enumerate(flock.boids):
            x[i*n+(j)]=boid.position.x
            y[i*n+(j)] = boid.position.y
            z[i*n+(j)] = boid.phase%(2*3.14)
    ati=np.vstack((x,y,z))   
    return ati

def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        line.set_data(data[0:2, 30*num-30:30*num])
        line.set_3d_properties(data[2, 30*num-30:30*num])
    return lines
# Attach 3D axis to the figure
fig = plt.figure()
ax =  fig.add_subplot(111, projection="3d",)
fig.add_axes(ax)

k = 500
data = [make_update(k)]

lines = [ax.plot(data[0][0, 0:n], data[0][1, 0:n], data[0][2, 0:n], 'o')[0]]

# Setthe axes properties
ax.set_xlim3d([0.0, 300])
ax.set_xlabel('X')

ax.set_ylim3d([-1.0, 300])
ax.set_ylabel('Y')

ax.set_zlim3d([-6.0, 6.0])
ax.set_zlabel('Z')

ax.set_title('3D Test')

# Creating the Animation object
ani = animation.FuncAnimation(fig, update_lines, k, fargs=(data, lines),
                              interval=50, blit=False)
plt.show()            
    
        
    

