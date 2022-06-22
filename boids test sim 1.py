

from p5 import*
import random as ra 
class Boid():

    def __init__(self, x, y,radiusvel= 50,radiuscohe=50,rradiusrep=10,width=640, height=360):
        self.position = Vector(x, y)
        angle=(ra.uniform(0,2*math.pi))
        self.velocity=Vector(cos(angle),sin(angle))
        self.acceleration=Vector(0,0)
        self.rv=radiusvel
        self.rc=radiuscohe
        self.rr=rradiusrep
        self.maxvel=50
        self.maxacc=10
        self.width=width
        self.height=height

    def borders(self):
        if self.position.x>self.width:
            self.position.x=0
        if self.position.x<0:
            self.position.x=self.width
        if self.position.y>self.width:
            self.position.y=0
        if self.position.y<0:
            self.position.y=self.height            
        




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
            desveldir= (agg- self.position).normalize()
            desvel=desveldir*self.maxvel
            steerdir=(desvel-self.velocity).normalize()
            steer+= steerdir*self.maxacc
            return steer 
        else:
            return steer
    

    def flockeffect(self,boidset,dt=.1,paramalign=5,paramseperate=2,paramcohes=5,):
        a=self.alignacc(boidset)*paramalign
        b=self.cohesionacc(boidset)*paramcohes

        c=self.seperate(boidset)*paramseperate
        self.acceleration+=a
        self.acceleration+=b
        self.acceleration+=c
        
        
    def update(self,dt=.1):
        self.velocity+=self.acceleration*dt
        
        self.velocity.limit(self.maxvel)
        
        
        self.position+=self.velocity*dt
        self.acceleration=Vector(0,0)
    def run(self, boids,dt=.1):
        self.flockeffect(boids,dt)
        self.borders()
        self.update(dt)

            
class Flock:
    def __init__(self, count=150, width=300, height=300):
        self.width = width
        self.height = height
        self.boids = []
        for i in range(count):
            boid = Boid(ra.uniform(0,width/2), .5*ra.uniform(0,height/2))
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

n=40
flock = Flock(n)
P = np.zeros((n,2))

def update(*args):
    flock.run()
    for i,boid in enumerate(flock.boids):
        P[i,1]=boid.position.x
        P[i,0] = boid.position.y
    scatter.set_offsets(P)

fig = plt.figure(figsize=(8, 5))
ax = fig.add_axes([0.0, 0.0, 1.0, 1.0], frameon=True)
scatter = ax.scatter(P[:,0], P[:,1],
                     s=30, facecolor="red", edgecolor="None", alpha=0.5)

animation = FuncAnimation(fig, update, interval=10)
ax.set_xlim(0,640)
ax.set_ylim(0,360)
ax.set_xticks([])
ax.set_yticks([])
plt.show()


















