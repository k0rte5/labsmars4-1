
#import
import  numpy as np
import pygame, sys
import time


# euclidian dist check
def dist(p1,p2):
    dx=p2[0]-p1[0]
    dy=p2[1]-p1[1]
    return np.sqrt(dx*dx+dy*dy)


#obj model

class Obj:
    def __init__(self,x,y,color) -> None:
        self.x=x
        self.y=y
        self.color=color
        self.reservedRobot=None
        self.finished=False
    def getPos(self):
        return(self.x, self.y)
    def draw(self,screen):
        r=10
        pygame.draw.ellipse(screen, self.color,[self.x-r,self.y-r,2*r,2*r],2)


# robot model

class Robot:
    def __init__(self, x, y):
        self.x=x
        self.y=y
        self.attachedObj=None
        self.target=None

    def getPos(self):
        return (self.x, self.y)

    def draw(self, screen):
        r=20
        pygame.draw.ellipse(screen, (255, 0, 0),[self.x-r, self.y-r, 2*r, 2*r], 2)

    def simulate(self):
        if self.target !=None:
            p1=self.target
            p2=(self.x, self.y)
            v=np.array(p1)-np.array(p2)
            d=dist(p1,p2)
            if d>0:
                v=v/d
            self.x+=v[0]*5
            self.y+=v[1]*5
        if self.attachedObj !=None:
            self.attachedObj.x=self.x
            self.attachedObj.y=self.y
    
    def findNearestObj(self,objs,threshhold=100500):
        res=None
        D=100500
        for o in objs:
            if o.reservedRobot !=None and o.reservedRobot !=self:
                continue
            if o.finished:
                continue
            dNew=dist(o.getPos(),self.getPos())
            if dNew<D:
                D=dNew
                res=o
        if D>threshhold:
            res=None
        return res
    
    def take(self,obj):
        if obj !=None:
            self.attachedObj=obj


# task distribution

def distributeTasks(robots, objs,goal):
    #distributing to every robot
    for r in robots:
        #finish check
        if r.attachedObj !=None and dist(r.getPos(), goal.getPos()) <20:
            r.attachedObj.finished=True
            r.attachedObj=None
            r.target=None
        else:
            #taking possibility check
            if r.attachedObj==None:
                obj=r.findNearestObj(objs)
                if obj !=None and dist(r.getPos(), obj.getPos()) < 20:
                    r.take(obj)
                    r.target = goal.getPos()
                    return
        #checking near objs
        if r.target==None and r.attachedObj==None:
            obj=r.findNearestObj(objs)
            if obj==None:
                continue
            if obj.reservedRobot !=None:
                continue
            r.target=obj.getPos()
            obj.reservedRobot=r


#finished alg criteria

def checkMission(robots, objs, goal):
    for r in robots:
        if dist(r.getPos(), goal.getPos())>20:
            return False
    for o in objs:
        if o.reservedRobot==None:
            return False
    return True
        


#obj generation sequence

def generateObjs(N):
    res=[]
    for i in range(N):
        o=Obj(np.random.randint(50, 1500-50),np.random.randint(50, 1300-50),(0,255,0))
        res.append(o)
    return res


#main
def main():

    #canvas parameters
    WIDTH=1500
    HEIGHT=1300

    pygame.init()
    screen=pygame.display.set_mode((WIDTH,HEIGHT))

    start=time.time()

    robots= [
        Robot(150,150),
        Robot(150,250),
        Robot(150,350),
        Robot(150,450),
        Robot(150,550),
        Robot(150,650)
            ]
    objs=generateObjs(5)

    goal=Obj(750,450,(0,0,255))


    while True:
        #interface
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                pygame.quit()
                sys.exit()
        #simulation
        if checkMission(robots,objs,goal):
                break

        screen.fill((255,255,255))

        distributeTasks(robots,objs,goal)

        for r in robots:
            r.simulate()
            r.draw(screen)

        for o in objs:
            o.draw(screen)

        goal.draw(screen)

        pygame.display.update()
        pygame.time.delay(50)

        # group task completion calculation
        end=time.time()
        print(end-start)
            


main()
NN=[1,2,3,4,5,6]
MM=[5,10,15]


