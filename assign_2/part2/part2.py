# pyODE example 2: Connecting bodies with joints

import pygame
from pygame.locals import *
import ode
from ipdb import set_trace

def coord(x,y):
    "Convert world coordinates to pixel coordinates."
    return int(320 + 5*x), int(400 - 5*y)


# Initialize pygame
pygame.init()

# Open a display
srf = pygame.display.set_mode((640,480))

# Create a world object
world = ode.World()
world.setGravity((0,-9.81,0))

# Create a space object
space = ode.Space()

# # floor 
# floor = ode.GeomPlane(space, (0,1,0), 0)

# Create two bodies
body0 = ode.Body(world)
M = ode.Mass()
M.setCylinderTotal(1, 1,0.005,1.0)
body0.setMass(M)
body0.setPosition((-10,30,0))

body1 = ode.Body(world)
M = ode.Mass()
M.setBoxTotal(1, 1.0,1.0,1.0)
body1.setMass(M)
body1.setPosition((-10,30,0))

print "cart mass:"
print body1.getMass()
print "*****\n"

body2 = ode.Body(world)
M = ode.Mass()
M.setSphereTotal(0.1, 0.005)
body2.setMass(M)
body2.setPosition((25,40,0))

print "pole mass"
print body2.getMass()
print "*****\n"

# Connect body1 with the static environment
j0 = ode.SliderJoint(world)
j0.attach( body0, ode.environment)
j0.setAxis((1,0,0))
# j0.addForce(200)          ## to increase the motion of the cart

j1 = ode.BallJoint(world)
j1.attach( body0, body1)
j1.setAnchor( (-10,30,0) )

j2 = ode.HingeJoint(world)
# j2 = ode.BallJoint(world)
j2.attach(body2, body1)
j2.setAnchor( (-10,50,0) )
j2.setAxis((1,0,0))


# Simulation loop...

fps = 100
dt = 1.0/fps
loopFlag = True
clk = pygame.time.Clock()

while loopFlag:
    events = pygame.event.get()
    for e in events:
        if e.type==QUIT:
            loopFlag=False
        if e.type==KEYDOWN:
            loopFlag=False

    # Clear the screen
    srf.fill((255,255,255))

    # Draw the two bodies
    x0,y0,z0 = body0.getPosition()
    x1,y1,z1 = body1.getPosition()
    x2,y2,z2 = body2.getPosition()
    
    tmp0 = coord(x0,y0)
    tmp1 = coord(x1,y1)
    tmp2 = coord(x2,y2)
    pygame.draw.line(srf,(0,0,0),(tmp0[0] -500, tmp0[1]+10), (tmp0[0] +500, tmp0[1]+10), 2)
    pygame.draw.rect(srf, (55,0,200), ((tmp1[0] - 50, tmp1[1], 100,20)) ,0)
    pygame.draw.line(srf, (55,0,200), coord(x2,y2), (tmp1[0],tmp1[1]), 2)
    pygame.draw.circle(srf, (55,0,200), coord(x2,y2), 10, 0)
    
    pygame.display.update()

    # Next simulation step
    # world.quickStep(dt)       # for quick but bad simulation
    world.step(dt)
    
    # Try to keep the specified framerate    
    clk.tick(fps)