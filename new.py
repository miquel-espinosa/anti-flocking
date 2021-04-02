from matplotlib.pyplot import hist
import pygame  
import sys  
import math  
from pygame.locals import *
from swarm import *
from functions import *
from constants import *

pygame.init()

#---Step 3 - define color---  
WHITE = (255, 255, 255)  
BLACK = (0, 0, 0)  
GREEN = (0, 255, 0)  
RED = (255, 0, 0)  
BLUE = (0, 0, 255)  
YELLOW = (255, 255, 0)

#---Step 4: define window size, title name, font setting and create clock---  
size = width, height = 1500, 900 
screen = pygame.display.set_mode(size)  
# pygame.display.set_caption("Simulation")  
#The initial definition of font, note that English is like this  
myfont=pygame.font.Font(None,60)  
#If it is Chinese, the font hwfs = Chinese imitation song font, put it in the root directory  
#myfont=pygame.font.Font('hwfs.ttf',60)  
#Create a clock object (you can control the frequency of the game cycle) --- you must---  
clock = pygame.time.Clock()


# obstacles
obs1 = Obstacle(ld=[700,290],ru=[640,790])
obs2 = Obstacle(ld=[210,600],ru=[510,800])
obstacles = [obs1, obs2]


history_x = [[] for i in range(NUM_UAVS)]
history_y = [[] for i in range(NUM_UAVS)]
swarm = Swarm(NUM_UAVS, obstacles)


while True:  
    #---6-1 --- first---  
    #---pygame's game cycle is essential -- personal advice and likes---  
    for event in pygame.event.get():  
        if event.type == QUIT:  
            sys.exit()  
    #The background color is black  
    screen.fill(BLACK)  
    #Display text settings on screen  
    textImage=myfont.render("Swarm simulation",True,GREEN)  
    #Display at screen coordinates 100 and 100 
    screen.blit(textImage,(100,100)) 
    #---6-2 --- draw sun's size, position and color setting, and the size of 60 is more appropriate---  
  
    for i in obstacles:
        pygame.draw.rect(screen, YELLOW, (i.ld,i.ru))  

    for agent in range(NUM_UAVS):

        swarm.vel_dec[agent]=swarm.vel_dec[agent]+random.randint(2,3)
        history_x[agent].append(swarm.vel_dec[agent])
        print(history_x)

        if len(history_x[agent]) > 400:  
            history_x[agent].pop(0)  
        for i in range(len(history_x[agent])):  
            #The trace line is green = green = 0255,0  
            pygame.draw.circle(screen, GREEN ,history_x[agent][i].astype(int), 1, 0) 
    
    #Refresh  
    pygame.display.flip()  
    #The larger the value, the faster the refresh, the faster the ball moves  
    clock.tick(40) 

