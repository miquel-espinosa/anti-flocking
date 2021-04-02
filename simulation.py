import pygame  
import sys  
import math  
from pygame.locals import *

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
pygame.display.set_caption("sunlight-earth-Moon-Diagram of Venus, etc")  
#The initial definition of font, note that English is like this  
myfont=pygame.font.Font(None,60)  
#If it is Chinese, the font hwfs = Chinese imitation song font, put it in the root directory  
#myfont=pygame.font.Font('hwfs.ttf',60)  
#Create a clock object (you can control the frequency of the game cycle) --- you must---  
clock = pygame.time.Clock()

#---Step 5 - initialize related definitions - specific to the definition of each game---  
#Define three empty lists  
'''  
pos_v=[]  
pos_e = []  
pos_mm = []  
'''  
#Same as above  
pos_v=pos_e=pos_mm=[]  
#The angle at which other planets, such as Venus, the earth, and the moon, revolve  
roll_v = roll_e = roll_m = 0  
roll_3=roll_4=roll_5=roll_6=roll_7=roll_8=0  
  
#Position of the sun - relatively fixed - Center  
#Knowledge point: size is a line assignment method, equivalent to a tuple of size=(width, height)  
#size[0]=width,size[1]=height  
position = size[0]//2, size[1]//2


#---Step 6 - game cycle---  
while True:  
    #---6-1 --- first---  
    #---pygame's game cycle is essential -- personal advice and likes---  
    for event in pygame.event.get():  
        if event.type == QUIT:  
            sys.exit()  
    #The background color is black  
    screen.fill(BLACK)  
    #Display text settings on screen  
    textImage=myfont.render("Sun=yellow,Earth=blue,Moon=green,Venus=red",True,GREEN)  
    #Display at screen coordinates 100 and 100  
    screen.blit(textImage,(100,100))  
  
    #---6-2 --- draw sun's size, position and color setting, and the size of 60 is more appropriate---  
    pygame.draw.circle(screen, YELLOW, position, 60, 0)  
  
    #---6-3 --- earth = the Earth---  
    roll_e += 0.01# Suppose the earth rotates 0.01 PI per frame  
    pos_e_x = int(size[0]//2 + size[1]//6*math.sin(roll_e))  
    pos_e_y = int(size[1]//2 + size[1]//6*math.cos(roll_e))  
    pygame.draw.circle(screen, BLUE, (pos_e_x, pos_e_y), 15, 0)  
      
    #---The trajectory of the earth---  
    pos_e.append((pos_e_x, pos_e_y))  
    if len(pos_e) > 255:  
        pos_e.pop(0)  
    for i in range(len(pos_e)):  
        #The trace line is green = green = 0255,0  
        pygame.draw.circle(screen, GREEN, pos_e[i], 1, 0)  
  
    #---6-4 --- moon = the Moon---  
    roll_m += 0.1# Suppose the moon rotates 0.1pi per frame  
    pos_m_x = int(pos_e_x + size[1]//20*math.sin(roll_m))  
    pos_m_y = int(pos_e_y + size[1]//20*math.cos(roll_m))  
    pygame.draw.circle(screen, GREEN, (pos_m_x, pos_m_y), 5, 0)  
  
    #---The trajectory of the moon---  
    pos_mm.append((pos_m_x, pos_m_y))  
    if len(pos_mm) > 255:  
        pos_mm.pop(0)  
    for i in range(len(pos_mm)):  
        #The trace line is green = green = 0255,0  
        pygame.draw.circle(screen, GREEN ,pos_mm[i], 1, 0)  
  
    #---6-5 --- Venus = the Venus---  
    roll_v += 0.015# Suppose Venus rotates 0.015 PI per frame      
    pos_v_x = int(size[0]//2 + size[1]//3*math.sin(roll_v))  
    pos_v_y = int(size[1]//2 + size[1]//3*math.cos(roll_v))  
    pygame.draw.circle(screen, RED, (pos_v_x, pos_v_y), 20, 0)  
      
    #---The trajectory of Venus - not if it's necessary---  
    pos_v.append((pos_v_x, pos_v_y))  
    if len(pos_v) > 255:  
        pos_v.pop(0)  
    for i in range(len(pos_v)):  
        #The trace line is green = green = 0255,0  
        pygame.draw.circle(screen, (0,255,0), pos_v[i], 1, 0)  
      
    #---6-6 --- other planets --- the disadvantage is not elliptical orbit---  
    # 3  
    roll_3 += 0.03# Suppose Venus rotates 0.03pi per frame      
    pos_3_x = int(size[0]//2 + size[1]//3.5*math.sin(roll_3))  
    pos_3_y = int(size[1]//2 + size[1]//3.5*math.cos(roll_3))  
    pygame.draw.circle(screen, WHITE,(pos_3_x, pos_3_y), 20, 0)  
  
    # 4  
    roll_4 += 0.04# Suppose Venus rotates 0.04 PI per frame      
    pos_4_x = int(size[0]//2 + size[1]//4*math.sin(roll_4))  
    pos_4_y = int(size[1]//2 + size[1]//4*math.cos(roll_4))  
    pygame.draw.circle(screen, WHITE,(pos_4_x, pos_4_y), 20, 0)  
  
    # 5  
    roll_5 += 0.05# Suppose Venus rotates 0.05 PI per frame      
    pos_5_x = int(size[0]//2 + size[1]//5*math.sin(roll_5))  
    pos_5_y = int(size[1]//2 + size[1]//5*math.cos(roll_5))  
    pygame.draw.circle(screen, WHITE, (pos_5_x, pos_5_y), 20, 0)  
    # 6  
    roll_6 += 0.06# Suppose Venus rotates 0.06 PI per frame      
    pos_6_x = int(size[0]//2 + size[1]//2.5*math.sin(roll_6))  
    pos_6_y = int(size[1]//2 + size[1]//2.5*math.cos(roll_6))  
    pygame.draw.circle(screen, WHITE,(pos_6_x, pos_6_y), 20, 0)  
  
    # 7  
    roll_7 += 0.07# Suppose Venus rotates 0.07 PI per frame      
    pos_7_x = int(size[0]//2 + size[1]//4.5*math.sin(roll_7))  
    pos_7_y = int(size[1]//2 + size[1]//4.5*math.cos(roll_7))  
    pygame.draw.circle(screen, WHITE, (pos_7_x, pos_7_y), 20, 0)  
  
    # 8  
    roll_8 += 0.08# Suppose Venus rotates 0.08pi per frame      
    pos_8_x = int(size[0]//2 + size[1]//5.5*math.sin(roll_8))  
    pos_8_y = int(size[1]//2 + size[1]//5.5*math.cos(roll_8))  
    pygame.draw.circle(screen, WHITE, (pos_8_x, pos_8_y), 20, 0)  
  
    #Refresh  
    pygame.display.flip()  
    #The larger the value, the faster the refresh, the faster the ball moves  
    clock.tick(40) 