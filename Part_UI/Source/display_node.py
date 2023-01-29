#!/usr/bin/python3
# touchv
# Texy 5/2/2014

import pygame, sys, os, time
from pygame.locals import *
from math import trunc
from random import *
from pygame.constants import K_DOWN

dt = 0
running = True
scene_num = 0
# run the game loop
#pygame init
pygame.init()

# set up the window
screen = pygame.display.set_mode((1280, 800))
pygame.display.set_caption('crashlab')

# set up the colors
BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
RED   = (255,   0,   0)
GREEN = (  0, 255,   0)
BLUE  = (  0,   0, 255)
CYAN  = (  0, 255, 255)
MAGENTA=(255,   0, 255)
YELLOW =(255, 255,   0)

# load scene
scene0 = pygame.image.load("./source/scene0.png")
scene1 = pygame.image.load("./source/scene1.png")
scene2 = pygame.image.load("./source/scene2.png")
scene3 = pygame.image.load("./source/scene3.png")
scene4 = pygame.image.load("./source/scene4.png")
scene5 = pygame.image.load("./source/scene5.png")
scene6 = pygame.image.load("./source/scene6.png")
scene7 = pygame.image.load("./source/scene7.png")
scene8 = pygame.image.load("./source/scene8.png")
scene9 = pygame.image.load("./source/scene9.png")
scene10 = pygame.image.load("./source/scene10.png")

next = pygame.image.load("./source/next.png")

kirby1 = pygame.image.load("./source/kirby1.jpg")
kirby2 = pygame.image.load("./source/kirby2.jpg")

coffee1 = pygame.image.load("./source/coffee_01.jpg")
coffee2 = pygame.image.load("./source/coffee_02.jpg")
coffee3 = pygame.image.load("./source/coffee_03.jpg")
coffee4 = pygame.image.load("./source/coffee_04.jpg")
coffee5 = pygame.image.load("./source/coffee_05.jpg")
coffee6 = pygame.image.load("./source/coffee_06.jpg")

present = pygame.image.load("./source/present.png")

# load music
pygame.mixer.init()
pygame.mixer.music.load("./source/kirby_ost.wav")
pygame.mixer.music.play(-1)

# Display some text
font = pygame.font.Font(None, 100)
font2 = pygame.font.Font(None, 50)

#text = pygame.transform.rotate(text,270) 
clk0 = scene0.get_rect(centerx=screen.get_width()/2,centery=screen.get_height()/2)
clk1 = scene1.get_rect(centerx=scene1.get_width()/2,centery=scene1.get_height()/2)
clk2 = scene2.get_rect(centerx=scene2.get_width()/2,centery=scene2.get_height()/2)
clk3 = scene3.get_rect(centerx=scene3.get_width()/2,centery=scene3.get_height()/2)
clk4 = scene4.get_rect(centerx=scene4.get_width()/2,centery=scene4.get_height()/2)
clk5 = scene5.get_rect(centerx=scene5.get_width()/2,centery=scene5.get_height()/2)

clk7 = scene7.get_rect(centerx=scene7.get_width()/2,centery=scene7.get_height()/2)
clk8 = scene8.get_rect(centerx=scene8.get_width()/2,centery=scene8.get_height()/2)
clk9 = scene9.get_rect(centerx=scene9.get_width()/2,centery=scene9.get_height()/2)
clk10 = scene10.get_rect(centerx=scene10.get_width()/2,centery=scene10.get_height()/2)

clk_kirby1 = kirby1.get_rect(centerx=scene4.get_width()*(350/1280),centery=scene4.get_height()*(520/800))
clk_kirby2 = kirby2.get_rect(centerx=scene4.get_width()*(950/1280),centery=scene4.get_height()*(520/800))

clk_coffee1 = coffee1.get_rect(centerx=scene5.get_width()*(538/1280),centery=scene5.get_height()*(200/800))
clk_coffee2 = coffee2.get_rect(centerx=scene5.get_width()*(823/1280),centery=scene5.get_height()*(200/800))
clk_coffee3 = coffee3.get_rect(centerx=scene5.get_width()*(1108/1280),centery=scene5.get_height()*(200/800))
clk_coffee4 = coffee4.get_rect(centerx=scene5.get_width()*(538/1280),centery=scene5.get_height()*(600/800))
clk_coffee5 = coffee5.get_rect(centerx=scene5.get_width()*(823/1280),centery=scene5.get_height()*(600/800))
clk_coffee6 = coffee6.get_rect(centerx=scene5.get_width()*(1108/1280),centery=scene5.get_height()*(600/800))

clk_present = present.get_rect(centerx=scene6.get_width()*(640/1280),centery=scene6.get_height()*(330/800))
clk_next = next.get_rect(centerx=scene7.get_width()*(1100/1280),centery=scene7.get_height()*(740/800))


while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == KEYDOWN and event.key == K_ESCAPE:
            running = False

        if scene_num == 0:
            screen.blit(scene0, (0,0))
            pygame.display.flip()
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk0.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 1:
            screen.blit(scene1, (0,0))
            pygame.display.flip()
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk1.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 2:
            pygame.display.flip()
            screen.blit(scene2, (0,0))
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk2.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 3:
            pygame.display.flip()
            screen.blit(scene3, (0,0))
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk3.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 4:
            pygame.display.flip()
            screen.blit(scene4, (0,0))
            screen.blit(kirby1, clk_kirby1)
            screen.blit(kirby2, clk_kirby2)
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk_kirby1.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1
                elif clk_kirby2.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = 0
                         
        elif scene_num == 5:
            pygame.display.flip()
            screen.blit(scene5, (0,0))
            screen.blit(coffee1, clk_coffee1)
            screen.blit(coffee2, clk_coffee2)
            screen.blit(coffee3, clk_coffee3)
            screen.blit(coffee4, clk_coffee4)
            screen.blit(coffee5, clk_coffee5)
            screen.blit(coffee6, clk_coffee6)
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if (clk_coffee1.collidepoint(pygame.mouse.get_pos()) or clk_coffee2.collidepoint(pygame.mouse.get_pos()) or 
                clk_coffee3.collidepoint(pygame.mouse.get_pos()) or clk_coffee4.collidepoint(pygame.mouse.get_pos()) or
                clk_coffee5.collidepoint(pygame.mouse.get_pos()) or clk_coffee6.collidepoint(pygame.mouse.get_pos())):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 6:
            pygame.display.flip()
            screen.blit(scene6, (0,0))
            screen.blit(present, clk_present)
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk_present.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 7:
            pygame.display.flip()
            screen.blit(scene7, (0,0))
            screen.blit(next, clk_next)
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk_next.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 8:
            pygame.display.flip()
            screen.blit(scene8, (0,0))
            screen.blit(next, clk_next)
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk_next.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1

        elif scene_num == 9:
            pygame.display.flip()
            screen.blit(scene9, (0,0))
            screen.blit(next, clk_next)
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk_next.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = scene_num + 1
        
        elif scene_num == 10:
            pygame.display.flip()
            screen.blit(scene10, (0,0))
            if event.type == pygame.MOUSEBUTTONDOWN:
                print("Pos: %sx%s\n" % pygame.mouse.get_pos())
                if clk10.collidepoint(pygame.mouse.get_pos()):
                    dt = 0
                    scene_num = 0

pygame.quit()
sys.exit()
