import socket
import time
import os, sys

play_dest_received_first_time = 1
play_dest_reached_first_time =1
import pygame
mp3_path ='/home/ubuntu/catkin_ws/src/demos/fiducial_follow/nodes/'

def play_dest_received():
    global play_dest_reached_first_time
    global play_dest_received_first_time
    print("++++++++++++++++++++++++++++++++++")
    print("try to play play_dest_received()")
    print("++++++++++++++++++++++++++++++++++")
    if play_dest_received_first_time == 0:
        return
    import pygame
    #os.system('audacious ' +mp3_path+"Received-signals.wav"+    '&')
    play_dest_received_first_time = 0
    play_dest_reached_first_time   = 1
    #return 
    pygame.mixer.init()
    pygame.mixer.music.load(mp3_path+"Received-signals.wav")
    pygame.mixer.music.set_volume(1.0)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy() == True:
            pass
        
    pygame.mixer.init()
    pygame.mixer.music.load(mp3_path+"received.mp3")
    print(mp3_path+"received.mp3")
    pygame.mixer.music.play()
    pygame.time.delay(33)
    try: 
        pygame.mixer.init()
        sound = pygame.mixer.Sound(mp3_path+"Received-signals.wav")
        playing = sound.play()
        while playing.get_busy():
            pygame.time.delay(2)
    except Exception as e:
        print(e)
        print("Errorrrrrrrrrrrrrrrrrr")

play_dest_reached_first_time  = 1
play_dest_received()

while 1:
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('',12345))
    m=s.recv(1024)
    msg1=m.decode('utf-8')
    print('Received:' + msg1)
    if msg1[0] == '0':
        destination = msg1[1:]
        print(destination)

    if msg1 == destination:
        print("Destination is reached!")
        break
    
    else:
        print("Destination is not reached!")
        time.sleep(1)
        #break
 
    # disconnect the client
    s.close()
