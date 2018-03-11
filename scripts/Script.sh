#!/bin/bash
while true; 
   do 
     ./blink_both.py ;  sleep 1; 
     ./blink_left.py;  sleep 1;
     ./blink_right.py;  sleep 1;
     ./surprised.py;   sleep 1;
     ./glare.py ; sleep 1; 
     ./surprised.py ; sleep 1; 
     ./roll_eye_sync.py ;  sleep 1;  
     ./surprised.py ; sleep 1; 
     ./glare.py ; sleep 1; 
     ./surprised.py ; sleep 1; 
     ./annoyed.py ; sleep 1; 
     ./surprised.py ;sleep 1; 
   done
