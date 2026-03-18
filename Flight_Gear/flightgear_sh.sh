#!/bin/bash

# Navigate to the FlightGear directory
cd "" # Updated path

# Set the FG_ROOT environment variable
export FG_ROOT="" # Updated path

# Execute the FlightGear command
# Note: You will likely need to adjust the path to the fgfs executable as well.
# It might be inside the FlightGear.app package. Youan find the executable path by right-clicking
# on FlightGear.app, selecting 'Show Package Contents', and then browsing to the executable.
./bin/fgfs --fdm=null --native-fdm=socket,in,100,,5502,udp --fog-fastest --disable-clouds --start-date-sys=2016:04:05:20:00:00 --timeofday=noon --disable-sound --in-air --enable-freeze --altitude=415 --lat=42.3656 --lon=-71.0096 --heading=270 --offset-distance=0 --offset-azimuth=0 --prop:/input/joysticks/js[0]=0
 
# Other start-up locations (comments)
# Los Alamitos Between Hangars
#  --altitude=10 --lat=33.7930408 --lon=-118.05527391667175 --heading=180 
# Tier One Engineering Parking Lot
#  --altitude=23 --lat=33.63535467110566 --lon=-117.94078089140169 --heading=90 
# Long Beach Airport
#  --altitude=0 --lat=33.8209454867282 --lon=-118.16009419107667 --heading=90 

# List of arguments used above (comments)
# --aircraft=ask13
# --fdm=null 
# --native-fdm=socket,in,100,,5502,udp 
# --fog-fastest 
# --disable-clouds 
# --start-date-sys=2016:04:05:20:00:00 
# --timeofday=noon 
# --disable-sound 
# --in-air 
# --enable-freeze 
# --altitude=415 
# --lat=42.3656 
# --lon=-71.0096 
# --heading=270 
# --offset-distance=0 
# --offset-azimuth=0 
# --prop:/input/joysticks/js[0]=0

# List of Arguments we have used in the past (comments)
# --aircraft=ASK13 
# --enable-terrasync 
# --aircraft-dir="/Users/joshbromby/Desktop/FlightGear.app/data/Aircraft/ASK13" # Updated path

# Reference for all Arguments:
# http://wiki.flightgear.org/Command_line_options
