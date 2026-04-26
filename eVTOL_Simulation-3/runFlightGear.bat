@echo off
cd /d "C:\Program Files\FlightGear 2024.1\bin"

fgfs.exe ^
  --aircraft=c172p ^
  --fdm=null ^
  --native-fdm=socket,in,100,,5502,udp ^
  --disable-freeze ^
  --in-air ^
  --altitude=1000 ^
  --lat=34.05105 ^
  --lon=-118.25439 ^
  --heading=0