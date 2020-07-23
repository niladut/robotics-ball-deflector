# /bin/bash

echo "Converting Simulation View to Video"
ffmpeg -framerate 30 -i ./images/sim_view/img-%05d.png -c:v libx264 -pix_fmt yuv420p ./images/sim_view.mp4

echo "Converting Configuration View to Video"
ffmpeg -framerate 30 -i ./images/config_view/img-%05d.png -c:v libx264 -pix_fmt yuv420p ./images/config_view.mp4


echo "Converting Camera View to Video"
ffmpeg -framerate 30 -i ./images/cam_rgb_view/img-%05d.png -c:v libx264 -pix_fmt yuv420p ./images/cam_rgb_view.mp4
