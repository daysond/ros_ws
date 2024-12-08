sudo -S pkill -f camera_node
sudo -S pkill -f usb_cam_node
sudo modprobe -r uvcvideo 
sudo modprobe uvcvideo 