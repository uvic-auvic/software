# Installs the VNC server on the odroid

# Install the VNC server
sudo apt-get update
sudo apt-get install -y xfce4 xfce4-goodies tightvncserver

# Start the VNC server to create the config directories then kill it
vncserver
vncserver -kill :1

# Create the new startup exec
mv ~/.vnc/xstartup ~/.vnc/xstartup.bak
cp xstartup ~/.vnc/

# Move the startup script - Needs root access
sudo cp vncserver /etc/init.d/vncserver
sudo chmod +x /etc/init.d/vncserver

# Start the service script we just moved
sudo service vncserver start
