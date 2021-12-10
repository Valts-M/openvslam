#!/bin/bash

mkdir -p ~/.config/autostart
cp /usr/share/applications/vino-server.desktop ~/.config/autostart/.

gsettings set org.gnome.Vino prompt-enabled false
gsettings set org.gnome.Vino require-encryption false

gsettings set org.gnome.Vino authentication-methods "['vnc']"
gsettings set org.gnome.Vino vnc-password $(echo -n 'slambox'|base64)

sudo echo "
Section \"Screen\"
   Identifier    \"Default Screen\"
   Monitor        \"Configured Monitor\"
   Device        \"Default Device\"
   SubSection \"Display\"
       Depth    24\n
       Virtual 1920 1080
   EndSubSection
EndSection" >> /etc/X11/xorg.conf

sudo reboot now