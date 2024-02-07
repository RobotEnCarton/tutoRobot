Vous retrouverez ici les sources pour le tuto 4 sur le système de commande du robot.

-Le code sur l'asservissement en position absolue du robot modifié
-Les codes de commande du robot sur arduino et sur raspi

commande le gestionnaire de périphérique:

pour lister les id d'un périphérique:
udevadm info -q all -a /dev/ttyUSB0 | grep ATTRS

exemple :
lidar
    ATTRS{idVendor}=="10c4"
    ATTRS{idProduct}=="ea60"

arduino moteur
    ATTRS{idVendor}=="2a03"
    ATTRS{idProduct}=="0043"

fichier USB.rules à créer :
KERNEL=="ttyUSB?", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", NAME="USB_LIDAR", SYMLINK="USB_LIDAR"
KERNEL=="ttyACM?", ATTRS{idVendor}=="2a03", ATTRS{idProduct}=="0043", NAME="USB_MOTOR", SYMLINK="USB_MOTOR"

donner le fichier au gestionnaire :
sudo cp USB.rules /etc/udev/rules.d/

dire au gestionnaire de recharger ses règles :
udevadm control --reload-rules && udevadm trigger

Amusez-vous avec vos robots!

Droop
