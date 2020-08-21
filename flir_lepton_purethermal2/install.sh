#!/usr/bin/env bash

install_system_updates(){
    echo "updating system."
    sudo apt-get update
    sudo apt-get upgrade -y
}

install_dependencies(){
    echo "install dependencies."
    sudo apt-get install -y git 
    sudo apt-get install -y cmake
    sudo apt-get install -y libusb-1.0-0-dev
    sudo apt-get install -y libjpeg-dev
    sudo apt-get install -y python-opencv
}

install_libuvc(){
    echo "install libuvc."
    git clone https://github.com/groupgets/libuvc
    cd libuvc
    mkdir build
    cd build
    cmake ..
    make && sudo make install
    sudo ldconfig -v 
    cd ../..
}

add_usb_permission(){
    echo "add usb permission."
    sudo sh -c "echo 'SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"1e4e\", ATTRS{idProduct}==\"0100\", SYMLINK+=\"pt1\", GROUP=\"usb\", MODE=\"666\"' > /etc/udev/rules.d/99-pt1.rules"
}

main () {
    install_system_updates
    install_dependencies
    install_libuvc

    add_usb_permission

    echo "please reboot or reload udev rules."

}

main "$@"
