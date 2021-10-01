#!/usr/bin/env bash

install_system_updates(){
    echo "updating system."
    sudo apt-get update
    sudo apt-get upgrade -y
}

install_dependencies(){
    sudo apt-get install -y gcc-arm-none-eabi
    sudo apt-get install -y dfu-util
}

build_firmware(){
    git clone https://github.com/groupgets/purethermal1-firmware
    cd purethermal1-firmware
    make
}

flash(){
    dfu-util -a 0 -D main.bin -s 0x08000000
    ./scripts/flash.sh
}

main(){
    install_system_updates
    install_dependencies
    build_firmware
    flash
}


main "$@"