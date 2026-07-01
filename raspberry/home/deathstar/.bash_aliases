ctrl1() {
    cd ~/deathstar/control/controller1 || return
    cd build || return
    make -j6
    sudo ./main
}


shut() { 
	sudo shutdown now
}
