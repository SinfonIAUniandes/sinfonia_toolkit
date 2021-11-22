#!/tmp/gentoo/bin/bash

if [[ $_ != $0 ]]; then
    echo "Este archivo no puede ser ejecutado con 'source'"
    echo "Ejecute como: './compress_and_send.sh <folder>'"
    return
fi

if [ $# -eq 0 ]; then
    echo "No arguments provided"
    exit 1
fi

if ! [ -f $1 ]; then
    echo "El archivo $1 no existe"
    exit 1
fi

echo "Enviando archivo $1"
source ~/set_pepper_ip.sh
sshpass -p $PEPPER_PASS scp $1 $PEPPER_USER@$PEPPER_IP:/home/$PEPPER_USER/



