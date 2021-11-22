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

if ! [ -d $1 ]; then
    echo "El directorio $1 no existe"
    exit 1
fi

echo "Comprimiendo carpeta $1 ..."
tar cf - ./$1 -P | pv -s $(du -sb ./$1 | awk '{print $1}') | gzip > $1.tar.gz
