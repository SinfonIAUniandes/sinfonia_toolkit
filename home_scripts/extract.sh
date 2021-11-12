#!/bin/bash

if [[ $_ != $0 ]]; then
    echo "Este archivo no puede ser ejecutado con 'source'"
    echo "Ejecute como: './compress_and_send.sh <folder>'"
    return
fi

if [ $# -eq 0 ]; then
    echo "No arguments provided"
    exit 1
fi

tar -xzvf $1

