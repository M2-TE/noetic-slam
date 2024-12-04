#!/bin/bash

PIPE="/root/repo/pipe/pipe"

if [ ! -p "$PIPE" ]; then
    mkfifo "$PIPE"
fi

echo "cd imagesaver && bash runimagesaver.bash" > "$PIPE"
