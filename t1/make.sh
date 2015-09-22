#!/bin/sh

bold=$(tput bold)
reset=$(tput sgr0)

if [ "$1" = "clean" ]; then
  echo "${bold}Cleaning...${reset}"
  rm -r build bin
else
  mkdir -p build
  cd build
  cmake ..
  make
  cd ../bin
fi

