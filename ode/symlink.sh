#!/bin/bash

if [ $# -lt 1 ]
then
  echo "Usage: $0 PREFIX"
  echo
  echo "This script symlinks your local ODE installation into the source tree."
  echo "Run it by specifying the --prefix where you installed ODE."
  exit 1
fi

set -x

source=$1
target=ode
if [ "$(basename $(pwd))" = "ode" ]
then target="."
fi

ln -sf $source/include/ode $target/include
ln -sf $source/lib/libode* $target/lib

ls -lR $target

echo "All done !"
