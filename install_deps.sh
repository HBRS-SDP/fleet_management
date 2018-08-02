#!/bin/bash

# Default values
SUDO="sudo"
INSTALL_DIR="/usr/local/"
WORKSPACE_DIR="./"
J="-j1"
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Handle command line options
while :; do
  case $1 in
    -h|-\?|--help)   # Call a "show_help" function to display a synopsis, then exit.
      show_usage
      exit
      ;;
    --no-sudo)
      SUDO=""
      ;;
    --workspace-path)       # Takes an option argument, ensuring it has been specified.
      if [ -n "$2" ]; then
           WORKSPACE_DIR=$2
          shift
       else
           printf 'ERROR: "--workspace-path" requires a non-empty option argument.\n' >&2
           exit 1
       fi
       ;;
    --workspace-path=?*)
       WORKSPACE_DIR=${1#*=} # Delete everything up to "=" and assign the remainder.
       ;;
    --workspace-path=)         # Handle the case of an empty --file=
      printf 'ERROR: "--workspace-path" requires a non-empty option argument.\n' >&2
      exit 1
       ;;
    --install-path)       # Takes an option argument, ensuring it has been specified.
      if [ -n "$2" ]; then
           INSTALL_DIR=$2
          shift
       else
           printf 'ERROR: "--install-path" requires a non-empty option argument.\n' >&2
           exit 1
       fi
       ;;
    --install-path=?*)
       INSTALL_DIR=${1#*=} # Delete everything up to "=" and assign the remainder.
       ;;
    --install-path=)         # Handle the case of an empty --file=
      printf 'ERROR: "--install-path" requires a non-empty option argument.\n' >&2
      exit 1
      ;;

    -j)       # Takes an option argument, ensuring it has been specified.
      if [ -n "$2" ]; then
           J="-j${2}"
          shift
       else
           printf 'ERROR: "-j" requires a non-empty option argument.\n' >&2
           exit 1
       fi
       ;;
    -j=?*)
       J_TMP=${1#*=} # Delete everything up to "=" and assign the remainder.
       J="-j${J_TMP}"
       ;;
    -j=)         # Handle the case of an empty --file=
      printf 'ERROR: "-j" requires a non-empty option argument.\n' >&2
      exit 1
      ;;


    --)              # End of all options.
       shift
       break
       ;;
    -?*)
       printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
        ;;
    *)               # Default case: If no more options then break out of the loop.
        break
  esac

shift
done

# Go to workspace
cd ${WORKSPACE_DIR}

echo "Installing yaml-cpp"
wget https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.2.tar.gz
tar -xzf yaml-cpp-0.6.2.tar.gz
cd yaml-cpp-yaml-cpp-0.6.2
mkdir build
cd build
cmake ..
make
${SUDO} make install
cd ../..

echo "Installing MongoDB C Driver and BSON Library"
wget https://github.com/mongodb/mongo-c-driver/releases/download/1.12.0/mongo-c-driver-1.12.0.tar.gz
tar xzf mongo-c-driver-1.12.0.tar.gz
cd mongo-c-driver-1.12.0
mkdir cmake-build
cd cmake-build
cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF ..
make
${SUDO} make install
cd ../..

echo "Installing Mongocxx Driver"
curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/r3.3.1.tar.gz
tar -xzf r3.3.1.tar.gz
cd mongo-cxx-driver-r3.3.1/build
cmake -DCMAKE_BUILD_TYPE=Release -DBSONCXX_POLY_USE_MNMLSTC=1 -DCMAKE_INSTALL_PREFIX=/usr/local ..
${SUDO} make EP_mnmlstc_core
make
${SUDO} make install
cd ../..

cd ${SCRIPT_DIR} # go back
set +e
echo "SUCCESS"
