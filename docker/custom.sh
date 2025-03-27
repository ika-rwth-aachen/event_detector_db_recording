#!/bin/bash

set -e

# install dependencies for mongoc(xx) build
apt-get install -y wget libssl-dev libsasl2-dev ninja-build

# install mongoc
MONGOC_VERSION="1.24.0"
MONGOC_INSTALL_DIR="/usr/local"
echo "Installing mongoc $MONGOC_VERSION to $MONGOC_INSTALL_DIR"
cd /tmp
wget -q https://github.com/mongodb/mongo-c-driver/releases/download/$MONGOC_VERSION/mongo-c-driver-$MONGOC_VERSION.tar.gz -O mongo-c-driver.tar.gz
tar xzf mongo-c-driver.tar.gz
mkdir -p mongo-c-driver-$MONGOC_VERSION/build
cd mongo-c-driver-$MONGOC_VERSION/build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=$MONGOC_INSTALL_DIR \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_TESTS=OFF \
  -DENABLE_EXAMPLES=OFF \
  -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF \
  -G Ninja
cmake --build .
cmake --build . --target install
cd /tmp
rm -r mongo-c-driver*

# install mongocxx
MONGOCXX_VERSION="3.8.0"
MONGOCXX_INSTALL_DIR="/usr/local"
echo "Installing mongocxx $MONGOCXX_VERSION to $MONGOCXX_INSTALL_DIR"
cd /tmp
wget -q https://github.com/mongodb/mongo-cxx-driver/archive/r$MONGOCXX_VERSION.tar.gz -O mongo-cxx-driver.tar.gz
tar -xzf mongo-cxx-driver.tar.gz
cd mongo-cxx-driver-r$MONGOCXX_VERSION/build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=$MONGOCXX_INSTALL_DIR \
  -DCMAKE_PREFIX_PATH=$MONGOC_INSTALL_DIR \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_VERSION=$MONGOCXX_VERSION \
  -DENABLE_TESTS=OFF \
  -G Ninja
cmake --build . --target EP_mnmlstc_core
cmake --build .
cmake --build . --target install
cd /tmp
rm -r mongo-cxx-driver*
