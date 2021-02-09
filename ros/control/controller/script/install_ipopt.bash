#!/bin/bash
# Pass the Ipopt source directory as the first argument

sudo apt-get -y install gfortran

if ( ldconfig -p | grep libipopt ); then
    echo "Ipopt is already installed......."
    exit
fi

echo "Downloading Ipopt-3.12.7"

wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip

cd Ipopt-3.12.7
prefix=/usr/local
srcdir=$PWD

echo "Building Ipopt from ${srcdir}"
echo "Saving headers and libraries to ${prefix}"

echo "Configuring and building IPOPT"
./configure --prefix /usr/local

make -j$(nproc)
make test
sudo make install
if (grep 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' $HOME/.bashrc); then
    echo "LD_LIBRARY_PATH has been set."
else
    echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> $HOME/.bashrc
fi
sudo ldconfig
echo "IPOPT installed successfully"
source $HOME/.bashrc
rm -r Ipopt-3.12.7
