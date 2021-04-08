echo "Configuring and building JET ..."

cd ../../
export topdir=$(pwd)

cd src/frontend/vo/include/thirdparty/JET
mkdir build
cd build
cmake .. -DWITH_PYTHON=OFF
make -j

cd ${topdir}