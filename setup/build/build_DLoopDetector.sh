echo "Configuring and building Thirdparty/DBoW2 ..."

cd ../../
export topdir=$(pwd)

cd src/frontend/vo/include/thirdparty/DLoopDetector
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install 

echo "Uncompress vocabulary ..."

cd ${topdir}/vocabulary
tar -xf ORBvoc.txt.tar.gz
./../src/frontend/vo/include/thirdparty/DLoopDetector/build/dependencies/src/DBoW2/tools/bin_vocabulary

cd ${topdir}