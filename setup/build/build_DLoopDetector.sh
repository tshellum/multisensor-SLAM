echo "Configuring and building Thirdparty/DBoW2 ..."

cd ../../
export topdir=$(pwd)

# cd src/frontend/vo/include/thirdparty/DLoopDetector
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j
# sudo make install 


# cd ${topdir}/src/frontend/vo/include/thirdparty/DBoW2
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ${topdir}/src/frontend/vo/include/thirdparty/DBoW2_orb
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ${topdir}/src/frontend/vo/include/thirdparty/DLib
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

echo "Uncompress vocabulary ..."

cd ${topdir}/vocabulary
tar -xf ORBvoc.txt.tar.gz
./../src/frontend/vo/include/thirdparty/DLoopDetector/build/dependencies/src/DBoW2/tools/bin_vocabulary

cd ${topdir}