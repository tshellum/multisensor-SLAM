cd ../
export topdir=$(pwd)

echo "Uncompress vocabulary ..."
cd ${topdir}/vocabulary
tar -xf ORBvoc.txt.tar.gz


echo "Convert to .bin vocabulary ..."
cd convert
if [ ! -d "build" ] 
then
  mkdir build
fi
cd build
cmake .. -DCMAKE_C_COMPILER=gcc
make

cd ${topdir}/vocabulary
./convert/build/convert_txt2bin

