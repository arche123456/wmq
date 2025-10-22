pushd `pwd` > /dev/null
cd `dirname $0`

mkdir build
cd build
cmake .. && make -j
sudo make install

popd > /dev/null