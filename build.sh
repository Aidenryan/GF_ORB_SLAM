echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release # -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../

echo "Configuring and building ORB_SLAM1 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release # -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make -j4

# cd ..
# echo "Converting vocabulary to binary"
# ./tools/bin_vocabulary
