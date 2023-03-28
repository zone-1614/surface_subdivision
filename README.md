曲面细分算法合集

# build 
clone
```
git clone xxx
cd xxx
git submodule update --init --recursive
```

build executable:
```
cmake -B build 
cmake --build build
```

build emscripten:
```
emcmake cmake . -B cmake-build-emscripten -G "CodeBlocks - MinGW Makefiles"
cmake --build cmake-build-emscripten
```


> 自己的 vcpkg -DCMAKE_TOOLCHAIN_FILE:STRING="D:/software/vcpkg/scripts/buildsystems/vcpkg.cmake"

