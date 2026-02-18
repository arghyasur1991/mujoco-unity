# Native Plugins (macOS arm64)

Place the following dylibs here before running in Unity:

| Library | Source | Purpose |
|---|---|---|
| `libmujoco.dylib` | [MuJoCo release](https://github.com/google-deepmind/mujoco/releases) | MuJoCo C physics engine |
| `libmjmlx.dylib` | Build from [MuJoCo-MLX-Cpp](https://github.com/arghyasur1991/MuJoCo-MLX-Cpp) | MLX Metal GPU physics |
| `libmjb.dylib` | Build from [MuJoCo-MLX-Cpp](https://github.com/arghyasur1991/MuJoCo-MLX-Cpp) | Unified backend API |

## Build instructions

```bash
cd MuJoCo-MLX-Cpp
mkdir build && cd build
cmake .. -DMUJOCO_ROOT=/path/to/mujoco -DMLX_ROOT=/path/to/mlx -DCMAKE_BUILD_TYPE=Release
make -j mjmlx mjb
cp libmjmlx.0.1.0.dylib /path/to/mujoco-unity/Plugins/macOS/arm64/libmjmlx.dylib
cp libmjb.0.1.0.dylib /path/to/mujoco-unity/Plugins/macOS/arm64/libmjb.dylib
```
