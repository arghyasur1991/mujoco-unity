# MuJoCo Android arm64-v8a Plugins

Place the following files here after running `python build.py --mujoco-android`:

- `libmujoco.so` — MuJoCo physics engine (cross-compiled from mujoco-src/ with Android NDK)
- `libmjaccess.so` — C accessor shim (cross-compiled from NativeShim/mjaccess.c)

These `.so` files are build artifacts and not tracked in git.
Run `python build.py --mujoco-android` from the repo root to build them.
