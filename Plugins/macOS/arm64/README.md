# Native Plugins (macOS arm64)

| Library | Source | Purpose |
|---|---|---|
| `libmujoco.dylib` | [MuJoCo](https://github.com/google-deepmind/mujoco/releases) or `pip install mujoco` | MuJoCo C physics engine |
| `libmjaccess.dylib` | Built from `NativeShim/mjaccess.c` in this repo | Pure C accessor shim (double-precision getters/setters) |

## Build instructions

```bash
# Find MuJoCo headers/libs from conda/pip installation
MUJOCO_DIR=$(python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")

# Compile the accessor shim
cc -shared -O2 -fPIC \
  -o libmjaccess.dylib \
  ../../NativeShim/mjaccess.c \
  -I"$MUJOCO_DIR/include" \
  -L"$MUJOCO_DIR/dylib" \
  -lmujoco \
  -install_name @loader_path/libmjaccess.dylib

# Copy libmujoco from conda if not already present
cp "$MUJOCO_DIR/dylib/libmujoco.3.2.7.dylib" libmujoco.dylib
```

Or from the project root: `python build.py --mjaccess`
