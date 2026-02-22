# MuJoCo for Unity

Double-precision MuJoCo physics for Unity. Drop-in superset of the DeepMind `org.mujoco` Unity plugin with native `double` precision throughout and batched simulation for RL training.

## What is this?

This package provides:

1. **All original MuJoCo Unity components** -- `MjScene`, `MjBody`, `MjGeom`, `MjJoint`, `MjActuator`, etc. Scene authoring, MJCF import/export, and visualization work exactly as before.

2. **MuJoCo accessor shim (`libmjaccess`)** -- a thin pure-C library that exposes `mjModel`/`mjData` struct fields as typed accessor functions. All getters return native `double*` pointers directly into MuJoCo's arrays (zero-copy, zero-conversion). All setters use `memcpy` into native buffers.

3. **C# wrappers** -- `MjbModel`, `MjbData`, `MjbBatchedSim` provide managed access via `MjbDoubleSpan` (unsafe `double*` + length). The `IMjPhysicsBackend` interface and `MjCpuBackend` implementation use `double` throughout.

4. **Batched simulation** -- step hundreds of environments in parallel using GCD `dispatch_apply` for reinforcement learning training.

## Architecture

```
com.mobyr.mujoco
├── Runtime/
│   ├── Components/         # MjScene, MjBody, MjGeom, ... (from org.mujoco)
│   ├── Bindings/
│   │   ├── MjBindings.cs          # Original mj_* P/Invoke (backward compat)
│   │   ├── MjbNativeMethods.cs    # mjaccess_* P/Invoke (accessor shim API)
│   │   └── MjbTypes.cs            # MjbDoubleSpan, MjbFloatSpan, config structs
│   ├── Wrappers/
│   │   ├── MjbModel.cs            # Model handle with double-precision accessors
│   │   ├── MjbData.cs             # Simulation state (double getters/setters)
│   │   └── MjbBatchedSim.cs       # Vectorized multi-env simulation
│   ├── Backend/
│   │   ├── IMjPhysicsBackend.cs   # Physics interface (double throughout)
│   │   ├── MjCpuBackend.cs        # CPU backend implementation
│   │   └── BatchedPhysicsProxy.cs # Per-env slice into batched sim
│   └── NativeLoader/
│       └── MjbNativeLoader.cs     # dlopen preloader for native libs
├── NativeShim/
│   ├── mjaccess.c                 # Pure C accessor shim (~200 lines)
│   └── mjaccess.h                 # Public API header
├── Plugins/
│   └── macOS/arm64/
│       ├── libmujoco.dylib        # MuJoCo C engine
│       └── libmjaccess.dylib      # Accessor shim (pure C, ~40KB)
└── Editor/                         # Scene authoring tools (from org.mujoco)
```

### Dependency graph

```
mujoco-unity (this package)
  ├── libmjaccess.dylib ── Pure C accessor shim (mjaccess_*)
  │     └── libmujoco.dylib ── MuJoCo C physics engine
  └── MjBindings.cs ──────── Original mj_* API (backward compat)
        └── libmujoco.dylib
```

## Quick start

### Installation via UPM (git URL)

Add to your `Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.mobyr.mujoco": "https://github.com/arghyasur1991/mujoco-unity.git"
  }
}
```

Or use a local path during development:

```json
{
  "dependencies": {
    "com.mobyr.mujoco": "file:../../mujoco-unity"
  }
}
```

### Using the API

```csharp
using Mujoco.Mjb;

// Load model (static factory, no backend handle needed)
using var model = MjbModel.Load("humanoid.xml");

// Single environment
using var data = model.MakeData();
data.SetCtrl(new double[] { 0.1, 0.2, ... });
data.Step();
MjbDoubleSpan qpos = data.GetQpos();  // zero-copy double* into MuJoCo

// Batched simulation (128 environments on CPU, GCD parallel)
var config = new MjbBatchedConfig {
    numEnvs = 128,
    solverIterations = 3
};
using var sim = model.CreateBatchedSim(config);
sim.Step(batchedCtrl);
MjbDoubleSpan batchedQpos = sim.GetQpos();
```

### Using original MuJoCo components

All existing `org.mujoco` functionality works unchanged:

- Add `MjScene` to your scene
- Build robots with `MjBody`, `MjGeom`, `MjJoint`, `MjActuator`
- Import MJCF XML files
- Use `MjScene.Instance.Model` and `MjScene.Instance.Data` for direct access

## Unity 6 compatibility

This package includes fixes for Unity 6 that the upstream `org.mujoco` plugin has not yet addressed:

- `FindObjectsOfType` replaced with `FindObjectsByType`
- `MjScene` singleton pattern fixed for domain reload
- `MjMouseSpring.OnDisable` GUI context fix
- Deprecated `passive` flag attribute removed for MuJoCo 3.x
- Null/readability checks for mesh generation
- `PauseSimulation` and `ResetData` for RL training control

## Native library build

The accessor shim is built from source in `NativeShim/`:

```bash
# Requires MuJoCo installed (e.g. via pip install mujoco)
MUJOCO_DIR=$(python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")

cc -shared -O2 -fPIC \
  -o libmjaccess.dylib \
  NativeShim/mjaccess.c \
  -I"$MUJOCO_DIR/include" \
  -L"$MUJOCO_DIR/dylib" \
  -lmujoco \
  -install_name @loader_path/libmjaccess.dylib
```

Or use the project build system: `python build.py --mjaccess`

See `Plugins/macOS/arm64/README.md` for more details.

## Key design decisions

- **Double precision throughout**: MuJoCo uses `double` internally. The C shim returns `double*` directly — no conversion buffers, no float32 truncation. `float` casts only happen at the Unity API boundary (Vector3/Quaternion) and neural network boundary (TorchSharp tensors).
- **Zero-copy getters**: `mjaccess_get_qpos()` returns a pointer into `mjData.qpos` — no allocation, no memcpy.
- **Thin C shim**: ~200 lines of pure C. MuJoCo exposes `mjModel`/`mjData` as C structs whose layout may change between versions; the shim provides stable accessor functions so C# doesn't need to mirror struct layouts.
- **GCD batched stepping**: `dispatch_apply` for parallel `mj_step` across environments on macOS.

## License

Apache License 2.0. See [LICENSE](LICENSE).

Based on the [MuJoCo Unity plugin](https://github.com/google-deepmind/mujoco) by Google DeepMind (Apache 2.0).
