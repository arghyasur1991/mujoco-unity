// MuJoCo enum constants used by the mujoco-unity component system.
// These are integer enums matching the MuJoCo C header values.
// They have no P/Invoke dependency — just compile-time constants.

namespace Mujoco
{
    public enum mjtObj : int
    {
        mjOBJ_UNKNOWN = 0,
        mjOBJ_BODY = 1,
        mjOBJ_XBODY = 2,
        mjOBJ_JOINT = 3,
        mjOBJ_DOF = 4,
        mjOBJ_GEOM = 5,
        mjOBJ_SITE = 6,
        mjOBJ_CAMERA = 7,
        mjOBJ_LIGHT = 8,
        mjOBJ_FLEX = 9,
        mjOBJ_MESH = 10,
        mjOBJ_SKIN = 11,
        mjOBJ_HFIELD = 12,
        mjOBJ_TEXTURE = 13,
        mjOBJ_MATERIAL = 14,
        mjOBJ_PAIR = 15,
        mjOBJ_EXCLUDE = 16,
        mjOBJ_EQUALITY = 17,
        mjOBJ_TENDON = 18,
        mjOBJ_ACTUATOR = 19,
        mjOBJ_SENSOR = 20,
        mjOBJ_NUMERIC = 21,
        mjOBJ_TEXT = 22,
        mjOBJ_TUPLE = 23,
        mjOBJ_KEY = 24,
        mjOBJ_PLUGIN = 25,
        mjNOBJECT = 26,
        mjOBJ_FRAME = 100,
        mjOBJ_DEFAULT = 101,
        mjOBJ_MODEL = 102,
    }

    public enum mjtJoint : int
    {
        mjJNT_FREE = 0,
        mjJNT_BALL = 1,
        mjJNT_SLIDE = 2,
        mjJNT_HINGE = 3,
    }

    public enum mjtDyn : int
    {
        mjDYN_NONE = 0,
        mjDYN_INTEGRATOR = 1,
        mjDYN_FILTER = 2,
        mjDYN_FILTEREXACT = 3,
        mjDYN_MUSCLE = 4,
        mjDYN_USER = 5,
    }

    public enum mjtGain : int
    {
        mjGAIN_FIXED = 0,
        mjGAIN_AFFINE = 1,
        mjGAIN_MUSCLE = 2,
        mjGAIN_USER = 3,
    }

    public enum mjtBias : int
    {
        mjBIAS_NONE = 0,
        mjBIAS_AFFINE = 1,
        mjBIAS_MUSCLE = 2,
        mjBIAS_USER = 3,
    }
}
