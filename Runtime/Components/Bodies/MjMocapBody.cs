// Copyright 2026 Arghya Sur / Mobyr  –  Apache-2.0
//
// A MuJoCo mocap body — kinematically controlled, bypasses the physics integrator.
//
// WHAT IT DOES:
//   Generates a <body mocap="true"> element in the MJCF.  Mocap bodies are special:
//   • They have their own collision geometry (geoms) and participate in contact detection.
//   • They are NOT integrated by MuJoCo's dynamics — you drive them by writing
//     data.mocap_pos[mocapIdx] and data.mocap_quat[mocapIdx] each step.
//   • Contact forces from mocap geometry are applied to any dynamic bodies they touch,
//     so MuJoCo's solver handles push/poke/nudge naturally.
//
// TYPICAL USE — VR player hands:
//   1. Add a child GameObject to the MuJoCo scene root (sibling of the Synth).
//   2. Attach MjMocapBody + MjGeom (sphere or capsule) to it.
//   3. PlayerHandBodies.cs reads body_mocapid and writes mocap_pos/quat each frame.
//
// COORDINATE NOTE:
//   mocap_pos and mocap_quat are in MuJoCo world frame (Y↑, Z into screen).
//   Use MjEngineTool.SetMjVector3AtEntry / SetMjQuaternionAtEntry for conversion.

using System.Xml;
using UnityEngine;
using Mujoco.Mjb;

namespace Mujoco
{
    public class MjMocapBody : MjBody
    {
        protected override XmlElement OnGenerateMjcf(XmlDocument doc)
        {
            var mjcf = base.OnGenerateMjcf(doc);
            mjcf.SetAttribute("mocap", "true");
            return mjcf;
        }

        // Mocap body position is driven by PlayerHandBodies writing mocap_pos/quat —
        // MuJoCo keeps xpos/xquat in sync with those, so the default OnSyncState
        // (which reads xpos/xquat) would fight our updates.  Override to be a no-op.
        public override void OnSyncState(MjbData data) { }
    }
}
