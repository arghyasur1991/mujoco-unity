// Copyright 2019 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Linq;
using System.Xml;
using UnityEngine;
using Mujoco.Mjb;

namespace Mujoco {

// The component represents a physical shape and models its inertia and material properties.
public class MjGeom : MjShapeComponent {
  [Tooltip("If larger than zero, Density has no effect.")]
  public float Mass = 0.0f;

  [Tooltip("Material density. Set to 0 for a zero-mass geom.")]
  public float Density = 1000.0f;

  [Tooltip("Advanced settings.")]
  public MjGeomSettings Settings = MjGeomSettings.Default;

  public override mjtObj ObjectType => mjtObj.mjOBJ_GEOM;
  private MjTransformation _geomInGlobalFrame = new MjTransformation();
  private MjTransformation _comTransform;

  protected override void OnParseMjcf(XmlElement mjcf) {
    ShapeFromMjcf(mjcf);
    Mass = mjcf.GetFloatAttribute("mass", defaultValue: 0.0f);
    Density = mjcf.GetFloatAttribute("density", defaultValue: 1000.0f);
    MjEngineTool.ParseTransformMjcf(mjcf, transform);
    Settings.FromMjcf(mjcf);
  }

  protected override void OnBindToRuntime(MjbModel model, MjbData data) {
    var MjParent = MjHierarchyTool.FindParentComponent<MjBaseBody>(this);
    if (MjParent != null) {
      var comInParentFrame = new MjTransformation(
          translation: MjEngineTool.UnityVector3AtEntry(model.GeomPos, MujocoId),
          rotation: MjEngineTool.UnityQuaternionAtEntry(model.GeomQuat, MujocoId));

      var globalParentFrame = MjTransformation.LoadGlobal(transform.parent);
      var comInGlobalFrame = globalParentFrame * comInParentFrame;
      var globalFrame = MjTransformation.LoadGlobal(transform);
      _comTransform = comInGlobalFrame.Inverse() * globalFrame;
    }
  }

  protected override XmlElement OnGenerateMjcf(XmlDocument doc) {
    var mjcf = (XmlElement)doc.CreateElement("geom");
    if (Mass > 0) {
      mjcf.SetAttribute("mass", MjEngineTool.MakeLocaleInvariant($"{Mass}"));
    } else {
      mjcf.SetAttribute("density", MjEngineTool.MakeLocaleInvariant($"{Density}"));
    }
    ShapeToMjcf(mjcf, transform);
    MjEngineTool.PositionRotationToMjcf(mjcf, this);
    Settings.ToMjcf(mjcf);

    return mjcf;
  }

  public override void OnSyncState(MjbData data) {
    var geomXpos = data.GetGeomXpos();
    var geomXmat = data.GetGeomXmat();
    if (ShapeType == ShapeTypes.Mesh) {
      _geomInGlobalFrame.Set(
          translation: MjEngineTool.UnityVector3AtEntry(geomXpos, MujocoId),
          rotation: MjEngineTool.UnityQuaternionFromMatrixAtEntry(geomXmat, MujocoId));
      var comInGlobalFrame = _geomInGlobalFrame * _comTransform;
      comInGlobalFrame.StoreGlobal(transform);
    } else {
      transform.position = MjEngineTool.UnityVector3AtEntry(geomXpos, MujocoId);
      transform.rotation = MjEngineTool.UnityQuaternionFromMatrixAtEntry(geomXmat, MujocoId);
    }
  }

  public void OnDrawGizmosSelected() {
    Gizmos.color = Color.blue;
    DrawGizmos(transform);
  }
}
}
