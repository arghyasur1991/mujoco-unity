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
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Xml;
using UnityEngine;
using Mujoco.Mjb;

namespace Mujoco {

public static class MjEngineTool {
  private const int _elementsPerPosition = 3;
  private const int _elementsPerRotation = 4;
  private const int _elementsPerTransform = 7;
  private const int _elementsPerEquality = 11;

  public static string Sanitize(string name) {
    return name.Replace('/', '_');
  }

  public static string MakeLocaleInvariant(FormattableString interpolated) {
    return FormattableString.Invariant(interpolated);
  }

  // ── Model I/O via mjb ──────────────────────────────────────────────────

  public static MjbModel LoadModel(string xmlPath) {
    return MjbModel.Load(xmlPath);
  }

  public static MjbModel LoadModelFromString(string contents) {
    return MjbModel.LoadFromString(contents);
  }

  public static void SaveModelToFile(string fileName, MjbModel model) {
    model.SaveLastXml(fileName);
  }

  // ── Double-pointer helpers (mjb API returns double*) ────────────────────

  public static unsafe void SetMjVector3(double* mjTarget, Vector3 unityVec) {
    var mjVec = MjVector3(unityVec);
    mjTarget[0] = mjVec[0];
    mjTarget[1] = mjVec[1];
    mjTarget[2] = mjVec[2];
  }

  public static unsafe void SetMjQuaternion(double* mjTarget, Quaternion unityQuat) {
    var mjQuat = MjQuaternion(unityQuat);
    mjTarget[0] = mjQuat.w;
    mjTarget[1] = mjQuat.x;
    mjTarget[2] = mjQuat.y;
    mjTarget[3] = mjQuat.z;
  }

  public static unsafe void SetMjTransform(
      double* mjTarget, Vector3 unityVec, Quaternion unityQuat) {
    var mjVec = MjVector3(unityVec);
    var mjQuat = MjQuaternion(unityQuat);
    mjTarget[0] = mjVec[0];
    mjTarget[1] = mjVec[1];
    mjTarget[2] = mjVec[2];
    mjTarget[3] = mjQuat.w;
    mjTarget[4] = mjQuat.x;
    mjTarget[5] = mjQuat.y;
    mjTarget[6] = mjQuat.z;
  }

  public static unsafe Vector3 UnityVector3(double* mjVector) {
    return new Vector3((float)mjVector[0], (float)mjVector[2], (float)mjVector[1]);
  }

  public static unsafe Quaternion UnityQuaternion(double* mjQuat) {
    return new Quaternion(
        x:(float)mjQuat[1], y:(float)mjQuat[3], z:(float)mjQuat[2], w:(float)(-mjQuat[0]));
  }

  // ── Raw double* entry helpers (for test round-trips and manual pointer math) ─

  public static unsafe Vector3 MjVector3AtEntry(double* ptr, int entryIndex) {
    double* p = ptr + entryIndex * _elementsPerPosition;
    return new Vector3((float)p[0], (float)p[1], (float)p[2]);
  }

  public static unsafe Quaternion MjQuaternionAtEntry(double* ptr, int entryIndex) {
    double* p = ptr + entryIndex * _elementsPerRotation;
    return new Quaternion(w:(float)p[0], x:(float)p[1], y:(float)p[2], z:(float)p[3]);
  }

  // ── Span-based helpers (for MjbDoubleSpan arrays from mjb getters) ─────

  public static unsafe Vector3 UnityVector3AtEntry(MjbDoubleSpan span, int entryIndex) {
    double* p = span.Data + entryIndex * _elementsPerPosition;
    return new Vector3((float)p[0], (float)p[2], (float)p[1]);
  }

  public static unsafe Quaternion UnityQuaternionAtEntry(MjbDoubleSpan span, int entryIndex) {
    double* p = span.Data + entryIndex * _elementsPerRotation;
    return new Quaternion(x:(float)p[1], y:(float)p[3], z:(float)p[2], w:(float)(-p[0]));
  }

  public static unsafe Quaternion UnityQuaternionFromMatrixAtEntry(MjbDoubleSpan span, int entryIndex) {
    double* p = span.Data + entryIndex * 9;
    return UnityQuaternionFromMatrix(p);
  }

  public static unsafe void SetMjVector3AtEntry(MjbDoubleSpan span, int entryIndex, Vector3 unityVec) {
    double* p = span.Data + entryIndex * _elementsPerPosition;
    SetMjVector3(p, unityVec);
  }

  public static unsafe void SetMjQuaternionAtEntry(MjbDoubleSpan span, int entryIndex, Quaternion unityQuat) {
    double* p = span.Data + entryIndex * _elementsPerRotation;
    SetMjQuaternion(p, unityQuat);
  }

  public static unsafe void SetMjTransformAtEntry(MjbDoubleSpan span, int entryIndex, Vector3 unityVec, Quaternion unityQuat) {
    double* p = span.Data + entryIndex * _elementsPerEquality;
    SetMjTransform(p, unityVec, unityQuat);
  }

  // ── Mat2Quat (Shepperd's method, replaces mju_mat2Quat) ──────────────

  public static unsafe Quaternion UnityQuaternionFromMatrix(double* mat) {
    // Shepperd's method: convert 3x3 row-major rotation matrix to quaternion.
    // mat layout: [m00 m01 m02 m10 m11 m12 m20 m21 m22]
    float m00 = (float)mat[0], m01 = (float)mat[1], m02 = (float)mat[2];
    float m10 = (float)mat[3], m11 = (float)mat[4], m12 = (float)mat[5];
    float m20 = (float)mat[6], m21 = (float)mat[7], m22 = (float)mat[8];

    float trace = m00 + m11 + m22;
    float w, x, y, z;

    if (trace > 0) {
      float s = Mathf.Sqrt(trace + 1.0f) * 2.0f;
      w = 0.25f * s;
      x = (m21 - m12) / s;
      y = (m02 - m20) / s;
      z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
      float s = Mathf.Sqrt(1.0f + m00 - m11 - m22) * 2.0f;
      w = (m21 - m12) / s;
      x = 0.25f * s;
      y = (m01 + m10) / s;
      z = (m02 + m20) / s;
    } else if (m11 > m22) {
      float s = Mathf.Sqrt(1.0f + m11 - m00 - m22) * 2.0f;
      w = (m02 - m20) / s;
      x = (m01 + m10) / s;
      y = 0.25f * s;
      z = (m12 + m21) / s;
    } else {
      float s = Mathf.Sqrt(1.0f + m22 - m00 - m11) * 2.0f;
      w = (m10 - m01) / s;
      x = (m02 + m20) / s;
      y = (m12 + m21) / s;
      z = 0.25f * s;
    }

    // MuJoCo quat: [w x y z], Unity: Quaternion(x,y,z,w).
    // Apply MuJoCo→Unity swizzle: (mjX, mjZ, mjY, -mjW)
    return new Quaternion(x:x, y:z, z:y, w:-w);
  }

  // ── Pure geometry helpers (no pointer dependencies) ────────────────────

  public static Vector3 MjVector3(Vector3 unityVec) {
    return new Vector3(unityVec.x, unityVec.z, unityVec.y);
  }

  public static unsafe Vector3 UnityVector3(float[] coords, int entryIndex) {
    var startOffset = entryIndex * _elementsPerPosition;
    return new Vector3(coords[startOffset], coords[startOffset + 2], coords[startOffset + 1]);
  }

  public static Vector3 UnityVector3(Vector3 mjVector) {
    return new Vector3(mjVector.x, mjVector.z, mjVector.y);
  }

  public static Quaternion MjQuaternion(Quaternion unityQuat) {
    return new Quaternion(w:-unityQuat.w, x:unityQuat.x, y:unityQuat.z, z:unityQuat.y);
  }

  public static Quaternion UnityQuaternion(Quaternion mjQuat) {
    return new Quaternion(x:mjQuat.x, y:mjQuat.z, z:mjQuat.y, w:-mjQuat.w);
  }

  public static Vector3 MjExtents(Vector3 unityExtents) {
    return new Vector3(unityExtents.x, unityExtents.z, unityExtents.y);
  }

  public static Vector3 UnityExtents(Vector3 mjExtents) {
    return new Vector3(mjExtents.x, mjExtents.z, mjExtents.y);
  }

  public static Vector3 UnityEuler(Vector3 mjEuler) {
    return new Vector3(mjEuler.x, mjEuler.z, -mjEuler.y);
  }

  public static string Vector3ToMjcf(Vector3 vec) {
    return MakeLocaleInvariant($"{vec.x} {vec.y} {vec.z}");
  }

  public static string QuaternionToMjcf(Quaternion quat) {
    return MakeLocaleInvariant($"{quat.w} {quat.x} {quat.y} {quat.z}");
  }

  public static MjTransformation LocalTransformInParentBody(MjComponent component) {
    var MjParent = MjHierarchyTool.FindParentComponent<MjBaseBody>(component);
    var parentGlobalPosition = Vector3.zero;
    var parentGlobalRotation = Quaternion.identity;
    if (MjParent != null) {
      parentGlobalPosition = MjParent.transform.position;
      parentGlobalRotation = MjParent.transform.rotation;
    }

    var invParentGlobalRotation = Quaternion.Inverse(parentGlobalRotation);

    var localPosition =
        invParentGlobalRotation * (component.transform.position - parentGlobalPosition);
    var localRotation = invParentGlobalRotation * component.transform.rotation;
    return new MjTransformation(localPosition, localRotation);
  }

  public static readonly Quaternion MjQuaternionIdentity = new Quaternion(w:-1, x:0, y:0, z:0);
  public static readonly Vector3 MjVector3Up = new Vector3(0, 0, 1);

  public static Vector2 GetSorted(Vector2 vec) {
    return new Vector2(Math.Min(vec.x, vec.y), Math.Max(vec.x, vec.y));
  }

  public static string ArrayToMjcf(float[] array) {
    String ret = "";
    foreach (float entry in array) {
      ret += MakeLocaleInvariant($"{entry} ");
    }
    return ret.Substring(startIndex:0, length:ret.Length - 1);
  }

  public static string ListToMjcf(List<float> list) {
    String ret = "";
    foreach (float entry in list) {
      ret += MakeLocaleInvariant($"{entry} ");
    }
    return ret.Substring(startIndex:0, length:ret.Length - 1);
  }

  public static void PositionRotationToMjcf(XmlElement mjcf, MjComponent component) {
    var localTransform = LocalTransformInParentBody(component);
    mjcf.SetAttribute(
        "pos",
        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(localTransform.Translation)));
    mjcf.SetAttribute(
        "quat",
        MjEngineTool.QuaternionToMjcf(MjEngineTool.MjQuaternion(localTransform.Rotation)));
  }

  public static void PositionAxisToMjcf(XmlElement mjcf, MjComponent component) {
    var localTransform = LocalTransformInParentBody(component);
    var axis = (localTransform.Rotation * Vector3.right).normalized;

    mjcf.SetAttribute(
        "pos",
        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(localTransform.Translation)));
    mjcf.SetAttribute("axis", MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(axis)));
    mjcf.SetAttribute("ref", "0");
  }

  public static void PositionToMjcf(XmlElement mjcf, MjComponent component) {
    var localTransform = LocalTransformInParentBody(component);
    mjcf.SetAttribute(
        "pos",
        MjEngineTool.Vector3ToMjcf(MjEngineTool.MjVector3(localTransform.Translation)));
  }

  private const int _fromOffset = 0;
  private const int _toOffset = 1;
  private const float _fromToValidityTolerance = 1e-3f;

  public static bool ParseFromToMjcf(XmlElement mjcf, out Vector3 fromPoint, out Vector3 toPoint) {
    var fromto = mjcf.GetFloatArrayAttribute("fromto", defaultValue: null);
    if (fromto != null) {
      fromPoint = MjEngineTool.UnityVector3(fromto, _fromOffset);
      toPoint = MjEngineTool.UnityVector3(fromto, _toOffset);
      var nodeName = mjcf.GetStringAttribute("name", mjcf.Name);
      if ((toPoint - fromPoint).magnitude < _fromToValidityTolerance) {
        throw new ArgumentException(
            $"{nodeName}: 'fromto' produces a vector that's too short. {fromto} has magnitude " +
            "{fromto.magnitude} lower than the tolerance {_fromToValidityTolerance}");
      }
      return true;
    } else {
      fromPoint = Vector3.zero;
      toPoint = Vector3.zero;
      return false;
    }
  }

  public static void ParseTransformMjcf(XmlElement mjcf, Transform transform) {
    Vector3 fromPoint, toPoint;
    if (ParseFromToMjcf(mjcf, out fromPoint, out toPoint)) {
      transform.localPosition = Vector3.Lerp(toPoint, fromPoint, 0.5f);
      transform.localRotation = Quaternion.FromToRotation(
          Vector3.up, (toPoint - fromPoint).normalized);
    } else {
      transform.localPosition = MjEngineTool.UnityVector3(
          mjcf.GetVector3Attribute("pos", defaultValue: Vector3.zero));
      transform.localRotation = Quaternion.identity;

      if (mjcf.HasAttribute("quat")) {
        transform.localRotation = MjEngineTool.UnityQuaternion(
            mjcf.GetQuaternionAttribute("quat",
                                        defaultValue: MjEngineTool.MjQuaternionIdentity));
      } else if (mjcf.HasAttribute("zaxis")) {
        var upAxis = MjEngineTool.UnityVector3(
            mjcf.GetVector3Attribute("zaxis", defaultValue: MjEngineTool.MjVector3Up));
        transform.localRotation = Quaternion.FromToRotation(
            Vector3.up, upAxis);
      } else if (mjcf.HasAttribute("xyaxes")) {
        var xyaxes = mjcf.GetFloatArrayAttribute(
            "xyaxes", defaultValue: new float[] { 1, 0, 0, 0, 1, 0 });
        var xAxis = MjEngineTool.UnityVector3(xyaxes, 0);
        var yAxis = MjEngineTool.UnityVector3(xyaxes, 1);
        var zAxis = Vector3.Cross(xAxis, yAxis);
        transform.localRotation = Quaternion.LookRotation(zAxis, yAxis);
      } else if (mjcf.HasAttribute("axisangle")) {
        var axisAngle = mjcf.GetFloatArrayAttribute(
            "axisangle", defaultValue: new float[] { 0, 0, 1, 0 });
        var axis = MjEngineTool.UnityVector3(axisAngle, 0);
        var angle = axisAngle[3];
        if (MjSceneImportSettings.AnglesInDegrees == false) {
          angle *= Mathf.Rad2Deg;
        }
        transform.localRotation = Quaternion.AngleAxis(angle, axis);
      } else if (mjcf.HasAttribute("euler")) {
        var euler = MjEngineTool.UnityEuler(
            mjcf.GetVector3Attribute("euler", defaultValue: Vector3.zero));
        if (MjSceneImportSettings.AnglesInDegrees == false) {
          euler = euler * Mathf.Rad2Deg;
        }
        transform.localRotation = Quaternion.Euler(euler);
      }
    }
  }

  public static void LoadPlugins() {
    if(!Directory.Exists("Packages/org.mujoco/Runtime/Plugins/")) return;
    foreach (string pluginPath in Directory.GetFiles("Packages/org.mujoco/Runtime/Plugins/")) {
      MjbModel.LoadPluginLibrary(pluginPath);
    }
  }

}

public static class MjSceneImportSettings {
  public static bool AnglesInDegrees = true;
}
}
