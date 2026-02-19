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
using System.Xml;
using UnityEngine;
using Mujoco.Mjb;

namespace Mujoco {

[DisallowMultipleComponent]
public abstract class MjComponent : MonoBehaviour {

  public string MujocoName { get; private set; }
  public int MujocoId { get; protected set; }

  public abstract mjtObj ObjectType { get; }

  protected virtual bool _suppressNameAttribute => false;

  public void BindToRuntime(MjbModel model, MjbData data) {
    MujocoId = model.Name2Id((int)ObjectType, MujocoName);
    if (MujocoId == -1 && !_suppressNameAttribute) {
      throw new NullReferenceException($"element name {MujocoName} not found");
    }
    OnBindToRuntime(model, data);
  }

  public XmlElement GenerateMjcf(string name, XmlDocument doc) {
    MujocoName = name;
    var mjcf = OnGenerateMjcf(doc);
    if (!_suppressNameAttribute) {
      mjcf.SetAttribute("name", name);
    }
    return mjcf;
  }

  public void ParseMjcf(XmlElement mjcf) {
    OnParseMjcf(mjcf);
  }

  protected abstract void OnParseMjcf(XmlElement mjcf);
  protected abstract XmlElement OnGenerateMjcf(XmlDocument doc);

  protected virtual void OnBindToRuntime(MjbModel model, MjbData data) {}

  public virtual void OnSyncState(MjbData data) {}

  private bool _sceneExcludesMe = false;

  protected virtual void OnEnable() {
    if (MjScene.Instance == null) {
      throw new Exception("MuJoCo Scene not found");
    }
    if (MjScene.Instance.Model != null) {
      _sceneExcludesMe = true;
    }
  }

  protected void Update() {
    if (_sceneExcludesMe) {
      MjScene.Instance.SceneRecreationAtLateUpdateRequested = true;
      _sceneExcludesMe = false;
    }
  }

  private bool _exiting = false;
  public void OnApplicationQuit() {
    _exiting = true;
  }

  public void OnDisable() {
    if (!_exiting && MjScene.InstanceExists) {
      MjScene.Instance.SceneRecreationAtLateUpdateRequested = true;
    }
  }

}
}
