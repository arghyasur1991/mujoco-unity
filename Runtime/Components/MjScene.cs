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
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Xml;
using UnityEngine;
using UnityEngine.Profiling;
using Mujoco.Mjb;

namespace Mujoco {

public enum PhysicsBackendType { CPU, GPU }

public class PhysicsRuntimeException : Exception {
  public PhysicsRuntimeException(string message) : base(message) {}
}

public class MjScene : MonoBehaviour {

  public MjbBackend Backend { get; private set; }
  public MjbModel Model { get; private set; }
  public MjbData Data { get; private set; }

  [Header("Physics Backend")]
  [Tooltip("CPU: MuJoCo C (double, CPU). GPU: MuJoCo-MLX (float32, Metal GPU). " +
           "GPU does physics on Metal, then syncs state back to CPU for rendering.")]
  public PhysicsBackendType physicsBackend = PhysicsBackendType.GPU;

  private IMjPhysicsBackend _backend;
  private bool _isGpuBackend;
  private int _subStepsPerFixedUpdate = 1;

  public IMjPhysicsBackend PhysicsBackend => _backend;

  public MjcfGenerationContext GenerationContext {
    get {
      if (_generationContext == null) {
        throw new InvalidOperationException(
            "This property can only be accessed from the scope of MjComponent.GenerateMjcf().");
      }
      return _generationContext;
    }
  }

  public static MjScene Instance {
    get {
      if (_instance == null) {
        var instances = FindObjectsByType<MjScene>(FindObjectsSortMode.None);
        if (instances.Length == 1) {
          _instance = instances[0];
        } else if (instances.Length > 1) {
          _instance = instances[0];
          for (int i = 1; i < instances.Length; i++) {
            Destroy(instances[i].gameObject);
          }
        } else {
          GameObject go = new GameObject("MjScene");
          _instance = go.AddComponent<MjScene>();
        }
      }
      return _instance;
    }
  }

  public static bool InstanceExists { get => _instance != null; }

  public void Awake() {
    if (_instance == null) {
      _instance = this;
    } else if (_instance != this) {
      Destroy(gameObject);
    }
  }

  private static MjScene _instance = null;

  private List<MjComponent> _orderedComponents;

  public event EventHandler<MjStepArgs> postInitEvent;
  public event EventHandler<MjStepArgs> preUpdateEvent;
  public event EventHandler<MjStepArgs> ctrlCallback;
  public event EventHandler<MjStepArgs> postUpdateEvent;
  public event EventHandler<MjStepArgs> preDestroyEvent;

  protected void Start() {
    SceneRecreationAtLateUpdateRequested = false;
    CreateScene();
  }

  protected void OnDestroy() {
    DestroyScene();
  }

  public bool PauseSimulation = false;

  protected void FixedUpdate() {
    if (PauseSimulation) return;
    preUpdateEvent?.Invoke(this, new MjStepArgs(Model, Data));
    StepScene();
    postUpdateEvent?.Invoke(this, new MjStepArgs(Model, Data));
  }

  public void ResetData() {
    if (Model == null || Data == null) return;
    Data.ResetData();
    _backend?.ResetData();
    if (_isGpuBackend) {
      float[] qpos = _backend.GetQpos().ToArray();
      float[] qvel = _backend.GetQvel().ToArray();
      Data.SetQpos(qpos);
      Data.SetQvel(qvel);
    }
    Data.Forward();
    _lastWarningCounts = null;
    SyncUnityToMjState();
    postInitEvent?.Invoke(this, new MjStepArgs(Model, Data));
  }

  public bool SceneRecreationAtLateUpdateRequested = false;

  protected void LateUpdate() {
    if (SceneRecreationAtLateUpdateRequested) {
      RecreateScene();
      SceneRecreationAtLateUpdateRequested = false;
    }
  }

  private MjcfGenerationContext _generationContext;

  public XmlDocument CreateScene(bool skipCompile=false) {
    if (_generationContext != null) {
      throw new InvalidOperationException(
          "The scene is currently being generated on another thread.");
    }
    var hierarchyRoots = FindObjectsByType<MjComponent>(FindObjectsSortMode.None)
        .Where(component => MjHierarchyTool.FindParentComponent(component) == null)
        .Select(component => component.transform)
        .Distinct();
    _orderedComponents = new List<MjComponent>();
    foreach (var root in hierarchyRoots) {
      _orderedComponents.AddRange(MjHierarchyTool.LinearizeHierarchyBFS(root));
    }

    XmlDocument sceneMjcf = null;
    try {
      _generationContext = new MjcfGenerationContext();
      sceneMjcf = GenerateSceneMjcf(_orderedComponents);
    } catch (Exception e) {
      _generationContext = null;
      Debug.LogException(e);
#if UNITY_EDITOR
      UnityEditor.EditorApplication.isPlaying = false;
#else
      Application.Quit();
#endif
      throw;
    }
    _generationContext = null;

    var settings = MjGlobalSettings.Instance;
    if (settings && !string.IsNullOrEmpty(settings.DebugFileName)) {
      SaveToFile(sceneMjcf, Path.Combine(Application.temporaryCachePath, settings.DebugFileName));
    }

    if (!skipCompile) {
      CompileScene(sceneMjcf, _orderedComponents);
    }
    postInitEvent?.Invoke(this, new MjStepArgs(Model, Data));
    return sceneMjcf;
  }

  private void CompileScene(
      XmlDocument mjcf, IEnumerable<MjComponent> components) {
    Backend = MjbBackend.Create(MjbBackendType.CPU);
    Model = Backend.LoadModelFromString(mjcf.OuterXml);
    if (Model == null) {
      throw new NullReferenceException("Model loading failed, see other errors for root cause.");
    }
    Data = Model.MakeData();
    if (Data == null) {
      throw new NullReferenceException("Model loaded but MakeData failed.");
    }

    _lastWarningCounts = null;

    float mjTimestep = Model.Timestep;
    _subStepsPerFixedUpdate = Mathf.Max(1, Mathf.RoundToInt(Time.fixedDeltaTime / mjTimestep));
    if (_subStepsPerFixedUpdate > 1) {
      Debug.Log($"MjScene: sub-stepping {_subStepsPerFixedUpdate}x " +
                $"(MuJoCo dt={mjTimestep}s, Unity fixedDt={Time.fixedDeltaTime}s)");
    }

    foreach (var component in components) {
      component.BindToRuntime(Model, Data);
    }

    if (physicsBackend == PhysicsBackendType.GPU) {
      try {
        _backend = new MjGpuBackend(mjcf.OuterXml, fromString: true);
        _isGpuBackend = true;
        Debug.Log($"MjScene: GPU backend (compiled Metal pipeline). " +
                  $"nq={Model.Info.nq}, nu={Model.Info.nu}");
      } catch (Exception e) {
        Debug.LogError($"MjScene: GPU backend creation failed, falling back to CPU: {e.Message}");
        _backend = new MjCpuBackend(Model, Data);
        _isGpuBackend = false;
      }
    } else {
      _backend = new MjCpuBackend(Model, Data);
      _isGpuBackend = false;
    }
  }

  public void SyncUnityToMjState() {
    foreach (var component in _orderedComponents) {
      if (component != null && component.isActiveAndEnabled) {
        component.OnSyncState(Data);
      }
    }
  }

  public unsafe void RecreateScene() {
    var joints = FindObjectsByType<MjBaseJoint>(FindObjectsSortMode.None);
    var positions = new Dictionary<MjBaseJoint, float[]>();
    var velocities = new Dictionary<MjBaseJoint, float[]>();
    foreach (var joint in joints) {
      if (joint.QposAddress > -1) {
        int jntType = Model.JntType(joint.MujocoId);
        var qpos = Data.GetQpos();
        var qvel = Data.GetQvel();
        int qa = joint.QposAddress;
        int da = joint.DofAddress;
        switch (jntType) {
          default:
          case (int)mjtJoint.mjJNT_HINGE:
          case (int)mjtJoint.mjJNT_SLIDE:
            positions[joint] = new float[] { qpos[qa] };
            velocities[joint] = new float[] { qvel[da] };
            break;
          case (int)mjtJoint.mjJNT_BALL:
            positions[joint] = new float[] { qpos[qa], qpos[qa+1], qpos[qa+2], qpos[qa+3] };
            velocities[joint] = new float[] { qvel[da], qvel[da+1], qvel[da+2] };
            break;
          case (int)mjtJoint.mjJNT_FREE:
            positions[joint] = new float[] {
                qpos[qa], qpos[qa+1], qpos[qa+2], qpos[qa+3],
                qpos[qa+4], qpos[qa+5], qpos[qa+6] };
            velocities[joint] = new float[] {
                qvel[da], qvel[da+1], qvel[da+2],
                qvel[da+3], qvel[da+4], qvel[da+5] };
            break;
        }
      }
    }

    Data.ResetData();
    Data.Kinematics();
    SyncUnityToMjState();

    DestroyScene();
    CreateScene();

    foreach (var joint in joints) {
      try {
        var position = positions[joint];
        var velocity = velocities[joint];
        int jntType = Model.JntType(joint.MujocoId);
        int qa = joint.QposAddress;
        int da = joint.DofAddress;
        switch (jntType) {
          default:
          case (int)mjtJoint.mjJNT_HINGE:
          case (int)mjtJoint.mjJNT_SLIDE:
            Data.SetQposAt(qa, position[0]);
            Data.SetQvelAt(da, velocity[0]);
            break;
          case (int)mjtJoint.mjJNT_BALL:
            for (int i = 0; i < 4; i++) Data.SetQposAt(qa + i, position[i]);
            for (int i = 0; i < 3; i++) Data.SetQvelAt(da + i, velocity[i]);
            break;
          case (int)mjtJoint.mjJNT_FREE:
            for (int i = 0; i < 7; i++) Data.SetQposAt(qa + i, position[i]);
            for (int i = 0; i < 6; i++) Data.SetQvelAt(da + i, velocity[i]);
            break;
        }
      } catch {}
    }
    Data.Kinematics();
    SyncUnityToMjState();
  }

  public void DestroyScene() {
    preDestroyEvent?.Invoke(this, new MjStepArgs(Model, Data));
    _backend?.Dispose();
    _backend = null;
    _isGpuBackend = false;
    Data?.Dispose();
    Data = null;
    Model?.Dispose();
    Model = null;
    Backend?.Dispose();
    Backend = null;
  }

  public void StepScene() {
    if (Model == null || Data == null) {
      throw new NullReferenceException("Failed to create Mujoco runtime.");
    }
    Profiler.BeginSample("MjStep");
    Profiler.BeginSample("MjStep.mj_step");

    if (_isGpuBackend) {
      var ctrl = Data.GetCtrl();
      _backend.SetCtrl(ctrl.ToArray());
      for (int i = 0; i < _subStepsPerFixedUpdate; i++)
        _backend.Step();

      float[] qposArr = _backend.GetQpos().ToArray();
      float[] qvelArr = _backend.GetQvel().ToArray();
      Data.SetQpos(qposArr);
      Data.SetQvel(qvelArr);
      Data.Kinematics();
    } else if (ctrlCallback != null) {
      Data.Step1();
      ctrlCallback?.Invoke(this, new MjStepArgs(Model, Data));
      Data.Step2();
      for (int i = 1; i < _subStepsPerFixedUpdate; i++)
        _backend.Step();
    } else {
      for (int i = 0; i < _subStepsPerFixedUpdate; i++)
        _backend.Step();
    }
    Profiler.EndSample(); // MjStep.mj_step

    if (!_isGpuBackend)
      CheckForPhysicsException();

    Profiler.BeginSample("MjStep.OnSyncState");
    SyncUnityToMjState();
    Profiler.EndSample(); // MjStep.OnSyncState
    Profiler.EndSample(); // MjStep
  }

  private static readonly string[] _warningMessages = {
      "INERTIA: (Near-) Singular inertia matrix.",
      "CONTACTFULL: nconmax isn't sufficient.",
      "CNSTRFULL: njmax isn't sufficient.",
      "VGEOMFULL: who constructed a mjvScene?!",
      "BADQPOS: NaN/inf in qpos.",
      "BADQVEL: NaN/inf in qvel.",
      "BADQACC: NaN/inf in qacc.",
      "BADCTRL: NaN/inf in ctrl.",
  };
  private int[] _lastWarningCounts;

  private void CheckForPhysicsException() {
    if (_lastWarningCounts == null)
      _lastWarningCounts = new int[_warningMessages.Length];
    for (int i = 0; i < _warningMessages.Length; i++) {
      int count = Data.GetWarningCount(i);
      if (count > _lastWarningCounts[i]) {
        _lastWarningCounts[i] = count;
        throw new PhysicsRuntimeException(_warningMessages[i]);
      }
    }
  }

  // ── Scene MJCF generation (unchanged) ─────────────────────────────

  private XmlDocument GenerateSceneMjcf(IEnumerable<MjComponent> components) {
    var doc = new XmlDocument();
    var MjRoot = (XmlElement)doc.AppendChild(doc.CreateElement("mujoco"));

    var worldMjcf = (XmlElement)MjRoot.AppendChild(doc.CreateElement("worldbody"));
    BuildHierarchicalMjcf(
        doc,
        components.Where(component =>
            (component is MjBaseBody) ||
            (component is MjInertial) ||
            (component is MjBaseJoint) ||
            (component is MjGeom) ||
            (component is MjSite)),
        worldMjcf);

    MjRoot.AppendChild(GenerateMjcfSection(
        doc, components.Where(component => component is MjExclude), "contact"));

    MjRoot.AppendChild(GenerateMjcfSection(
        doc, components.Where(component => component is MjBaseTendon), "tendon"));

    MjRoot.AppendChild(GenerateMjcfSection(
        doc, components.Where(component => component is MjBaseConstraint), "equality"));

    MjRoot.AppendChild(
        GenerateMjcfSection(doc,
                            components.Where(component => component is MjActuator)
                                .OrderBy(component => component.transform.GetSiblingIndex()),
                            "actuator"));

    MjRoot.AppendChild(
        GenerateMjcfSection(doc,
                            components.Where(component => component is MjBaseSensor)
                                .OrderBy(component => component.transform.GetSiblingIndex()),
                            "sensor"));
    _generationContext.GenerateMjcf(MjRoot);
    return doc;
  }

  private XmlElement GenerateMjcfSection(
      XmlDocument doc, IEnumerable<MjComponent> components, string sectionName) {
    var section = doc.CreateElement(sectionName);
    foreach (var component in components) {
      var componentMjcf = component.GenerateMjcf(_generationContext.GenerateName(component), doc);
      section.AppendChild(componentMjcf);
    }
    return section;
  }

  private void BuildHierarchicalMjcf(
      XmlDocument doc, IEnumerable<MjComponent> components, XmlElement worldMjcf) {
    var associations = new Dictionary<MjComponent, XmlElement>();

    foreach (var component in components) {
      var componentMjcf = component.GenerateMjcf(
          _generationContext.GenerateName(component), doc);
      associations.Add(component, componentMjcf);
    }

    foreach (var component in components) {
      var componentMjcf = associations[component];
      var parentComponent = MjHierarchyTool.FindParentComponent(component);
      if (parentComponent != null) {
        var parentComponentMjcf = associations[parentComponent];
        parentComponentMjcf.AppendChild(componentMjcf);
      } else {
        worldMjcf.AppendChild(componentMjcf);
      }
    }
  }

  private void SaveToFile(XmlDocument document, string filePath) {
    try {
      using (var stream = File.Open(filePath, FileMode.Create)) {
        using (var writer = new XmlTextWriter(stream, new UTF8Encoding(false))) {
          writer.Formatting = Formatting.Indented;
          document.WriteContentTo(writer);
          Debug.Log($"MJCF saved to {filePath}");
        }
      }
    } catch (IOException ex) {
      Debug.LogWarning("Failed to save Xml to a file: " + ex.ToString(), this);
    }
  }
}

public class MjStepArgs : EventArgs
{
  public MjStepArgs(MjbModel model, MjbData data){
    this.model = model;
    this.data = data;
  }
  public readonly MjbModel model;
  public readonly MjbData data;
}
}
