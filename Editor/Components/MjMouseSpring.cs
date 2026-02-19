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
using UnityEditor;
using UnityEngine;
using Mujoco.Mjb;

namespace Mujoco {

  // During play mode, in the scene view, with a MjBody component selected, holding down control
  // and left mouse drag will apply a force on the body.  Holding shift down will change the applied
  // force direction between World XZ plane and Y[camera-up].

  [CustomEditor(typeof(MjComponent), true)]
  public class MjMouseSpring : Editor {
    private bool _lastShiftKeyState = false;

    private Plane _mouseDragPlane;
    private Vector3 _mouseDragCurrentPoint = Vector3.negativeInfinity;

    private Color _translucentRed = new Color(1, 0, 0, 0.1f);

    public void OnDisable() {
      // Release hot control if we held it. Can't call GUIUtility.GetControlID
      // outside OnGUI, so just reset hotControl unconditionally if non-zero.
      if (GUIUtility.hotControl != 0) {
        GUIUtility.hotControl = 0;
      }
    }

    private void SetDragOriginAndDragPlane(Vector3 planeOrigin, Vector3 normal) {
      normal[1] = 0;
      normal = _lastShiftKeyState ? Vector3.up : normal;
      _mouseDragPlane.SetNormalAndPosition(normal, planeOrigin);
    }

    private void UpdatePositionOnDragPlane(Vector3 mousePosition) {
      float rayDist;
      Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);

      if (_mouseDragPlane.Raycast(ray, out rayDist)) {
        _mouseDragCurrentPoint = ray.GetPoint(rayDist);
      }
    }

    private void DrawMouseSpringGui(Vector3 bodyPosition, Vector3 direction, Camera renderCamera) {
      // Update and draw the drag indicator.
      var rot = Quaternion.FromToRotation(Vector3.up, direction);

      // Backup defaults.
      var backupColour = Handles.color;

      // Update the drag plane indicator disc and outline...
      var discRadius = (_mouseDragCurrentPoint - bodyPosition).magnitude;

      Handles.color = Color.red;
      Handles.DrawWireDisc(bodyPosition, _mouseDragPlane.normal, discRadius);
      Handles.color = _translucentRed;
      Handles.DrawSolidDisc(bodyPosition, _mouseDragPlane.normal, discRadius);

      // ...and give it a phat poly line to show the drag anchor origin.
      Handles.color = Color.white;
      Handles.DrawAAPolyLine(6, new Vector3[] { bodyPosition, _mouseDragCurrentPoint });

      // Restore defaults.
      Handles.color = backupColour;
    }

    public unsafe void OnSceneGUI() {
      if (!Application.isPlaying) {
        return;
      }

      var currentEvent = UnityEngine.Event.current;

      // Cache the hot control to determine whether we're currently capturing mouse input.
      int uniqueID = GUIUtility.GetControlID(FocusType.Passive);

      // Mouse spring is active if the control key is held down and the user is dragging the
      // left mouse button, or if we're already in the process of capturing mouse input.
      bool mouseSpringActive = GUIUtility.hotControl == uniqueID;
      bool mouseSpringStarting = currentEvent.control && currentEvent.button == 0;
      if (!mouseSpringStarting && !mouseSpringActive) {
        return;
      }

      var sceneCamera = SceneView.currentDrawingSceneView?.camera;
      if (sceneCamera == null) {
        Debug.LogError("SceneView.currentDrawingSceneView is null");
        return;
      }

      var targetObject = target as MjComponent;
      MjBody body = targetObject.GetComponentInParent<MjBody>();
      if(!body)
        return;

      Vector3 bodyPosition =
          body != null ? body.transform.position : Vector3.zero;
      var scene = MjScene.Instance;

      switch (currentEvent.type) {
        case EventType.MouseDown:
          if (EditorWindow.mouseOverWindow == UnityEditor.EditorWindow.GetWindow<SceneView>()) {
            // The mouse was pressed in the scene view, so start capturing all mouse input until
            // the mouse button's released.
            GUIUtility.hotControl = uniqueID;

            _lastShiftKeyState = false;
            SetDragOriginAndDragPlane(bodyPosition, -sceneCamera.transform.forward);

            currentEvent.Use();
          }
          return;

        case EventType.MouseDrag:
          if (mouseSpringActive) {
            // We're currently capturing all mouse input, so consume the event.
            GUIUtility.hotControl = uniqueID;
            currentEvent.Use();
          }
          return;

        case EventType.MouseUp:
          if (mouseSpringActive) {
            // We're still capturing all mouse input so consume the event, but we can release our
            // control over capturing input now as the mouse button's been released.
            GUIUtility.hotControl = 0;
            currentEvent.Use();
            // as opposed to unity's addforce, xfrc_applied is persistent
            var xfrc = scene.Data.GetXfrcApplied();
            xfrc.Data[6*body.MujocoId + 0] = 0;
            xfrc.Data[6*body.MujocoId + 1] = 0;
            xfrc.Data[6*body.MujocoId + 2] = 0;
          }
          return;

        case EventType.Repaint: {
          if (mouseSpringActive) {

            if (currentEvent.shift != _lastShiftKeyState) {
              _lastShiftKeyState = currentEvent.shift;
              SetDragOriginAndDragPlane(bodyPosition, -sceneCamera.transform.forward);
            }

            // Raycast towards the drag plane to update _mouseDragCurrentPoint.
            UpdatePositionOnDragPlane(currentEvent.mousePosition);

            float[] mjBodyVel = new float[6];
            scene.Model.ObjectVelocity(
                scene.Data, (int)mjtObj.mjOBJ_BODY, body.MujocoId, 0, mjBodyVel);
            // linear velocity is in the last 3 entries
            Vector3 bodyVel = MjEngineTool.UnityVector3(
                new Vector3(mjBodyVel[3], mjBodyVel[4], mjBodyVel[5]));

            float springStiffness = 100;
            var settings = MjGlobalSettings.Instance;
            if (settings) {
              springStiffness = settings.MouseSpringStiffness;
            }

            Vector3 delta = _mouseDragCurrentPoint - bodyPosition;
            float mass = scene.Model.BodyMass(body.MujocoId);
            Vector3 unityForce = delta * springStiffness * mass;
            unityForce -= bodyVel * Mathf.Sqrt(springStiffness) * mass;
            Vector3 mjForce = MjEngineTool.MjVector3(unityForce);
            var xfrcApplied = scene.Data.GetXfrcApplied();
            xfrcApplied.Data[6*body.MujocoId + 0] = mjForce.x;
            xfrcApplied.Data[6*body.MujocoId + 1] = mjForce.y;
            xfrcApplied.Data[6*body.MujocoId + 2] = mjForce.z;

            // Draw and immediately force a repaint.
            DrawMouseSpringGui(body.transform.position, delta, sceneCamera);
            SceneView.RepaintAll();
          }
          return;
        }
      }
    }
  }
}
