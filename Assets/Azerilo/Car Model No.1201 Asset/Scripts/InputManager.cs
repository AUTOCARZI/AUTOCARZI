using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour {
    public float throttle;
    public float steer;
    public bool l;
    public bool brake;

    // Update is called once per frame
    void Update()
    {
        // Using the new Input System with Keyboard class
        Keyboard keyboard = Keyboard.current;

        if (keyboard != null)
        {
            // Reset values
            throttle = 0;
            steer = 0;

            // Check throttle (forward/backward)
            if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed)
                throttle = 1f;
            else if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed)
                throttle = -1f;

            // Check steering (left/right)
            if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed)
                steer = -1f;
            else if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed)
                steer = 1f;

            // Check L key for headlights
            l = keyboard.lKey.wasPressedThisFrame;

            // Check brake key (Space)
            brake = keyboard.spaceKey.isPressed;
        }
    }
}