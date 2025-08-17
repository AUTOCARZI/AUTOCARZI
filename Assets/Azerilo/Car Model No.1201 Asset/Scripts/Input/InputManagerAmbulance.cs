using UnityEngine;
using UnityEngine.InputSystem;

public class InputManagerAmbulance : MonoBehaviour
{
    public float throttle;
    public float steer;
    public bool l;
    public bool brake;
    public bool siren;
    public bool horn;

    [Header("Auto Mode for Scenario")]
    public bool isAutoMode = false;
    public float autoThrottle = 0f;
    public bool autoSiren = false;


    void Update()
    {
        if (isAutoMode)
        {
            throttle = autoThrottle;
            steer = 0f; // 직진
            brake = false;
            siren = autoSiren;
            horn = false;
            return;
        }

        Keyboard keyboard = Keyboard.current;
        if (keyboard != null)
        {
            throttle = 0;
            steer = 0;
            horn = false;
            if (keyboard.wKey.isPressed)
                throttle = 1f;
            else if (keyboard.sKey.isPressed)
                throttle = -1f;
            if (keyboard.aKey.isPressed)
                steer = -1f;
            else if (keyboard.dKey.isPressed)
                steer = 1f;
            l = keyboard.lKey.wasPressedThisFrame;
            brake = keyboard.bKey.isPressed;
            if (keyboard.zKey.wasPressedThisFrame)
                siren = !siren;
            horn = keyboard.hKey.isPressed;
        }
    }

    public void SetAutoMode(bool autoMode, float throttleValue = 0f, bool sirenOn = false)
    {
        isAutoMode = autoMode;
        autoThrottle = throttleValue;
        autoSiren = sirenOn;
    }
}