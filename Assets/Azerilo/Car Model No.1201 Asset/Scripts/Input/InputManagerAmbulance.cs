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


    void Update()
    {

        Keyboard keyboard = Keyboard.current;
        if (keyboard != null)
        {
            // Reset values
            throttle = 0;
            steer = 0;
            horn = false;

            // Check throttle (forward/backward)
            if (keyboard.wKey.isPressed)
                throttle = 1f;
            else if (keyboard.sKey.isPressed)
                throttle = -1f;

            // Check steering (left/right)
            if (keyboard.aKey.isPressed)
                steer = -1f;
            else if (keyboard.dKey.isPressed)
                steer = 1f;

            // Check L key for headlights
            l = keyboard.lKey.wasPressedThisFrame;

            // Check brake key (B)
            brake = keyboard.bKey.isPressed;

            // Check Z key for siren toggle
            if (keyboard.zKey.wasPressedThisFrame)
                siren = !siren;

            // Check H key for horn
            horn = keyboard.hKey.isPressed;
        }
    }
}