using UnityEngine;

// A LED node emitting light based on start/stop signals from the previous node in the chain.
public class LEDNode : MonoBehaviour
{
    // Previous linked LED node.
    public LEDNode prevNode = null;
    // Start/stop signals for the next linked LED node.
    public bool nextStart { get; private set; } = false;

    // Mark in GUI if first node in the chain.
    public bool isFirstNode = false;
    // Point light enable control.
    public bool isPointLightEn = false;
    // Slow light progress indicator.
    public bool slowProgress = false;

    [Header("Light Intensity Settings")]
    [Range(0.0f, 1.0f)]
    public float minIntensity = 0.1f; // Always lit at minimum level (very dim)
    [Range(0.1f, 20.0f)]
    public float maxIntensity = 5.0f; // Much brighter when wave passes through

    [Header("Timing Settings")]
    [Range(0.1f, 5.0f)]
    public float maxOnTime = 0.5f; // How long the bright intensity lasts
    [Range(0.1f, 10.0f)]
    public float onTimerSpeed = 2.0f;
    [Range(0.1f, 10.0f)]
    public float maxOffTime = 2.0f; // Reduced for faster cycles
    [Range(1.0f, 50.0f)]
    public float offTimerSpeed = 20.0f;

    // Attached components.
    private Light pointLight = null;
    private Renderer rend = null;

    // Current emitted light intensity.
    private float intensity = 0.1f; // Default lit intensity (very dim)
    private float onTime = 0;
    private float offTime = 0;

    // Emitted light increase/decrease controls.
    private enum LightState { INCR, DECR, IDLE }
    private LightState lightState = LightState.IDLE;

    void Start()
    {
        // Init the attached components.
        pointLight = this.GetComponent<Light>();
        rend = GetComponent<Renderer>();

        // Start with very dim default lighting
        intensity = minIntensity;

        // If point light is enabled, keep it on at minimum
        if (isPointLightEn && pointLight != null)
        {
            pointLight.enabled = true;
            pointLight.intensity = minIntensity;
        }
    }

    void Update()
    {
        ResolveNodeState();
        UpdateColor();
        UpdatePointLight();
    }

    // Public method to set intensity values at runtime
    public void SetIntensityRange(float min, float max)
    {
        minIntensity = Mathf.Clamp(min, 0.0f, max);
        maxIntensity = Mathf.Clamp(max, min, 20.0f);
    }

    // Public method to set timing values at runtime
    public void SetTimingSettings(float onTime, float offTime, float onSpeed, float offSpeed)
    {
        maxOnTime = Mathf.Clamp(onTime, 0.1f, 5.0f);
        maxOffTime = Mathf.Clamp(offTime, 0.1f, 10.0f);
        onTimerSpeed = Mathf.Clamp(onSpeed, 0.1f, 10.0f);
        offTimerSpeed = Mathf.Clamp(offSpeed, 1.0f, 50.0f);
    }

    // Decides on state of fading in/out based on the input parameters.
    private void ResolveNodeState()
    {
        switch (lightState)
        {
            case LightState.INCR:
                IntensityIncrease();
                break;
            case LightState.DECR:
                IntensityDecrease();
                break;
            case LightState.IDLE:
                LightIdle();
                break;
            default:
                break;
        }
    }

    // Gradually increase intensity level up to the maximum-level defined in GUI.
    private void IntensityIncrease()
    {
        if (!slowProgress)
        {
            if (nextStart == false)
                nextStart = true;
        }

        intensity = maxIntensity; // Bright intensity during wave

        // If ON timer hasn't been reached yet
        if (onTime < maxOnTime)
            onTime += onTimerSpeed * Time.deltaTime;
        // ON timer had been reached.
        else
        {
            onTime = 0;
            // Start decreasing intensity.
            lightState = LightState.DECR;
        }
    }

    // Gradually decrease intensity level back to default.
    private void IntensityDecrease()
    {
        intensity = minIntensity; // Back to default lit state

        if (slowProgress)
        {
            if (nextStart == false)
                nextStart = true;
        }
        else
        {
            // Stop sending the start signal to the next node.
            if (nextStart == true)
                nextStart = false;
        }

        // Move to the idle (default lit) state.
        lightState = LightState.IDLE;
    }

    // Light idles at default intensity until signaled to brighten.
    private void LightIdle()
    {
        intensity = minIntensity; // Keep at default intensity

        if (slowProgress)
        {
            if (nextStart == true)
                nextStart = false;
        }

        // If there is a previous linked node
        if (prevNode != null)
        {
            if (prevNode.nextStart)
            {
                // Start increasing light intensity.
                lightState = LightState.INCR;
            }
        }
        // If first node in the chain
        else if (isFirstNode)
        {
            // If max Idle time wasn't reached yet
            if (offTime < maxOffTime)
                offTime += offTimerSpeed * Time.deltaTime;
            else
            {
                offTime = 0;
                // Start increasing light intensity.
                lightState = LightState.INCR;
                // Light-up the next node in chain.
                if (nextStart == false)
                    nextStart = true;
            }
        }
    }

    // Updates the calculated LED color and intensity level.
    private void UpdateColor()
    {
        Material mat = rend.material;
        Color baseColor = mat.color;

        // Calculate the resulting color based on the intensity.
        Color finalColor = baseColor * Mathf.LinearToGammaSpace(intensity);
        mat.SetColor("_EmissionColor", finalColor);
    }

    // Updates point light intensity to match the LED intensity
    private void UpdatePointLight()
    {
        if (isPointLightEn && pointLight != null)
        {
            pointLight.intensity = intensity;
        }
    }
}