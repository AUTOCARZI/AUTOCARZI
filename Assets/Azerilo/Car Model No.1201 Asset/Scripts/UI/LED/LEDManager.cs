using System.Collections.Generic;
using UnityEngine;

public class LEDManager : MonoBehaviour
{
    [Header("LED Settings")]
    public Transform rightLEDParent;
    public Transform leftLEDParent;

    [Header("Sound Type Colors")]
    public Color LEDColor = Color.blue;
    public Color defaultColor = Color.black;

    private LEDNode[] allLEDs;
    private bool isCurrentlyBlinking = false;
    private Color currentLEDColor = Color.white;

    // SoundType별 색상 매핑
    private Dictionary<SoundType, Color> soundColorMap;

    void Start()
    {
        Debug.Log("[LEDManager] LEDManager Starting...");
        
        // 색상 매핑 초기화
        InitializeColorMapping();
        
        FindAllLEDs();
        SetAllLEDsColor(defaultColor);

        Debug.Log("[LEDManager] Subscribing to LEDControlEvent...");
        EventManager.Subscribe<LEDControlEvent>(OnLEDControl);
        Debug.Log("[LEDManager] LEDManager initialization complete");
    }

    void OnDestroy()
    {
        Debug.Log("[LEDManager] LEDManager OnDestroy - Unsubscribing events");
        EventManager.Unsubscribe<LEDControlEvent>(OnLEDControl);
    }

    private void InitializeColorMapping()
    {
        soundColorMap = new Dictionary<SoundType, Color>
        {
            [SoundType.None] = defaultColor,
            [SoundType.Default] = LEDColor,
            [SoundType.Ambulance] = LEDColor,
            [SoundType.CarHorn] = LEDColor,
            [SoundType.PoliceWhistle] = LEDColor,
        };

        Debug.Log($"[LEDManager] Initialized {soundColorMap.Count} sound type colors");
    }

    private void OnLEDControl(LEDControlEvent ledEvent)
    {
        Debug.Log($"[LEDManager] Received LEDControlEvent: Blink={ledEvent.shouldBlink}, Speed={ledEvent.blinkSpeed:F2}, TimerSpeed={ledEvent.timerSpeed:F2}, Volume={ledEvent.volume:F2}, Direction={ledEvent.direction}, SoundType={ledEvent.soundType}, Color={ledEvent.ledColor}");

        // 색상 설정 (SoundType 또는 직접 색상)
        Color targetColor = defaultColor;
        
        targetColor = soundColorMap[ledEvent.soundType];
        Debug.Log($"[LEDManager] Using {ledEvent.soundType} color: {targetColor}");

        SetAllLEDsColor(targetColor);

        if (ledEvent.shouldBlink)
        {
            if (!isCurrentlyBlinking)
            {
                Debug.Log($"[LEDManager] Starting LED chained blinking for direction: {ledEvent.direction} with color: {targetColor}");
                SetupChainedBlinking(ledEvent.blinkSpeed, ledEvent.timerSpeed, ledEvent.volume, ledEvent.direction);
                isCurrentlyBlinking = true;
            }
            else
            {
                Debug.Log($"[LEDManager] Already blinking - updating parameters and color");
                UpdateBlinkingParameters(ledEvent.blinkSpeed, ledEvent.timerSpeed);
            }
        }
        else
        {
            Debug.Log($"[LEDManager] Stopping LED blinking - resetting to normal state");
            ForceStopAllLEDs();
            isCurrentlyBlinking = false;
        }
    }

    void FindAllLEDs()
    {
        List<LEDNode> allLEDsList = new List<LEDNode>();

        if (rightLEDParent != null)
        {
            LEDNode[] rightLEDs = rightLEDParent.GetComponentsInChildren<LEDNode>();
            allLEDsList.AddRange(rightLEDs);
            Debug.Log($"[LEDManager] Found {rightLEDs.Length} RIGHT LED nodes");
        }

        if (leftLEDParent != null)
        {
            LEDNode[] leftLEDs = leftLEDParent.GetComponentsInChildren<LEDNode>();
            allLEDsList.AddRange(leftLEDs);
            Debug.Log($"[LEDManager] Found {leftLEDs.Length} LEFT LED nodes");
        }

        allLEDs = allLEDsList.ToArray();
        Debug.Log($"[LEDManager] Total LEDs found: {allLEDs.Length}");
    }

    private void SetupChainedBlinking(float blinkSpeed, float timerSpeed, float volume, string direction)
    {
        if (allLEDs == null || allLEDs.Length == 0)
        {
            Debug.LogWarning("[LEDManager] No LEDs found to setup blinking!");
            return;
        }

        Debug.Log($"[LEDManager] Setting up {allLEDs.Length} LEDs for chained blinking");

        // Initialize all LEDs
        foreach (LEDNode led in allLEDs)
        {
            led.isFirstNode = false;
            led.prevNode = null;
            led.SetTimingSettings(0.8f, blinkSpeed, timerSpeed, timerSpeed);
            
            // LED에 색상 설정 메서드가 있다면 호출
            // led.SetColor(currentLEDColor);
        }

        // Setup chain
        for (int i = 0; i < allLEDs.Length; i++)
        {
            if (i == 0)
            {
                allLEDs[i].isFirstNode = true;
                allLEDs[i].prevNode = null;
            }
            else
            {
                allLEDs[i].isFirstNode = false;
                allLEDs[i].prevNode = allLEDs[i - 1];
            }
        }

        Debug.Log($"[LEDManager] LEDs chained blinking setup complete! Direction: {direction}, Volume: {volume:F2}, Color: {currentLEDColor}");
    }

    private void UpdateBlinkingParameters(float blinkSpeed, float timerSpeed)
    {
        if (allLEDs == null) return;

        foreach (LEDNode led in allLEDs)
        {
            led.SetTimingSettings(0.8f, blinkSpeed, timerSpeed, timerSpeed);
        }

        Debug.Log($"[LEDManager] Updated blinking parameters: Speed={blinkSpeed:F2}, Timer={timerSpeed:F2}");
    }

    private void ForceStopAllLEDs()
    {
        if (allLEDs == null)
        {
            Debug.LogWarning("[LEDManager] No LEDs to stop!");
            return;
        }

        Debug.Log($"[LEDManager] Force stopping {allLEDs.Length} LEDs");

        foreach (LEDNode led in allLEDs)
        {
            led.isFirstNode = false;
            led.prevNode = null;
            led.SetTimingSettings(0.1f, 999f, 1.0f, 1.0f);
        }

        Debug.Log("[LEDManager] All LEDs force stopped and reset to normal state");
    }

    private void SetAllLEDsColor(Color color)
    {
        currentLEDColor = color;

        if (allLEDs == null) return;

        foreach (LEDNode led in allLEDs)
        {
            // LEDNode에 색상 설정 메서드가 있다고 가정
            // 실제 구현은 LEDNode의 구조에 따라 다를 수 있습니다
            
            // 방법 1: LEDNode에 SetColor 메서드가 있는 경우
            // led.SetColor(color);
            
            // 방법 2: LEDNode가 Light 컴포넌트를 가지고 있는 경우
            Light ledLight = led.GetComponent<Light>();
            if (ledLight != null)
            {
                ledLight.color = color;
            }
            
            // 방법 3: LEDNode가 Renderer를 가지고 있는 경우 (Material 색상 변경)
            Renderer ledRenderer = led.GetComponent<Renderer>();
            if (ledRenderer != null && ledRenderer.material != null)
            {
                ledRenderer.material.color = color;
                
                // Emissive Material인 경우
                if (ledRenderer.material.HasProperty("_EmissionColor"))
                {
                    ledRenderer.material.SetColor("_EmissionColor", color);
                }
            }
        }

        Debug.Log($"[LEDManager] Set all LEDs color to: {color}");
    }

    // Public API methods
    public void SetLEDColorForSoundType(SoundType soundType)
    {
        if (soundColorMap.ContainsKey(soundType))
        {
            SetAllLEDsColor(soundColorMap[soundType]);
            Debug.Log($"[LEDManager] Set LEDs to {soundType} color: {soundColorMap[soundType]}");
        }
        else
        {
            SetAllLEDsColor(defaultColor);
            Debug.Log($"[LEDManager] Unknown sound type {soundType}, using default color");
        }
    }

    public void SetCustomSoundTypeColor(SoundType soundType, Color color)
    {
        soundColorMap[soundType] = color;
        Debug.Log($"[LEDManager] Updated {soundType} color to: {color}");
    }

    public Color GetSoundTypeColor(SoundType soundType)
    {
        return soundColorMap.ContainsKey(soundType) ? soundColorMap[soundType] : defaultColor;
    }

    public bool IsBlinking() => isCurrentlyBlinking;

    public void ForceStopBlinking()
    {
        Debug.Log("[LEDManager] External force stop requested");
        ForceStopAllLEDs();
        isCurrentlyBlinking = false;
    }
}