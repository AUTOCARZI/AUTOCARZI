using System.Collections.Generic;
using UnityEngine;

public class LEDManager : MonoBehaviour
{
    [Header("LED Settings")]
    public Transform rightLEDParent;
    public Transform leftLEDParent;

    private LEDNode[] allLEDs;
    private bool isCurrentlyBlinking = false; // 현재 깜빡임 상태 추적

    void Start()
    {
        Debug.Log("[LEDManager] LEDManager Starting...");
        FindAllLEDs();

        Debug.Log("[LEDManager] Subscribing to LEDControlEvent...");
        EventManager.Subscribe<LEDControlEvent>(OnLEDControl);
        Debug.Log("[LEDManager] LEDManager initialization complete");
    }

    void OnDestroy()
    {
        Debug.Log("[LEDManager] LEDManager OnDestroy - Unsubscribing events");
        EventManager.Unsubscribe<LEDControlEvent>(OnLEDControl);
    }

    private void OnLEDControl(LEDControlEvent ledEvent)
    {
        Debug.Log($"[LEDManager] Received LEDControlEvent: Blink={ledEvent.shouldBlink}, Speed={ledEvent.blinkSpeed:F2}, TimerSpeed={ledEvent.timerSpeed:F2}, Volume={ledEvent.volume:F2}, Direction={ledEvent.direction}");

        if (ledEvent.shouldBlink)
        {
            if (!isCurrentlyBlinking) // 이미 깜빡이고 있지 않을 때만 시작
            {
                Debug.Log($"[LEDManager] Starting LED chained blinking for direction: {ledEvent.direction}");
                SetupChainedBlinking(ledEvent.blinkSpeed, ledEvent.timerSpeed, ledEvent.volume, ledEvent.direction);
                isCurrentlyBlinking = true;
            }
            else
            {
                Debug.Log($"[LEDManager] Already blinking - updating parameters");
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

        Debug.Log($"[LEDManager] LEDs chained blinking setup complete! Direction: {direction}, Volume: {volume:F2}");
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
            Debug.LogWarning("[LEDManager] ⚠️ No LEDs to stop!");
            return;
        }

        Debug.Log($"[LEDManager] Force stopping {allLEDs.Length} LEDs");

        foreach (LEDNode led in allLEDs)
        {
            // 완전히 리셋
            led.isFirstNode = false;
            led.prevNode = null;

            // 정상 상태로 복원 (매우 긴 offTime으로 사실상 정지)
            led.SetTimingSettings(0.1f, 999f, 1.0f, 1.0f);

            // LED가 켜져있다면 강제로 끄기 (LEDNode에 이런 메서드가 있다면)
            // led.ForceOff(); // 만약 LEDNode에 이런 메서드가 있다면 사용
        }

        Debug.Log("[LEDManager] All LEDs force stopped and reset to normal state");
    }

    // 현재 깜빡임 상태 확인용 (디버깅)
    public bool IsBlinking() => isCurrentlyBlinking;

    // 강제 정지용 public 메서드 (외부에서 호출 가능)
    public void ForceStopBlinking()
    {
        Debug.Log("[LEDManager] External force stop requested");
        ForceStopAllLEDs();
        isCurrentlyBlinking = false;
    }
}