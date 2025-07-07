using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(InputManager))]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(LightingManager))]
public class CarController : MonoBehaviour
{
    public InputManager im;
    public LightingManager lm;
    private HUDManager hm;
    public List<WheelCollider> throttleWheels;
    public List<GameObject> steeringWheels;
    public List<GameObject> meshes;
    public float strengthCoefficient = 10000f;
    public float maxTurn = 20f;
    public Transform CM;
    public Rigidbody rb;
    public float brakeStrength;

    [Header("Audio Detection")]
    public CarControllerAmbulance ambulanceToDetect;
    public float ambulanceVolumeThreshold = 0.3f;

    [Header("LED Warning System")]
    public Transform rightLEDParent;
    public Transform leftLEDParent;

    [Header("Direction-based Blink Thresholds")]
    public float behindBlinkThreshold = 0.90f;    // 뒤쪽 (behind, behind-left, behind-right)
    public float sideBlinkThreshold = 0.90f;     // 옆쪽 (to the left, to the right)
    public float frontBlinkThreshold = 0.95f;    // 앞쪽 (ahead, ahead-left, ahead-right)

    [Header("HUD System")]
    public Canvas hudCanvas;
    public UnityEngine.UI.RawImage hudAmbulanceLeft;
    public UnityEngine.UI.RawImage hudAmbulanceRight;
    public float hudBlinkInterval = 1f;
    public float hudFadeSpeed = 2f;

    private float currentAmbulanceVolume;
    private Vector3 ambulanceDirection;
    private float ambulanceDistance;
    private string ambulanceRelativeDirection;
    private LEDNode[] allLEDs;
    private float currentBlinkThreshold; // 현재 방향에 따른 임계값

    void Start()
    {
        im = GetComponent<InputManager>();
        rb = GetComponent<Rigidbody>();
        if (CM)
        {
            rb.centerOfMass = CM.position;
        }

        if (ambulanceToDetect == null)
        {
            ambulanceToDetect = FindObjectOfType<CarControllerAmbulance>();
        }

        FindAllLEDs();
        InitializeHUDSystem();
    }

    void InitializeHUDSystem()
    {
        if (hudCanvas != null)
        {
            hm = new HUDManager(hudCanvas, hudBlinkInterval, hudFadeSpeed);

            // 앰뷸런스 상황 HUD 등록
            hm.RegisterHUD("ambulance-left", hudAmbulanceLeft);
            hm.RegisterHUD("ambulance-right", hudAmbulanceRight);

            Debug.Log("HUD System initialized");
        }
    }

    void FindAllLEDs()
    {
        List<LEDNode> allLEDsList = new List<LEDNode>();

        if (rightLEDParent != null)
        {
            LEDNode[] rightLEDs = rightLEDParent.GetComponentsInChildren<LEDNode>();
            allLEDsList.AddRange(rightLEDs);
            Debug.Log($"Found {rightLEDs.Length} RIGHT LED nodes");
        }

        if (leftLEDParent != null)
        {
            LEDNode[] leftLEDs = leftLEDParent.GetComponentsInChildren<LEDNode>();
            allLEDsList.AddRange(leftLEDs);
            Debug.Log($"Found {leftLEDs.Length} LEFT LED nodes");
        }

        allLEDs = allLEDsList.ToArray();
        Debug.Log($"Total LEDs found: {allLEDs.Length}");
    }

    void Update()
    {
        if (im.l)
        {
            lm.ToggleHeadlights();
        }

        CheckAmbulanceAudio();
        UpdateWarningLEDs();

        // HUD 업데이트
        if (hm != null)
        {
            UpdateHUDSystem();
        }
    }

    void UpdateHUDSystem()
    {
        // 앰뷸런스 상황 체크
        CheckAmbulanceSituation();
    }

    void CheckAmbulanceSituation()
    {
        bool shouldShowAmbulanceHUD = currentAmbulanceVolume > ambulanceVolumeThreshold;

        if (shouldShowAmbulanceHUD)
        {
            switch (ambulanceRelativeDirection)
            {
                case "behind-left":
                    hm.StartBlinking("ambulance-left");
                    hm.StopBlinking("ambulance-right");
                    break;
                case "behind-right":
                    hm.StartBlinking("ambulance-right");
                    hm.StopBlinking("ambulance-left");
                    break;
                default:
                    hm.StopBlinking("ambulance-left");
                    hm.StopBlinking("ambulance-right");
                    break;
            }
        }
        else
        {
            hm.StopBlinking("ambulance-left");
            hm.StopBlinking("ambulance-right");
        }
    }

    void CheckAmbulanceAudio()
    {
        if (ambulanceToDetect != null)
        {
            currentAmbulanceVolume = ambulanceToDetect.GetPerceivedVolumeFrom(this.transform);

            Vector3 toAmbulance = ambulanceToDetect.transform.position - this.transform.position;
            ambulanceDistance = toAmbulance.magnitude;
            ambulanceDirection = toAmbulance.normalized;

            ambulanceRelativeDirection = GetRelativeDirection(ambulanceDirection);

            // 방향에 따른 임계값 설정
            currentBlinkThreshold = GetBlinkThresholdForDirection(ambulanceRelativeDirection);

            if (currentAmbulanceVolume > ambulanceVolumeThreshold)
            {
                Debug.Log($"Ambulance {ambulanceRelativeDirection}! Volume: {currentAmbulanceVolume:F2}, Blink Threshold: {currentBlinkThreshold:F2}, Distance: {ambulanceDistance:F1}m");
                ReactToAmbulanceDirection(ambulanceRelativeDirection);
            }
        }
    }

    float GetBlinkThresholdForDirection(string direction)
    {
        switch (direction)
        {
            // 뒤쪽 - 0.90 임계값
            case "behind":
            case "behind-left":
            case "behind-right":
                return behindBlinkThreshold;

            // 옆쪽 - 0.90 임계값
            case "to the left":
            case "to the right":
                return sideBlinkThreshold;

            // 앞쪽 - 0.95 임계값
            case "ahead":
            case "ahead-left":
            case "ahead-right":
                return frontBlinkThreshold;

            default:
                return frontBlinkThreshold; // 기본값은 앞쪽과 동일
        }
    }

    void UpdateWarningLEDs()
    {
        if (allLEDs != null && allLEDs.Length > 0)
        {
            if (currentAmbulanceVolume >= currentBlinkThreshold)
            {
                // Calculate blink speed based on volume
                float volumeRatio = (currentAmbulanceVolume - currentBlinkThreshold) / (1.0f - currentBlinkThreshold);
                volumeRatio = Mathf.Clamp01(volumeRatio);

                float minOffTime = 0.5f;
                float maxOffTime = 2.0f;
                float blinkSpeed = Mathf.Lerp(maxOffTime, minOffTime, volumeRatio);

                float minTimerSpeed = 2.0f;
                float maxTimerSpeed = 8.0f;
                float timerSpeed = Mathf.Lerp(minTimerSpeed, maxTimerSpeed, volumeRatio);

                // 모든 LED 초기화
                foreach (LEDNode led in allLEDs)
                {
                    led.isFirstNode = false;
                    led.prevNode = null;

                    led.SetTimingSettings(
                        0.8f,        // onTime
                        blinkSpeed,  // offTime
                        timerSpeed,  // onTimerSpeed
                        timerSpeed   // offTimerSpeed
                    );
                }

                // 체인 연결: 각 LED가 이전 LED를 참조하도록 설정
                for (int i = 0; i < allLEDs.Length; i++)
                {
                    if (i == 0)
                    {
                        // 첫 번째 LED만 firstNode로 설정
                        allLEDs[i].isFirstNode = true;
                        allLEDs[i].prevNode = null;
                    }
                    else
                    {
                        // 나머지 LED들은 이전 LED를 참조
                        allLEDs[i].isFirstNode = false;
                        allLEDs[i].prevNode = allLEDs[i - 1];
                    }
                }

                Debug.Log($"LEDs chained blinking! Direction: {ambulanceRelativeDirection}, Volume: {currentAmbulanceVolume:F2}");
            }
            else
            {
                // Volume below threshold - 모든 LED 정상상태로
                foreach (LEDNode led in allLEDs)
                {
                    led.SetTimingSettings(0.1f, 10.0f, 1.0f, 1.0f);
                    led.isFirstNode = false;
                    led.prevNode = null;
                }
            }
        }
    }

    string GetRelativeDirection(Vector3 directionToAmbulance)
    {
        Vector3 localDirection = transform.InverseTransformDirection(directionToAmbulance);
        float angle = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;

        if (angle < 0) angle += 360f;

        if (angle >= 337.5f || angle < 22.5f) return "ahead";
        else if (angle >= 22.5f && angle < 67.5f) return "ahead-right";
        else if (angle >= 67.5f && angle < 112.5f) return "to the right";
        else if (angle >= 112.5f && angle < 157.5f) return "behind-right";
        else if (angle >= 157.5f && angle < 202.5f) return "behind";
        else if (angle >= 202.5f && angle < 247.5f) return "behind-left";
        else if (angle >= 247.5f && angle < 292.5f) return "to the left";
        else return "ahead-left";
    }

    void ReactToAmbulanceDirection(string direction)
    {
        switch (direction)
        {
            case "ahead":
                Debug.Log("Ambulance is directly ahead - should slow down and move aside!");
                break;
            case "ahead-right":
                Debug.Log("Ambulance is ahead-right - should move left!");
                break;
            case "to the right":
                Debug.Log("Ambulance is to the right - should move left!");
                break;
            case "behind-right":
                Debug.Log("Ambulance is behind-right - should move left and let it pass!");
                break;
            case "behind":
                Debug.Log("Ambulance is directly behind - should move aside!");
                break;
            case "behind-left":
                Debug.Log("Ambulance is behind-left - should move right and let it pass!");
                break;
            case "to the left":
                Debug.Log("Ambulance is to the left - should move right!");
                break;
            case "ahead-left":
                Debug.Log("Ambulance is ahead-left - should move right!");
                break;
        }
    }

    void FixedUpdate()
    {
        foreach (WheelCollider wheel in throttleWheels)
        {
            if (im.brake)
            {
                wheel.motorTorque = 0f;
                wheel.brakeTorque = brakeStrength * Time.deltaTime;
            }
            else
            {
                wheel.motorTorque = strengthCoefficient * Time.deltaTime * im.throttle;
                wheel.brakeTorque = 0f;
            }
        }
        foreach (GameObject wheel in steeringWheels)
        {
            wheel.GetComponent<WheelCollider>().steerAngle = maxTurn * im.steer;
            wheel.transform.localEulerAngles = new Vector3(0f, im.steer * maxTurn, 0f);
        }
        foreach (GameObject mesh in meshes)
        {
            mesh.transform.Rotate(rb.linearVelocity.magnitude * (transform.InverseTransformDirection(rb.linearVelocity).z >= 0 ? 1 : -1) / (2 * Mathf.PI * 0.33f), 0f, 0f);
        }
    }

    void OnDestroy()
    {
        if (hm != null)
        {
            hm.Cleanup();
        }
    }

    public float GetCurrentAmbulanceVolume() => currentAmbulanceVolume;
    public Vector3 GetAmbulanceDirection() => ambulanceDirection;
    public float GetAmbulanceDistance() => ambulanceDistance;
    public string GetAmbulanceRelativeDirection() => ambulanceRelativeDirection;
}

// HUD 관리 시스템
public class HUDManager
{
    private Canvas canvas;
    private float blinkInterval;
    private float fadeSpeed;
    private Dictionary<string, HUDElement> hudElements;
    private Dictionary<string, Coroutine> blinkingCoroutines;
    private MonoBehaviour coroutineRunner;

    public HUDManager(Canvas hudCanvas, float interval, float speed)
    {
        canvas = hudCanvas;
        blinkInterval = interval;
        fadeSpeed = speed;
        hudElements = new Dictionary<string, HUDElement>();
        blinkingCoroutines = new Dictionary<string, Coroutine>();
        coroutineRunner = hudCanvas.GetComponent<MonoBehaviour>();

        if (canvas != null)
        {
            canvas.gameObject.SetActive(true);
        }
    }

    public void RegisterHUD(string key, UnityEngine.UI.RawImage rawImage)
    {
        if (rawImage != null)
        {
            HUDElement element = new HUDElement(rawImage);
            hudElements[key] = element;

            // 초기 설정: 투명하게 만들고 비활성화
            Color color = rawImage.color;
            color.a = 0f;
            rawImage.color = color;
            rawImage.gameObject.SetActive(false);

            Debug.Log($"📱 Registered HUD: {key}");
        }
    }

    public void StartBlinking(string key)
    {
        if (!hudElements.ContainsKey(key)) return;

        // 이미 깜빡이고 있다면 중복 시작하지 않음
        if (blinkingCoroutines.ContainsKey(key) && blinkingCoroutines[key] != null) return;

        if (coroutineRunner != null)
        {
            blinkingCoroutines[key] = coroutineRunner.StartCoroutine(BlinkCoroutine(key));
        }
    }

    public void StopBlinking(string key)
    {
        if (blinkingCoroutines.ContainsKey(key) && blinkingCoroutines[key] != null)
        {
            if (coroutineRunner != null)
            {
                coroutineRunner.StopCoroutine(blinkingCoroutines[key]);
            }
            blinkingCoroutines[key] = null;
        }

        // HUD를 fade-out으로 숨기기
        if (hudElements.ContainsKey(key) && coroutineRunner != null)
        {
            coroutineRunner.StartCoroutine(FadeOut(key));
        }
    }

    public void StopAllBlinking()
    {
        foreach (string key in hudElements.Keys)
        {
            StopBlinking(key);
        }
    }

    private System.Collections.IEnumerator BlinkCoroutine(string key)
    {
        while (true)
        {
            // Fade In
            yield return FadeIn(key);

            // 표시 시간 (30%)
            yield return new UnityEngine.WaitForSeconds(blinkInterval * 0.3f);

            // Fade Out
            yield return FadeOut(key);

            // 숨김 시간 (70%)
            yield return new UnityEngine.WaitForSeconds(blinkInterval * 0.7f);
        }
    }

    private System.Collections.IEnumerator FadeIn(string key)
    {
        if (!hudElements.ContainsKey(key)) yield break;

        HUDElement element = hudElements[key];
        element.rawImage.gameObject.SetActive(true);

        Color color = element.rawImage.color;
        float startAlpha = color.a;
        float elapsed = 0f;
        float duration = 1f / fadeSpeed;

        while (elapsed < duration)
        {
            elapsed += UnityEngine.Time.deltaTime;
            color.a = UnityEngine.Mathf.Lerp(startAlpha, 1f, elapsed / duration);
            element.rawImage.color = color;
            yield return null;
        }

        color.a = 1f;
        element.rawImage.color = color;
    }

    private System.Collections.IEnumerator FadeOut(string key)
    {
        if (!hudElements.ContainsKey(key)) yield break;

        HUDElement element = hudElements[key];

        Color color = element.rawImage.color;
        float startAlpha = color.a;
        float elapsed = 0f;
        float duration = 1f / fadeSpeed;

        while (elapsed < duration)
        {
            elapsed += UnityEngine.Time.deltaTime;
            color.a = UnityEngine.Mathf.Lerp(startAlpha, 0f, elapsed / duration);
            element.rawImage.color = color;
            yield return null;
        }

        color.a = 0f;
        element.rawImage.color = color;
        element.rawImage.gameObject.SetActive(false);
    }

    public void ShowMultiple(params string[] keys)
    {
        foreach (string key in keys)
        {
            StartBlinking(key);
        }
    }

    public void Cleanup()
    {
        StopAllBlinking();
    }
}

// HUD 요소 클래스
public class HUDElement
{
    public UnityEngine.UI.RawImage rawImage;
    public bool isActive;

    public HUDElement(UnityEngine.UI.RawImage img)
    {
        rawImage = img;
        isActive = false;
    }
}