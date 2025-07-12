using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

[RequireComponent(typeof(InputManager))]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(LightingManager))]
public class CarController : MonoBehaviour
{
    [Header("Car Physics")]
    public List<WheelCollider> throttleWheels;
    public List<GameObject> steeringWheels;
    public List<GameObject> meshes;
    public float strengthCoefficient = 10000f;
    public float maxTurn = 20f;
    public Transform CM;
    public float brakeStrength;

    [Header("Audio Detection")]
    public CarControllerAmbulance ambulanceToDetect;
    public float ambulanceVolumeThreshold = 0.3f;

    [Header("Sound Response System")]
    public SoundResponseManager soundResponseManager = new SoundResponseManager();

    private InputManager im;
    private LightingManager lm;
    private Rigidbody rb;

    // Audio detection variables
    private float currentAmbulanceVolume;
    private Vector3 ambulanceDirection;
    private float ambulanceDistance;
    private string ambulanceRelativeDirection;
    private float currentBlinkThreshold;

    // HUD 상태 관리 (중복 이벤트 방지)
    private string currentActiveHUD = "";

    // 사운드 소스 맵
    private Dictionary<SoundType, ISoundSource> soundSources;

    void Start()
    {
        im = GetComponent<InputManager>();
        lm = GetComponent<LightingManager>();
        rb = GetComponent<Rigidbody>();

        if (CM)
        {
            rb.centerOfMass = CM.position;
        }

        if (ambulanceToDetect == null)
        {
            ambulanceToDetect = FindFirstObjectByType<CarControllerAmbulance>();
        }

        InitializeSoundSystem();

        // 모든 이벤트 구독
        EventManager.Subscribe<SoundEvent>(OnSoundEventReceived);
        EventManager.Subscribe<CarInputEvent>(OnCarInputEventReceived);

        Debug.Log("[CarController] CarController initialized with Emergency Response System");
    }

    void OnDestroy()
    {
        EventManager.Unsubscribe<SoundEvent>(OnSoundEventReceived);
        EventManager.Unsubscribe<CarInputEvent>(OnCarInputEventReceived);
    }

    void Update()
    {
        // 입력 이벤트 받기
        bool headlightPressed = im.l;
        var inputEvent = new CarInputEvent(im.throttle, im.steer, im.brake, headlightPressed);
        EventManager.Publish(inputEvent);

        CheckAllAudioSources();
    }

void InitializeSoundSystem()
    {
        soundResponseManager.Initialize();

        // 사운드 소스 맵 초기화
        soundSources = new Dictionary<SoundType, ISoundSource>();

        // 앰뷸런스 프로필 등록
        if (ambulanceToDetect != null)
        {
            soundSources[SoundType.Ambulance] = new AmbulanceSoundSource(ambulanceToDetect);
        }

        // 경적 프로필 등록
        // soundSources[SoundType.CarHorn] = new CarHornSoundSource(carHornSource, carHornTransform);

        Debug.Log($"[CarController] Sound Response System initialized with {soundSources.Count} sources");

        // 모든 프로필 정보 출력
        foreach (SoundType soundType in soundResponseManager.GetRegisteredSoundTypes())
        {
            soundResponseManager.PrintProfileInfo(soundType);
        }
    }

    // 모든 사운드 소스 체크
    void CheckAllAudioSources()
    {
        foreach (var kvp in soundSources)
        {
            SoundType soundType = kvp.Key;
            ISoundSource source = kvp.Value;

            if (source.IsActive())
            {
                float volume = source.GetPerceivedVolume(this.transform);
                Vector3 sourcePosition = source.GetPosition();
                CheckSoundAndRespond(soundType, volume, sourcePosition);
            }
        }
    }

    // 사운드 체크 메서드
    private void CheckSoundAndRespond(SoundType soundType, float volume, Vector3 sourcePosition)
    {
        var profile = soundResponseManager.GetProfile(soundType);
        if (profile == null) return;

        if (volume > profile.activationThreshold)
        {
            Vector3 toSource = sourcePosition - this.transform.position;
            float distance = toSource.magnitude;
            Vector3 direction = toSource.normalized;
            string relativeDirection = GetRelativeDirection(direction);

            var soundDirection = ConvertToSoundDirection(relativeDirection);
            var soundEvent = new SoundEvent(
                soundDirection,
                distance,
                volume,
                soundType,
                sourcePosition,
                this.transform.position,
                soundResponseManager.GetDirectionThreshold(soundType, relativeDirection)
            );

            EventManager.Publish(soundEvent);
            Debug.Log($"[CarController] {soundType} detected: Volume={volume:F2}, Direction={relativeDirection}");
        }
        else
        {
            StopAllEmergencyEffects();
        }
    }

    private void StopAllEmergencyEffects()
    {
        // LED 중지
        var ledStopEvent = new LEDControlEvent(0f, 0f, false, 0f, "");
        EventManager.Publish(ledStopEvent);

        // HUD 중지
        if (!string.IsNullOrEmpty(currentActiveHUD))
        {
            EventManager.Publish(new HUDControlEvent(currentActiveHUD, false));
            currentActiveHUD = "";
        }

        Debug.Log("[CarController] All emergency effects stopped - volume below threshold");
    }

    // 이벤트 핸들러
    private void OnSoundEventReceived(SoundEvent soundEvent)
    {       
        RouteToHUD(soundEvent);
        RouteToLED(soundEvent);
    }

    private void OnCarInputEventReceived(CarInputEvent inputEvent)
    {
        RouteToMovement(inputEvent);

        if (inputEvent.headlights)
        {
            lm.ToggleHeadlights();
        }
    }

    private void RouteToHUD(SoundEvent soundEvent)
    {
        string directionStr = ConvertToDirectionString(soundEvent.direction);
        var volumeLevel = soundResponseManager.GetVolumeLevel(soundEvent.soundType, soundEvent.volume);

        if (volumeLevel == null)
        {
            Debug.LogWarning($"[CarController] No volume level found for {soundEvent.soundType}");
            return;
        }

        Debug.Log($"[CarController] Sound Response: {soundEvent.soundType} - {volumeLevel.name}");
        Debug.Log($"[CarController] RouteToHUD Debug:");
        Debug.Log($"  - Sound Type: {soundEvent.soundType}");
        Debug.Log($"  - Direction: {directionStr}");
        Debug.Log($"  - Volume: {soundEvent.volume:F2}");
        Debug.Log($"  - Volume Level: {volumeLevel.name}");
        Debug.Log($"  - Should Show HUD: {volumeLevel.showHUD}");

        // 대상 HUD 결정 -> 고치기
        string targetHUD = "";
        if (volumeLevel.showHUD)
        {
            string[] targetHUDs = soundResponseManager.GetHUDsForDirection(soundEvent.soundType, directionStr);
            if (targetHUDs.Length > 0)
            {
                targetHUD = targetHUDs[0]; // 첫 번째 HUD 사용
            }
        }

        // 상태가 변경될 때만 이벤트 발행
        if (currentActiveHUD != targetHUD)
        {
            // 이전 HUD 끄기
            if (!string.IsNullOrEmpty(currentActiveHUD))
            {
                EventManager.Publish(new HUDControlEvent(currentActiveHUD, false));
                Debug.Log($"[CarController] Deactivating HUD: {currentActiveHUD}");
            }

            // 새 HUD 켜기
            if (!string.IsNullOrEmpty(targetHUD))
            {
                EventManager.Publish(new HUDControlEvent(targetHUD, true, directionStr));
                Debug.Log($"[CarController] Activating HUD: {targetHUD} for {soundEvent.soundType}");
            }

            currentActiveHUD = targetHUD;
            Debug.Log($"[CarController] HUD state changed to: {targetHUD}");
        }
    }

    // 모듈화된 LED 라우팅
    private void RouteToLED(SoundEvent soundEvent)
    {
        string directionStr = ConvertToDirectionString(soundEvent.direction);
        var volumeLevel = soundResponseManager.GetVolumeLevel(soundEvent.soundType, soundEvent.volume);
        var profile = soundResponseManager.GetProfile(soundEvent.soundType);

        if (volumeLevel == null || profile == null)
        {
            Debug.LogWarning($"[CarController] Missing profile or volume level for {soundEvent.soundType}");
            return;
        }

        float directionThreshold = soundResponseManager.GetDirectionThreshold(soundEvent.soundType, directionStr);
        bool shouldBlink = volumeLevel.activateLED && soundEvent.volume >= directionThreshold;

        Debug.Log($"[CarController] RouteToLED Debug:");
        Debug.Log($"  - Sound Type: {soundEvent.soundType}");
        Debug.Log($"  - Direction: {directionStr}");
        Debug.Log($"  - Volume: {soundEvent.volume:F2}");
        Debug.Log($"  - Direction Threshold: {directionThreshold:F2}");
        Debug.Log($"  - Volume Level: {volumeLevel.name}");
        Debug.Log($"  - Should Blink: {shouldBlink}");

        if (shouldBlink)
        {
            // 프로필에서 깜빡임 설정 가져오기
            float blinkSpeed = profile.ledBlinkSpeed;
            float timerSpeed = profile.ledTimerSpeed;

            var ledEvent = new LEDControlEvent(blinkSpeed, timerSpeed, true, soundEvent.volume, directionStr);
            EventManager.Publish(ledEvent);

            Debug.Log($"[CarController] LED Event: Speed={blinkSpeed:F2}, Timer={timerSpeed:F2} (From {soundEvent.soundType} profile)");
        }
        else
        {
            var ledEvent = new LEDControlEvent(0f, 0f, false, soundEvent.volume, directionStr);
            EventManager.Publish(ledEvent);

            Debug.Log($"[CarController] LED Event: STOP (Level: {volumeLevel.name})");
        }
    }

    private void RouteToMovement(CarInputEvent inputEvent)
    {
        var movementEvent = new MovementControlEvent(inputEvent.throttle, inputEvent.steer, inputEvent.brake);
        EventManager.Publish(movementEvent);
    }


    private SoundEvent.Direction ConvertToSoundDirection(string directionStr)
    {
        switch (directionStr)
        {
            case "ahead": return SoundEvent.Direction.Ahead;
            case "ahead-right": return SoundEvent.Direction.AheadRight;
            case "to the right": return SoundEvent.Direction.Right;
            case "behind-right": return SoundEvent.Direction.BehindRight;
            case "behind": return SoundEvent.Direction.Behind;
            case "behind-left": return SoundEvent.Direction.BehindLeft;
            case "to the left": return SoundEvent.Direction.Left;
            case "ahead-left": return SoundEvent.Direction.AheadLeft;
            default: return SoundEvent.Direction.Ahead;
        }
    }

    private string ConvertToDirectionString(SoundEvent.Direction direction)
    {
        switch (direction)
        {
            case SoundEvent.Direction.Ahead: return "ahead";
            case SoundEvent.Direction.AheadRight: return "ahead-right";
            case SoundEvent.Direction.Right: return "to the right";
            case SoundEvent.Direction.BehindRight: return "behind-right";
            case SoundEvent.Direction.Behind: return "behind";
            case SoundEvent.Direction.BehindLeft: return "behind-left";
            case SoundEvent.Direction.Left: return "to the left";
            case SoundEvent.Direction.AheadLeft: return "ahead-left";
            default: return "ahead";
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

    // ==================== PUBLIC API METHODS ====================

    // 런타임에 새 사운드 소스 추가
    public void RegisterSoundSource(SoundType soundType, ISoundSource soundSource)
    {
        if (soundSources == null)
        {
            soundSources = new Dictionary<SoundType, ISoundSource>();
        }

        soundSources[soundType] = soundSource;
        Debug.Log($"[CarController] Registered new sound source: {soundType}");
    }

    // 사운드 소스 제거
    public void UnregisterSoundSource(SoundType soundType)
    {
        if (soundSources != null && soundSources.ContainsKey(soundType))
        {
            soundSources.Remove(soundType);
            Debug.Log($"[CarController] Unregistered sound source: {soundType}");
        }
    }

    // 설정 업데이트 메서드들
    public void UpdateSoundThreshold(SoundType soundType, string direction, float threshold)
    {
        soundResponseManager.UpdateDirectionThreshold(soundType, direction, threshold);
    }

    public void UpdateSoundActivationThreshold(SoundType soundType, float threshold)
    {
        soundResponseManager.UpdateActivationThreshold(soundType, threshold);
    }

    public float GetCurrentAmbulanceVolume() => currentAmbulanceVolume;
    public Vector3 GetAmbulanceDirection() => ambulanceDirection;
    public float GetAmbulanceDistance() => ambulanceDistance;
    public string GetAmbulanceRelativeDirection() => ambulanceRelativeDirection;
    public SoundResponseManager GetSoundResponseManager() => soundResponseManager;
    public Dictionary<SoundType, ISoundSource> GetSoundSources() => soundSources;
}