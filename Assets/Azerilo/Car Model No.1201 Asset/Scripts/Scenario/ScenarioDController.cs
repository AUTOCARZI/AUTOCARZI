using UnityEngine;

public class ScenarioDController : MonoBehaviour
{
    [Header("Detection Settings")]
    public bool rainMode = false;

    private Rigidbody rb;
    private AutonomousDrivingController autonomousController;
    private PoliceOfficer police;
    private HUDManager hudManager;
    private CarController carController;
    
    [Header("Detection Settings")]
    public float whistleVolumeThreshold = 0.4f;
    
    [Header("Movement Settings")]
    public float defaultSpeed = 10f; 
    public float maxSpeed = 20f;
    public bool enableDefaultMovement = true;
    public bool hitWall = false;
    
    // 차량 상태
    private enum CarState
    {
        Normal,         // 정상 주행
        WhistleDetected // 호루라기 감지
    }
    
    public bool isTurningRight = false;
    private CarState currentCarState = CarState.Normal;
    private bool scenarioStarted = false;
    private bool scenarioEnded = false;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        autonomousController = GetComponent<AutonomousDrivingController>();
        carController = GetComponent<CarController>();
        police = FindFirstObjectByType<PoliceOfficer>();
        hudManager = FindFirstObjectByType<HUDManager>();
        
        if (hudManager == null)
        {
            Debug.LogError("[ScenarioD] HUDManager를 찾을 수 없습니다!");
        }
        
        if (police == null)
        {
            Debug.LogError("[ScenarioD] PoliceOfficer를 찾을 수 없습니다!");
        }
        
        if (carController == null)
        {
            Debug.LogError("[ScenarioD] CarController를 찾을 수 없습니다!");
        }
        
        scenarioStarted = true;
        Debug.Log("[ScenarioD] 시나리오 시작 - 호루라기 감지 대기 중, 기본 전진 모드");
    }
    
    void Update()
    {
        if (!scenarioStarted || scenarioEnded) return;

        CheckWhistleDetection();
        HandleDefaultMovement();
    }
    
    void HandleDefaultMovement()
    {
        if (!enableDefaultMovement) return;
    
        bool isAutonomousActive = autonomousController != null && autonomousController.isAutonomousMode;
    
        if (rainMode && !isAutonomousActive && !hitWall){
            float steerInput = isTurningRight ? 1f : 0f;
            Debug.Log($"[ScenarioD] Movement - Throttle: 0.5, Steer: {steerInput}, isTurningRight: {isTurningRight}");
            var movementEvent = new MovementControlEvent(0.4f, steerInput, false);
            EventManager.Publish(movementEvent);
        }
    }
    
    void CheckWhistleDetection()
    {
        if (police == null || police.whistleAudioSource == null) return;

        bool whistlePlaying = police.whistleAudioSource.isPlaying;
        float perceivedVolume = 0f;

        if (whistlePlaying)
        {
            perceivedVolume = GetWhistleVolumeFrom(transform);
            Debug.Log($"[ScenarioD] 호루라기 소리 감지: {perceivedVolume:F3}, 임계값: {whistleVolumeThreshold}");
        }

        if (perceivedVolume > whistleVolumeThreshold && currentCarState == CarState.Normal)
        {
            Debug.Log("[ScenarioD] 호루라기 임계값 초과 - HUD 표시");
            SetCarState(CarState.WhistleDetected);
        }
        else if (perceivedVolume <= whistleVolumeThreshold * 0.5f && currentCarState == CarState.WhistleDetected)
        {
            Debug.Log("[ScenarioD] 호루라기 소리 약해짐 - HUD 숨김");
            SetCarState(CarState.Normal);
        }
    }
    
    float GetWhistleVolumeFrom(Transform listener)
    {
        if (police == null || police.whistleAudioSource == null || !police.whistleAudioSource.isPlaying) 
            return 0f;

        float distance = Vector3.Distance(police.transform.position, listener.position);
        float baseVolume = police.whistleAudioSource.volume;
        
        // PoliceWhistleSoundSource와 동일한 거리 감쇠 계산
        float distanceAttenuation = Mathf.Max(0f, 1f - (distance / 200f));
        float perceivedVolume = baseVolume * distanceAttenuation;

        return perceivedVolume;
    }
    
    void SetCarState(CarState newState)
    {
        if (currentCarState == newState) return;

        CarState previousState = currentCarState;
        currentCarState = newState;
        Debug.Log($"[ScenarioD] 상태 변경: {previousState} → {newState}");

        switch (newState)
        {
            case CarState.Normal:
                HideBypassAccidentHUD();
                break;

            case CarState.WhistleDetected:
                ShowBypassAccidentHUD();
                break;
        }
    }
    
    void OnTriggerEnter(Collider other)
    {   
        Debug.Log($"[ScenarioD] 충돌 감지: {other.name}, 태그: {other.tag}");
        
        if (rainMode){
            if (other.CompareTag("Step"))
            {     
                Debug.Log("[ScenarioD] Step collision confirmed - starting turn");
                StartCoroutine(TurnRightForDuration(2.6f));
            } 
        }

        if (other.CompareTag("Wall"))
            {
                Debug.Log("[ScenarioD] Wall 충돌 - 자율주행 비활성화, 기본 움직임 활성화");

                if (autonomousController != null)
                {
                    autonomousController.SetAutonomousMode(false);
                }
            
                // Reset velocity but allow default movement to take over
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            
                // Enable default movement
                enableDefaultMovement = true;
                hitWall = true;
            
                Destroy(other.gameObject);
            } 
    }
    
    System.Collections.IEnumerator TurnRightForDuration(float duration)
    {
        isTurningRight = true;
        Debug.Log($"[ScenarioD] {duration}초간 우회전 시작");
        
        yield return new WaitForSeconds(duration);
        
        isTurningRight = false;
        Debug.Log("[ScenarioD] 우회전 완료 - 직진으로 복귀");
    }
    
    void EndScenario()
    {
        if (scenarioEnded) return;

        scenarioEnded = true;
        currentCarState = CarState.Normal;
        enableDefaultMovement = false; // Stop default movement when scenario ends
        
        Debug.Log("[ScenarioD] 시나리오 종료");
        HideBypassAccidentHUD();
    }
    
    void ShowBypassAccidentHUD()
    {
        Debug.Log("[ScenarioD] ShowBypassAccidentHUD 호출됨");
        if (hudManager != null) 
        {
            Debug.Log("[ScenarioD] HUDManager 존재, bypass-accident 이벤트 발행");
            EventManager.Publish(new HUDControlEvent("bypass-accident", true));
        }
        else
        {
            Debug.LogError("[ScenarioD] HUDManager가 null입니다!");
        }
    }

    void HideBypassAccidentHUD()
    {
        Debug.Log("[ScenarioD] HideBypassAccidentHUD 호출됨");
        if (hudManager != null) 
        {
            Debug.Log("[ScenarioD] bypass-accident HUD 숨김 이벤트 발행");
            EventManager.Publish(new HUDControlEvent("bypass-accident", false));
        }
    }
    
    public void SetDefaultMovement(bool enabled)
    {
        enableDefaultMovement = enabled;
        Debug.Log($"[ScenarioD] Default movement set to: {enabled}");
    }
    
    public void SetDefaultSpeed(float speed)
    {
        defaultSpeed = Mathf.Max(0f, speed);
        Debug.Log($"[ScenarioD] Default speed set to: {defaultSpeed}");
    }
    
    public bool IsDefaultMovementEnabled()
    {
        return enableDefaultMovement;
    }
}