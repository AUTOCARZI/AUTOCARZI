using UnityEngine;

public class ScenarioDController : MonoBehaviour
{
    private Rigidbody rb;
    private AutonomousDrivingController autonomousController;
    private PoliceOfficer police;
    private HUDManager hudManager;
    
    [Header("Detection Settings")]
    public float whistleVolumeThreshold = 0.4f;
    
    // 차량 상태
    private enum CarState
    {
        Normal,         // 정상 주행
        WhistleDetected // 호루라기 감지
    }
    
    private CarState currentCarState = CarState.Normal;
    private bool scenarioStarted = false;
    private bool scenarioEnded = false;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        autonomousController = GetComponent<AutonomousDrivingController>();
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
        
        scenarioStarted = true;
        Debug.Log("[ScenarioD] 시나리오 시작 - 호루라기 감지 대기 중");
    }
    
    void Update()
    {
        if (!scenarioStarted || scenarioEnded) return;

        CheckWhistleDetection();
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
        
        if (other.CompareTag("Wall"))
        {
            Debug.Log("[ScenarioD] Wall 충돌 - 자율주행 비활성화");
            autonomousController.SetAutonomousMode(false);
            
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            
            Destroy(other.gameObject);
        } 
        else if (other.CompareTag("Auto"))
        {
            Debug.Log("[ScenarioD] Auto 충돌 - 시나리오 종료");
            autonomousController.SetAutonomousMode(true);
            
            if (police != null)
            {
                police.StopWhistling();
            }
            
            // 시나리오 종료
            EndScenario();
            
            Destroy(other.gameObject);
        }
    }
    
    void EndScenario()
    {
        if (scenarioEnded) return;

        scenarioEnded = true;
        currentCarState = CarState.Normal;
        
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
}