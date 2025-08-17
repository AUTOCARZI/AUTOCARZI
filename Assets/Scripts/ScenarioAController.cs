using UnityEngine;
using System.Collections;

public class ScenarioAController : MonoBehaviour
{
  [Header("Scenario Objects")]
  public CarController car;
  public CarControllerAmbulance ambulance;
  public Transform stopLine;

  [Header("Detection Settings")]
  public float sirenVolumeThreshold = 0.4f;
  public float stopLineDetectionDistance = 10f;
  [Tooltip("stopLine의 forward 방향이 진행 방향의 '앞'을 가리키도록 배치하세요. 방향이 반대라면 체크하여 반전합니다.")]
  public bool invertStopLineDirection = false;

  [Header("Movement Settings")]
  public float ambulanceSpeed = 4.0f;     // 앰뷸런스 이동 속도
  public float normalSpeed = 50f;         // 일반 자율주행 속도
  public float emergencyStopSpeed = 0f;   // 응급 정지 시 속도
  public float slowDownSpeed = 15f;       // 사이렌 인지 후 감속 속도

  [Header("Scenario Control")]
  public float scenarioEndDistance = 80f;  // 앰뷸런스가 이 거리만큼 멀어지면 시나리오 종료



  // 차량 상태
  private enum CarState
  {
    Normal,         // 정상 주행
    EmergencySlowDown,  // 사이렌 감지로 감속
    EmergencyStopped    // 정지선에서 정지
  }

  private AutonomousDrivingController carAutonomous;
  private HUDManager hudManager;
  private CarState currentCarState = CarState.Normal;
  private bool scenarioStarted = false;
  private bool scenarioEnded = false;


  void Start()
  {
    InitializeComponents();
    StartCoroutine(StartScenarioCoroutine());
  }

  void InitializeComponents()
  {
    carAutonomous = car.GetComponent<AutonomousDrivingController>();
    hudManager = FindFirstObjectByType<HUDManager>();

    if (carAutonomous == null)
    {
      Debug.LogError("[ScenarioA] AutonomousDrivingController를 찾을 수 없습니다!");
    }

    if (hudManager == null)
    {
      Debug.LogError("[ScenarioA] HUDManager를 찾을 수 없습니다!");
    }
  }

  IEnumerator StartScenarioCoroutine()
  {
    yield return new WaitForSeconds(0.5f); // 초기화 대기

    StartScenario();
    scenarioStarted = true;
  }

  void StartScenario()
  {
    if (carAutonomous != null)
    {
      carAutonomous.isAutonomousMode = true;
      carAutonomous.targetSpeed = normalSpeed;
      carAutonomous.enableEmergencyBraking = false;
    }
    StartAmbulanceMovement();
  }

  void StartAmbulanceMovement()
  {
    if (ambulance != null)
    {
      var ambulanceInput = ambulance.GetComponent<InputManagerAmbulance>();
      if (ambulanceInput != null)
      {
        ambulanceInput.SetAutoMode(true, ambulanceSpeed, true);
      }
    }
  }

  void Update()
  {
    if (!scenarioStarted || scenarioEnded) return;

    CheckSirenDetection();
    CheckStopLineProximity();
    CheckStopLineCrossed();
    CheckScenarioEndCondition();
  }

  void CheckScenarioEndCondition()
  {
    if (ambulance == null || scenarioEnded) return;

    float distance = Vector3.Distance(car.transform.position, ambulance.transform.position);

    if (distance > scenarioEndDistance)
    {
      EndScenario();
    }
  }

  void CheckStopLineProximity()
  {
    if (stopLine == null || currentCarState != CarState.EmergencySlowDown) return;
    if (!IsCarBeforeStopLine()) return;

    float distanceToStopLine = Vector3.Distance(car.transform.position, stopLine.position);

    if (distanceToStopLine < stopLineDetectionDistance)
    {
      SetCarState(CarState.EmergencyStopped);
    }
  }

  void CheckSirenDetection()
  {
    if (ambulance == null) return;

    bool sirenPlaying = ambulance.IsSirenPlaying();
    float perceivedVolume = 0f;

    if (sirenPlaying)
    {
      perceivedVolume = ambulance.GetSirenVolumeFrom(car.transform);
    }

    if (perceivedVolume > sirenVolumeThreshold && currentCarState == CarState.Normal && IsCarBeforeStopLine())
    {
      SetCarState(CarState.EmergencySlowDown);
    }
    else if (perceivedVolume <= sirenVolumeThreshold * 0.5f && currentCarState != CarState.Normal)
    {
      SetCarState(CarState.Normal);
    }
  }

  // 정지선을 통과했는지 확인하고 통과 시 즉시 정상 주행으로 복귀
  void CheckStopLineCrossed()
  {
    if (stopLine == null) return;
    if (currentCarState == CarState.EmergencySlowDown && HasCarPassedStopLine())
    {
      SetCarState(CarState.Normal);
    }

    if (currentCarState == CarState.EmergencyStopped && HasCarPassedStopLine())
    {
      SetCarState(CarState.Normal);
    }
  }

  // 정지선의 법선(방향) 계산
  Vector3 GetStopLineNormal()
  {
    if (stopLine == null) return Vector3.forward;
    return invertStopLineDirection ? -stopLine.forward : stopLine.forward;
  }

  // 차량이 정지선 '이전'에 있는지 여부 (정지선 평면 기준 음/양측 판정)
  bool IsCarBeforeStopLine()
  {
    if (stopLine == null || car == null) return true;
    Vector3 toCar = car.transform.position - stopLine.position;
    float side = Vector3.Dot(GetStopLineNormal(), toCar);
    return side < 0f;
  }

  // 차량이 정지선을 '통과'했는지 여부
  bool HasCarPassedStopLine()
  {
    if (stopLine == null || car == null) return false;
    Vector3 toCar = car.transform.position - stopLine.position;
    float side = Vector3.Dot(GetStopLineNormal(), toCar);
    return side >= 0f;
  }

  void SetCarState(CarState newState)
  {
    if (currentCarState == newState) return;

    CarState previousState = currentCarState;
    currentCarState = newState;

    switch (newState)
    {
      case CarState.Normal:
        if (carAutonomous != null)
        {
          carAutonomous.targetSpeed = normalSpeed;
        }
        HideAmbulanceHUD();
        break;

      case CarState.EmergencySlowDown:
        if (carAutonomous != null)
        {
          carAutonomous.targetSpeed = slowDownSpeed;
        }
        ShowAmbulanceHUD();
        break;

      case CarState.EmergencyStopped:
        if (carAutonomous != null)
        {
          carAutonomous.targetSpeed = emergencyStopSpeed;
        }
        break;
    }
  }

  void EndScenario()
  {
    if (scenarioEnded) return;

    scenarioEnded = true;
    currentCarState = CarState.Normal;

    HideAmbulanceHUD();
    RestoreOriginalSettings();

    // 앰뷸런스 자동 모드 해제
    if (ambulance != null)
    {
      var ambulanceInput = ambulance.GetComponent<InputManagerAmbulance>();
      if (ambulanceInput != null)
      {
        ambulanceInput.SetAutoMode(false);
      }
    }
  }

  void RestoreOriginalSettings()
  {
    if (carAutonomous != null)
    {
      carAutonomous.targetSpeed = normalSpeed;
      carAutonomous.enableEmergencyBraking = true;
    }
  }

  void ShowAmbulanceHUD()
  {
    if (hudManager != null) EventManager.Publish(new HUDControlEvent("ambulance-front", true));
  }

  void HideAmbulanceHUD()
  {
    if (hudManager != null) EventManager.Publish(new HUDControlEvent("ambulance-front", false));
  }

  // 디버그용 기즈모
  void OnDrawGizmos()
  {
    if (stopLine != null)
    {
      // 정지선 표시
      Gizmos.color = Color.red;
      Gizmos.DrawWireCube(stopLine.position, new Vector3(5f, 0.1f, 1f));

      // 정지선 감지 범위
      Gizmos.color = Color.yellow;
      Gizmos.DrawWireSphere(stopLine.position, stopLineDetectionDistance);
    }

    if (ambulance != null && car != null)
    {
      // Car와 Ambulance 연결선
      Gizmos.color = currentCarState != CarState.Normal ? Color.red : Color.green;
      Gizmos.DrawLine(car.transform.position, ambulance.transform.position);
    }

    // 차량 상태 표시
    if (currentCarState != CarState.Normal && car != null)
    {
      Gizmos.color = currentCarState == CarState.EmergencyStopped ? Color.red : Color.orange;
      Gizmos.DrawWireSphere(car.transform.position, 3f);
    }
  }

  // 시나리오 강제 재시작 (디버그용)
  [ContextMenu("Restart Scenario")]
  public void RestartScenario()
  {
    currentCarState = CarState.Normal;
    scenarioStarted = false;
    scenarioEnded = false;

    HideAmbulanceHUD();

    // 앰뷸런스 자동 모드 해제
    if (ambulance != null)
    {
      var ambulanceInput = ambulance.GetComponent<InputManagerAmbulance>();
      if (ambulanceInput != null)
      {
        ambulanceInput.SetAutoMode(false);
      }
    }

    StartCoroutine(StartScenarioCoroutine());
  }

  // 시나리오 정지 (디버그용)
  [ContextMenu("Stop Scenario")]
  public void StopScenario()
  {
    scenarioStarted = false;
    scenarioEnded = true;
    currentCarState = CarState.Normal;

    HideAmbulanceHUD();
    RestoreOriginalSettings();

    // 앰뷸런스 자동 모드 해제
    if (ambulance != null)
    {
      var ambulanceInput = ambulance.GetComponent<InputManagerAmbulance>();
      if (ambulanceInput != null)
      {
        ambulanceInput.SetAutoMode(false);
      }
    }


  }
}
