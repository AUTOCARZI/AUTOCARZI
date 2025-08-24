using UnityEngine;
using System.Collections;

public class ScenarioCController : MonoBehaviour
{
  [Header("Scenario Objects")]
  public CarController car;

  [Header("Movement Settings")]
  public float normalSpeed = 15;          // 일반 자율주행 속도
  public float laneChangeSpeed = 15f;     // 차선 변경 시 속도 (우측)
  public float laneCorrectionSpeed = 30f; // 차선 교정 시 속도 (좌측)
  public float rightTurnSpeed = 30;      // 우회전 시 속도

  [Header("Scenario Control")]
  public float bypassEventDelay = 0f;
  public float laneChangeDistance = 20f;     // 차선 변경에 필요한 거리
  public float laneCorrectionDistance = 9f; // 차선 교정에 필요한 거리
  public float rightTurnDelay = 2f;
  public float rightTurnDistance = 15f;      // 우회전에 필요한 거리

  // 거리 추적 변수
  private Vector3 stateStartPosition;
  private float distanceTraveled;
  private float totalDistance;

  // 차량 상태
  private enum CarState
  {
    Normal,         // 정상 자율주행
    LaneChanging,   // 차선 변경 중 (우측)
    LaneCorrecting, // 차선 교정 중 (좌측)
    RightTurning    // 우회전 중
  }

  [Header("Movement Control")]
  public float laneChangeSteer = 0.5f;    // 차선 변경을 위한 조향 입력 (더 강하게)
  public float laneCorrectionSteer = -1.0f; // 차선 교정을 위한 좌측 조향
  public float rightTurnSteer = 1.0f;     // 우회전을 위한 조향 입력
  public float laneChangeThrottle = 0.8f; // 차선 변경 중 스로틀 (빠르게)
  public float rightTurnThrottle = 0.6f;  // 우회전 중 스로틀

  private AutonomousDrivingController carAutonomous;
  private HUDManager hudManager;
  private CarState currentCarState = CarState.Normal;
  private bool isManualControlActive = false;
  private bool scenarioStarted = false;
  private bool scenarioEnded = false;

  void Start()
  {
    InitializeComponents();

    // 즉시 normalSpeed 적용
    if (carAutonomous != null)
    {
      carAutonomous.isAutonomousMode = true;
      carAutonomous.targetSpeed = normalSpeed;
      Debug.Log($"[ScenarioC] 초기 normalSpeed 적용: {normalSpeed}");
    }

    StartCoroutine(StartScenarioCoroutine());
  }

  void InitializeComponents()
  {
    // car 참조 null 체크
    if (car == null)
    {
      Debug.LogError("[ScenarioC] Car 참조가 설정되지 않았습니다! Inspector에서 CarController를 할당해주세요.");
      enabled = false;
      return;
    }

    carAutonomous = car.GetComponent<AutonomousDrivingController>();
    hudManager = FindFirstObjectByType<HUDManager>();

    if (carAutonomous == null)
    {
      Debug.LogError("[ScenarioC] AutonomousDrivingController를 찾을 수 없습니다!");
      enabled = false;
      return;
    }

    if (hudManager == null)
    {
      Debug.LogError("[ScenarioC] HUDManager를 찾을 수 없습니다!");
    }

    Debug.Log("[ScenarioC] ScenarioCController 초기화 완료");
  }

  IEnumerator StartScenarioCoroutine()
  {
    yield return new WaitForSeconds(0.5f); // 초기화 대기

    StartScenario();
    scenarioStarted = true;
  }

  void StartScenario()
  {
    // 기본적으로 자율주행 시작
    if (carAutonomous != null)
    {
      carAutonomous.isAutonomousMode = true;
      carAutonomous.targetSpeed = normalSpeed;
      carAutonomous.enableEmergencyBraking = false;
    }

    Debug.Log("[ScenarioC] 시나리오 C 시작 - 자율주행 활성화");

    // 시나리오 실행
    StartCoroutine(ExecuteScenarioCoroutine());
  }

  IEnumerator ExecuteScenarioCoroutine()
  {
    // 시작하자마자 bypass-traffic 이벤트 발행
    TriggerBypassTrafficEvent();

    // 거리 기반 시나리오 실행
    yield return StartCoroutine(ExecuteDistanceBasedScenario());

    EndScenario();
  }

  IEnumerator ExecuteDistanceBasedScenario()
  {
    // 1단계: 차선 변경
    SetCarState(CarState.LaneChanging);
    yield return StartCoroutine(WaitForDistance(laneChangeDistance));

    // 2단계: 차선 교정
    SetCarState(CarState.LaneCorrecting);
    yield return StartCoroutine(WaitForDistance(laneCorrectionDistance));

    // 3단계: 정상 주행 (잠시 대기)
    SetCarState(CarState.Normal);
    yield return new WaitForSeconds(rightTurnDelay);

    // 4단계: 우회전
    SetCarState(CarState.RightTurning);
    yield return StartCoroutine(WaitForDistance(rightTurnDistance));

    // 5단계: 정상 주행으로 복귀
    SetCarState(CarState.Normal);
  }

  IEnumerator WaitForDistance(float targetDistance)
  {
    if (car == null) yield break;

    stateStartPosition = car.transform.position;
    distanceTraveled = 0f;

    while (distanceTraveled < targetDistance)
    {
      Vector3 currentPosition = car.transform.position;
      distanceTraveled = Vector3.Distance(stateStartPosition, currentPosition);
      
      // 디버그 정보
      if (Time.frameCount % 30 == 0) // 30프레임마다 출력
      {
        Debug.Log($"[ScenarioC] 상태: {currentCarState}, 진행거리: {distanceTraveled:F1}m / {targetDistance}m");
      }

      yield return null; // 한 프레임 대기
    }

    Debug.Log($"[ScenarioC] {currentCarState} 상태 완료 - 총 {distanceTraveled:F1}m 이동");
  }

  void EndScenario()
  {
    if (scenarioEnded) return;

    scenarioEnded = true;
    currentCarState = CarState.Normal;

    // 정상 주행으로 복귀
    SetCarState(CarState.Normal);

    // bypass-traffic HUD 숨기기
    HideBypassTrafficEvent();

    Debug.Log("[ScenarioC] 시나리오 완료");
  }

  void RestoreOriginalSettings()
  {
    if (carAutonomous != null)
    {
      carAutonomous.SetAutonomousMode(true);
      carAutonomous.targetSpeed = normalSpeed;
      carAutonomous.enableEmergencyBraking = true;
      carAutonomous.SetSteeringOverride(false); // 조향 오버라이드 해제
    }
    isManualControlActive = false;
  }

  void SetCarState(CarState newState)
  {
    if (currentCarState == newState) return;

    currentCarState = newState;

    switch (newState)
    {
      case CarState.Normal:
        // 자율주행 모드 활성화 + 조향 오버라이드 해제
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = normalSpeed;
          carAutonomous.SetSteeringOverride(false); // 조향 오버라이드 해제
        }
        isManualControlActive = false;
        Debug.Log("[ScenarioC] 정상 자율주행 상태 - 차선 유지 모드");
        break;

      case CarState.LaneChanging:
        // 자율주행 유지하되 우측 조향 오버라이드 (차선 변경을 위해)
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = laneChangeSpeed;
          carAutonomous.SetSteeringOverride(true, laneChangeSteer);
        }
        isManualControlActive = false; // 수동 제어 비활성화
        Debug.Log("[ScenarioC] 차선 변경 시작 - 우측 조향 오버라이드");
        break;

      case CarState.LaneCorrecting:
        // 자율주행 유지하되 좌측 조향 오버라이드 (차선 교정을 위해)
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = laneCorrectionSpeed; // 전용 속도 사용
          carAutonomous.SetSteeringOverride(true, laneCorrectionSteer);
        }
        isManualControlActive = false; // 수동 제어 비활성화
        Debug.Log("[ScenarioC] 차선 교정 시작 - 좌측 조향 오버라이드");
        break;

      case CarState.RightTurning:
        // 자율주행 유지하되 조향만 오버라이드 (우회전을 위해)
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = rightTurnSpeed;
          carAutonomous.SetSteeringOverride(true, rightTurnSteer);
        }
        isManualControlActive = false; // 수동 제어 비활성화
        Debug.Log("[ScenarioC] 우회전 시작 - 자율주행 + 조향 오버라이드");
        break;
    }
  }

  void TriggerBypassTrafficEvent()
  {
    // bypass-traffic HUD 표시
    if (hudManager != null)
    {
      EventManager.Publish(new HUDControlEvent("bypass-traffic", true));
    }

    Debug.Log("[ScenarioC] 'bypass-traffic' 이벤트 발행");
  }

  void HideBypassTrafficEvent()
  {
    // bypass-traffic HUD 숨기기
    if (hudManager != null)
    {
      EventManager.Publish(new HUDControlEvent("bypass-traffic", false));
    }

    Debug.Log("[ScenarioC] 'bypass-traffic' 이벤트 종료");
  }

  void Update()
  {
    if (!scenarioStarted || scenarioEnded) return;

    // 총 이동거리 추적
    if (car != null)
    {
      if (totalDistance == 0f) // 초기 설정
      {
        totalDistance = 0f;
      }
    }

    // Normal 상태일 때 normalSpeed가 제대로 적용되고 있는지 확인
    if (currentCarState == CarState.Normal && carAutonomous != null && carAutonomous.isAutonomousMode)
    {
      if (Mathf.Abs(carAutonomous.targetSpeed - normalSpeed) > 0.1f)
      {
        carAutonomous.targetSpeed = normalSpeed;
        Debug.Log($"[ScenarioC] normalSpeed 업데이트: {normalSpeed}");
      }
    }

    // 수동 제어가 활성화된 경우 조향 입력 전송
    if (isManualControlActive)
    {
      SendManualInput();
    }
  }

  void SendManualInput()
  {
    float throttle = 0f;
    float steer = 0f;
    bool brake = false;

    switch (currentCarState)
    {
      case CarState.LaneChanging:
        throttle = laneChangeThrottle;
        steer = laneChangeSteer; // 오른쪽으로 조향
        break;

      case CarState.RightTurning:
        throttle = rightTurnThrottle;
        steer = rightTurnSteer; // 우회전을 위한 강한 조향
        break;

      case CarState.Normal:
      default:
        // 자율주행 모드로 복귀
        if (carAutonomous != null && !carAutonomous.isAutonomousMode)
        {
          carAutonomous.SetAutonomousMode(true);
          isManualControlActive = false;
        }
        return;
    }

    // CarInputEvent를 통해 조향 입력 전송
    var inputEvent = new CarInputEvent(throttle, steer, brake, false);
    EventManager.Publish(inputEvent);
  }

  // 시나리오 재시작 (디버그용)
  [ContextMenu("Restart Scenario")]
  public void RestartScenario()
  {
    StopAllCoroutines();

    scenarioStarted = false;
    scenarioEnded = false;
    currentCarState = CarState.Normal;

    // 자율주행 복구
    RestoreOriginalSettings();

    // HUD 숨기기
    HideBypassTrafficEvent();

    StartCoroutine(StartScenarioCoroutine());
    Debug.Log("[ScenarioC] 시나리오 재시작");
  }

  // 시나리오 정지 (디버그용)
  [ContextMenu("Stop Scenario")]
  public void StopScenario()
  {
    scenarioStarted = false;
    scenarioEnded = true;
    currentCarState = CarState.Normal;

    RestoreOriginalSettings();
    HideBypassTrafficEvent();

    Debug.Log("[ScenarioC] 시나리오 정지");
  }
}
