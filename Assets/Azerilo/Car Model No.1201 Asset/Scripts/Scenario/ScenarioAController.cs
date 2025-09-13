using UnityEngine;
using System.Collections;

public class ScenarioAController : MonoBehaviour
{
  [Header("Scenario Objects")]
  public CarController car;
  public CarControllerAmbulance ambulance;
  public Transform stopLine;

  [Header("Detection Settings")]
  public float sirenVolumeThreshold = 0.1f;
  public float stopLineDetectionDistance = 10f;
  [Tooltip("stopLine의 forward 방향이 진행 방향의 '앞'을 가리키도록 배치하세요. 방향이 반대라면 체크하여 반전합니다.")]
  public bool invertStopLineDirection = false;

  [Header("Movement Settings")]
  public float ambulanceSpeed = 2.0f;     // 앰뷸런스 이동 속도
  public float normalSpeed = 40f;         // 일반 자율주행 속도
  public float emergencyStopSpeed = 0f;   // 응급 정지 시 속도
  public float slowDownSpeed = 15f;       // 사이렌 인지 후 감속 속도

  [Header("Scenario Control")]
  public float waitForSeconds = 3f;       // 정지 후 대기 시간
  public float straightDistance = 40f;    // 직진 거리
  public float scenarioEndDistance = 200f;  // 앰뷸런스가 이 거리만큼 멀어지면 시나리오 종료



  // 차량 상태
  private enum CarState
  {
    Normal,         // 정상 주행
    EmergencySlowDown,  // 사이렌 감지로 감속
    EmergencyStopped    // 정지선에서 정지
  }

  [Header("Movement Control")]
  public float throttleValue = 0.8f;
  public float brakeValue = 1.0f;

  private Vector3 stateStartPosition;
  private float distanceTraveled;
  private bool isMovingForward = true;

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
      carAutonomous.isAutonomousMode = false;
      carAutonomous.enableEmergencyBraking = false;
    }
    StartAmbulanceMovement();
    StartCoroutine(ExecuteScenarioCoroutine());
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

  IEnumerator ExecuteScenarioCoroutine()
  {
    yield return StartCoroutine(ExecuteDistanceBasedScenario());
    EndScenario();
  }

  IEnumerator ExecuteDistanceBasedScenario()
  {
    SetCarState(CarState.Normal);
    yield return StartCoroutine(MoveForwardDistance(straightDistance));

    SetCarState(CarState.EmergencyStopped);
    yield return new WaitForSeconds(waitForSeconds);

    SetCarState(CarState.Normal);
    yield return StartCoroutine(MoveForwardDistance(straightDistance));

    SetCarState(CarState.Normal);
  }

  IEnumerator MoveForwardDistance(float targetDistance)
  {
    if (car == null) yield break;

    stateStartPosition = car.transform.position;
    distanceTraveled = 0f;

    while (distanceTraveled < targetDistance)
    {
      Vector3 currentPosition = car.transform.position;
      distanceTraveled = Vector3.Distance(stateStartPosition, currentPosition);
      yield return null;
    }
  }

  void Update()
  {
    if (!scenarioStarted || scenarioEnded) return;

    CheckScenarioEndCondition();
    SendManualInput();
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
        isMovingForward = true;
        break;

      case CarState.EmergencySlowDown:
        isMovingForward = true;
        break;

      case CarState.EmergencyStopped:
        isMovingForward = false;
        break;
    }
  }

  void SendManualInput()
  {
    float throttle = 0f;
    float steer = 0f;
    bool brake = false;

    switch (currentCarState)
    {
      case CarState.Normal:
        if (isMovingForward)
        {
          throttle = throttleValue;
        }
        break;

      case CarState.EmergencySlowDown:
        if (isMovingForward)
        {
          throttle = throttleValue * 0.5f;
        }
        break;

      case CarState.EmergencyStopped:
        brake = true;
        break;
    }

    var inputEvent = new CarInputEvent(throttle, steer, brake, false);
    EventManager.Publish(inputEvent);
  }

  void EndScenario()
  {
    if (scenarioEnded) return;

    scenarioEnded = true;
    currentCarState = CarState.Normal;

    // HideAmbulanceHUD();
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
      carAutonomous.isAutonomousMode = true;
      carAutonomous.targetSpeed = normalSpeed;
      carAutonomous.enableEmergencyBraking = true;
    }
    isMovingForward = false;
  }


}
