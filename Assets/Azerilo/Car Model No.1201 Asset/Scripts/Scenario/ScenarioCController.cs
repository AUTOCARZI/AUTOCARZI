using UnityEngine;
using System.Collections;

public class ScenarioCController : MonoBehaviour
{
  [Header("Scenario Objects")]
  public CarController car;

  [Header("Movement Settings")]
  public float normalSpeed = 15;
  public float laneChangeSpeed = 15f;
  public float laneCorrectionSpeed = 30f;
  public float rightTurnSpeed = 30;

  [Header("Scenario Control")]
  public float straightDistance = 20f;
  public float laneChangeDistance = 21f;
  public float laneCorrectionDistance = 9f;
  public float beforeRightTurnDistance = 13f;
  public float rightTurnDistance = 10f;

  private Vector3 stateStartPosition;
  private float distanceTraveled;

  private enum CarState
  {
    Normal,
    LaneChanging,
    LaneCorrecting,
    RightTurning
  }

  [Header("Movement Control")]
  public float laneChangeSteer = 0.5f;
  public float laneCorrectionSteer = -1.0f;
  public float rightTurnSteer = 2.5f;
  public float laneChangeThrottle = 0.8f;
  public float rightTurnThrottle = 1.0f;

  private AutonomousDrivingController carAutonomous;
  private HUDManager hudManager;
  private LEDManager ledManager;
  private CarState currentCarState = CarState.Normal;
  private bool isManualControlActive = false;
  private bool scenarioStarted = false;
  private bool scenarioEnded = false;

  void Start()
  {
    InitializeComponents();

    if (carAutonomous != null)
    {
      carAutonomous.SetAutonomousMode(false);
    }

    // 즉시 수동제어 활성화
    isManualControlActive = true;
    currentCarState = CarState.Normal;

    StartCoroutine(StartScenarioCoroutine());
  }

  void InitializeComponents()
  {
    if (car == null)
    {
      enabled = false;
      return;
    }

    carAutonomous = car.GetComponent<AutonomousDrivingController>();
    hudManager = FindFirstObjectByType<HUDManager>();
    ledManager = FindFirstObjectByType<LEDManager>();
  }

  IEnumerator StartScenarioCoroutine()
  {
    scenarioStarted = true;
    yield return new WaitForSeconds(0.5f);

    StartScenario();
  }

  void StartScenario()
  {
    if (carAutonomous != null)
    {
      carAutonomous.SetAutonomousMode(false);
      carAutonomous.enableEmergencyBraking = false;
    }

    StartCoroutine(ExecuteScenarioCoroutine());
  }

  IEnumerator ExecuteScenarioCoroutine()
  {
    yield return StartCoroutine(ExecuteDistanceBasedScenario());
    EndScenario();
  }

  IEnumerator ExecuteDistanceBasedScenario()
  {
    yield return StartCoroutine(WaitForDistance(straightDistance));

    TriggerBypassTrafficEvent();
    SetCarState(CarState.LaneChanging);
    yield return StartCoroutine(WaitForDistance(laneChangeDistance));

    SetCarState(CarState.LaneCorrecting);
    yield return StartCoroutine(WaitForDistance(laneCorrectionDistance));
    
    TriggerBypassTrafficEndEvent();
    SetCarState(CarState.Normal);
    yield return StartCoroutine(WaitForDistance(beforeRightTurnDistance));

    SetCarState(CarState.RightTurning);
    yield return StartCoroutine(WaitForDistance(rightTurnDistance));

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
      yield return null;
    }
  }

  void EndScenario()
  {
    if (scenarioEnded) return;

    scenarioEnded = true;
    currentCarState = CarState.Normal;

    SetCarState(CarState.Normal);
  }

  void RestoreOriginalSettings()
  {
    if (carAutonomous != null)
    {
      carAutonomous.SetAutonomousMode(false);
      carAutonomous.enableEmergencyBraking = true;
    }
    isManualControlActive = true;
  }

  void SetCarState(CarState newState)
  {
    if (currentCarState == newState) return;

    currentCarState = newState;

    switch (newState)
    {
      case CarState.Normal:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        break;

      case CarState.LaneChanging:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        break;

      case CarState.LaneCorrecting:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        break;

      case CarState.RightTurning:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        break;
    }
  }

  void TriggerBypassTrafficEvent()
  {
    if (hudManager != null && ledManager != null )
    {
      EventManager.Publish(new HUDControlEvent("bypass-traffic", true));
      EventManager.Publish(new LEDControlEvent(1.0f, 1.0f, true, 0, "", SoundType.Default)); 
    }
  }

  void TriggerBypassTrafficEndEvent()
  {
    if (ledManager != null )
    {
      EventManager.Publish(new LEDControlEvent(0f, 0f, false, 0, "", SoundType.None)); 
    }
  }

  void Update()
  {
    if (!scenarioStarted || scenarioEnded) 
    {
      return;
    }

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
      case CarState.Normal:
        throttle = 0.4f;
        steer = 0f;
        break;
        
      case CarState.LaneChanging:
        throttle = 0.4f;
        steer = laneChangeSteer;
        break;
        
      case CarState.LaneCorrecting:
        throttle = 0.4f;
        steer = laneCorrectionSteer;
        break;

      case CarState.RightTurning:
        throttle = 0.4f;
        steer = rightTurnSteer;
        break;
    }

    var movementEvent = new MovementControlEvent(throttle, steer, brake);
    EventManager.Publish(movementEvent);
  }

  [ContextMenu("Restart Scenario")]
  public void RestartScenario()
  {
    StopAllCoroutines();

    scenarioStarted = false;
    scenarioEnded = false;
    currentCarState = CarState.Normal;

    RestoreOriginalSettings();

    StartCoroutine(StartScenarioCoroutine());
  }

  [ContextMenu("Stop Scenario")]
  public void StopScenario()
  {
    scenarioStarted = false;
    scenarioEnded = true;
    currentCarState = CarState.Normal;

    RestoreOriginalSettings();
  }
}
