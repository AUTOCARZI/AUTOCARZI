using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;

public class AutonomousCarAgent : Agent
{
    [Header("Car Controller Reference")]
    public CarController carController;

    [Header("Sensors")]
    public Transform sensorOrigin;
    public float sensorRange = 20f;
    public int sensorCount = 3;
    public LayerMask obstacleLayer = -1;

    [Header("Lane Detection")]
    public float laneDetectionRange = 10f;
    public int laneRayCount = 3;
    public LayerMask laneLayer = -1;
    public LayerMask vehicleLayer = -1;

    [Header("Lane Following")]
    public float laneKeepingReward = 0.5f;
    public float perfectLaneReward = 0.8f;
    public float laneChangeReward = 0.3f;
    public float wrongLaneReward = -1.0f;
    public float emergencyLaneChangeReward = 0.5f;
    public float rapidReturnReward = 0.4f;
    public float superPerfectBonus = 1.0f;

    [Header("Braking System")]
    public float emergencyBrakeReward = 0.1f;
    public float safeBrakingReward = 0.05f;
    public float crashAvoidanceReward = 0.2f;
    public float recklessDrivingPenalty = -0.1f;

    private bool wasInDanger = false;
    private float dangerStartTime = 0f;

    [Header("Target")]
    public Transform target;
    public float targetReachedDistance = 5f;

    [Header("Action Scaling")]
    [Range(0.5f, 3f)]
    public float steeringMultiplier = 1.5f;
    [Range(0.1f, 1f)]
    public float steeringThreshold = 0.5f;
    public bool useDiscreteSteering = false;
    public float discreteSteeringStrength = 1f;

    [Header("Steering Smoothing")]
    public float steeringSmoothing = 0.1f;
    private float lastAppliedSteering = 0f;

    [Header("Rewards")]
    public float rewardForMovingTowardsTarget = 0.1f;
    public float rewardForReachingTarget = 10f;
    public float rewardForPriceCollection = 5f;
    public float penaltyForCollision = -15f;
    public float penaltyForMovingAwayFromTarget = -0.05f;
    public float rewardForStrongSteering = 0.05f;

    private Vector3 previousPosition;
    private float previousDistanceToTarget;
    private bool hasCollided = false;

    // 지속적 회전 추적용 변수들
    private float continuousSteerTime = 0f;
    private float lastSteerDirection = 0f;

    // 차선 상태 추적용 변수들
    private float currentLanePosition = 0f;
    private bool isNearVehicle = false;
    private bool isChangingLane = false;
    private float laneChangeStartTime = 0f;
    private float timeInLane = 0f;
    private float lastLaneViolationTime = 0f;
    private bool wasInEmergency = false;

    // 차선 넘어감 감지용 변수들
    private bool wasInLaneBoundary = true;
    private float laneViolationStartTime = 0f;
    private int totalLaneViolations = 0;

    // 초기 위치 저장용
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    public override void Initialize()
    {
        Debug.Log("🚀 AutonomousCarAgent 초기화 시작!");

        // CarController가 없으면 찾기
        if (carController == null)
            carController = GetComponent<CarController>();

        // 센서 원점이 없으면 차량 중심으로 설정
        if (sensorOrigin == null)
            sensorOrigin = transform;

        // 초기 위치와 회전 저장
        initialPosition = transform.position;
        initialRotation = transform.rotation;
        Debug.Log($"📍 초기 위치 저장: {initialPosition}");

        // 시뮬레이션 속도 설정
        Time.timeScale = 10f; // 50배는 너무 빨라서 10배로 조정
        Time.fixedDeltaTime = 0.02f / Time.timeScale;
        Application.targetFrameRate = -1;
        QualitySettings.vSyncCount = 0;

        Debug.Log($"⚡ 시뮬레이션 {Time.timeScale}배 가속 설정 완료!");
        Debug.Log("✅ AutonomousCarAgent 초기화 완료!");
    }

    void Update()
    {
        // 스크립트가 작동하는지 확인
        if (Time.time % 10f < Time.deltaTime)
        {
            Debug.Log($"🔄 Agent 살아있음 - Time: {Time.time:F1}s, StepCount: {StepCount}");
            RequestDecision();
        }
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log($"🔄 새 에피소드 시작! Episode: {CompletedEpisodes}, 이전 Step: {StepCount}");

        // 에피소드 시작 시 초기화
        hasCollided = false;
        previousPosition = transform.position;

        if (target != null)
        {
            previousDistanceToTarget = Vector3.Distance(transform.position, target.position);
            Debug.Log($"📍 타겟까지 거리: {previousDistanceToTarget:F1}m");
        }
        else
        {
            Debug.LogWarning("⚠️ 타겟이 설정되지 않았습니다!");
        }

        // 차량 위치와 회전 리셋
        ResetCarPosition();

        // 이전 에피소드 통계 로그
        LogEpisodeStats();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Vector Observation Space Size = 14개로 맞춤

        // 1. 레이캐스트 센서 데이터 수집 (3개)
        for (int i = 0; i < sensorCount; i++)
        {
            float angle = (i - sensorCount / 2) * 30f;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            RaycastHit[] hits = Physics.RaycastAll(sensorOrigin.position, direction, sensorRange, obstacleLayer);
            float nearestDistance = sensorRange;
            bool foundRealObstacle = false;

            foreach (RaycastHit hit in hits)
            {
                // Price 태그 완전 무시
                if (hit.collider.CompareTag("Price"))
                    continue;

                // 도로/바닥도 무시
                if (hit.collider.name.Contains("Freeway") || hit.collider.name.Contains("Road") ||
                    hit.collider.name.Contains("Ground") || hit.collider.name.Contains("Curve"))
                    continue;

                if (hit.distance < nearestDistance)
                {
                    nearestDistance = hit.distance;
                    foundRealObstacle = true;
                }
            }

            sensor.AddObservation(foundRealObstacle ? nearestDistance / sensorRange : 1f);
        }

        // 2. 차량 속도 정보 (2개) - Null 체크 추가
        if (carController?.rb != null)
        {
            Vector3 localVelocity = transform.InverseTransformDirection(carController.rb.linearVelocity);
            sensor.AddObservation(Mathf.Clamp(localVelocity.x / 20f, -1f, 1f)); // 좌우 속도
            sensor.AddObservation(Mathf.Clamp(localVelocity.z / 20f, -1f, 1f)); // 전후 속도
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        // 3. 타겟까지의 방향과 거리 (3개)
        if (target != null)
        {
            Vector3 directionToTarget = (target.position - transform.position).normalized;
            Vector3 localTargetDirection = transform.InverseTransformDirection(directionToTarget);

            sensor.AddObservation(localTargetDirection.x); // 타겟의 좌우 방향
            sensor.AddObservation(localTargetDirection.z); // 타겟의 전후 방향

            float distanceToTarget = Vector3.Distance(transform.position, target.position);
            sensor.AddObservation(Mathf.Clamp(distanceToTarget / 100f, 0f, 1f)); // 정규화된 거리
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        // 4. 차량의 각속도 (1개) - Null 체크 추가
        if (carController?.rb != null)
        {
            sensor.AddObservation(Mathf.Clamp(carController.rb.angularVelocity.y / 5f, -1f, 1f));
        }
        else
        {
            sensor.AddObservation(0f);
        }

        // 5. 차선 및 차량 감지 데이터 (5개)
        DetectLaneAndVehiclesSimple(sensor);

        // 총 14개 관찰값: 3 + 2 + 3 + 1 + 5 = 14
    }

    // 간단한 차선 및 차량 감지 함수 (정확히 5개 관찰값 추가)
    private void DetectLaneAndVehiclesSimple(VectorSensor sensor)
    {
        // 1. 좌측 차선 거리
        Vector3 leftDirection = -transform.right;
        if (Physics.Raycast(sensorOrigin.position, leftDirection, out RaycastHit leftHit, 10f, laneLayer))
        {
            sensor.AddObservation(leftHit.distance / 10f);
        }
        else
        {
            sensor.AddObservation(1f);
        }

        // 2. 우측 차선 거리
        Vector3 rightDirection = transform.right;
        if (Physics.Raycast(sensorOrigin.position, rightDirection, out RaycastHit rightHit, 10f, laneLayer))
        {
            sensor.AddObservation(rightHit.distance / 10f);
        }
        else
        {
            sensor.AddObservation(1f);
        }

        // 3. 전방 차량 거리
        if (Physics.Raycast(sensorOrigin.position, transform.forward, out RaycastHit frontHit, laneDetectionRange, vehicleLayer))
        {
            sensor.AddObservation(frontHit.distance / laneDetectionRange);
        }
        else
        {
            sensor.AddObservation(1f);
        }

        // 4. 좌측 차량 거리
        if (Physics.Raycast(sensorOrigin.position, leftDirection, out RaycastHit leftVehicleHit, laneDetectionRange, vehicleLayer))
        {
            sensor.AddObservation(leftVehicleHit.distance / laneDetectionRange);
            isNearVehicle = leftVehicleHit.distance < 8f;
        }
        else
        {
            sensor.AddObservation(1f);
        }

        // 5. 우측 차량 거리
        if (Physics.Raycast(sensorOrigin.position, rightDirection, out RaycastHit rightVehicleHit, laneDetectionRange, vehicleLayer))
        {
            sensor.AddObservation(rightVehicleHit.distance / laneDetectionRange);
            if (rightVehicleHit.distance < 8f) isNearVehicle = true;
        }
        else
        {
            sensor.AddObservation(1f);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Null 체크
        if (carController?.im == null)
        {
            Debug.LogWarning("⚠️ CarController 또는 InputManager가 없습니다!");
            return;
        }

        float rawThrottle = actionBuffers.ContinuousActions[0];
        float rawSteering = actionBuffers.ContinuousActions[1];

        // NaN 값 체크 및 방지
        if (float.IsNaN(rawThrottle) || float.IsInfinity(rawThrottle))
        {
            Debug.LogWarning("⚠️ Throttle 값이 NaN/Infinity! 0으로 대체");
            rawThrottle = 0f;
        }

        if (float.IsNaN(rawSteering) || float.IsInfinity(rawSteering))
        {
            Debug.LogWarning("⚠️ Steering 값이 NaN/Infinity! 0으로 대체");
            rawSteering = 0f;
        }

        // 스로틀 처리 (후진 제한)
        float throttle = Mathf.Clamp(rawThrottle, -0.5f, 1f);

        // 조향 처리
        float steering = ProcessSteeringInput(rawSteering);

        // 브레이크 처리
        bool brake = ShouldBrake(throttle, steering);

        // 추가 NaN 체크
        if (float.IsNaN(throttle)) throttle = 0f;
        if (float.IsNaN(steering)) steering = 0f;

        // AI 입력 로그 (500스텝마다로 줄임)
        if (StepCount % 500 == 0)
        {
            Debug.Log($"[Step {StepCount}] AI Input - Raw Steer: {rawSteering:F2} → Final: {steering:F2}, Throttle: {throttle:F2}, Brake: {brake}");
        }

        // InputManager에 값 전달
        carController.im.throttle = brake ? 0f : throttle;
        carController.im.steer = steering;
        carController.im.brake = brake;

        // 장애물 감지 및 보상 계산
        ProcessObstacleDetection();
        ProcessBrakingBehavior(brake, throttle);
        ProcessLaneFollowing();
        ProcessSteeringRewards(rawSteering, steering);
        CalculateRewards();

        // 종료 조건 체크
        CheckEndConditions();
    }

    private float ProcessSteeringInput(float rawSteering)
    {
        if (useDiscreteSteering)
        {
            // 이산적 조향
            if (rawSteering > 0.2f)
                return discreteSteeringStrength;
            else if (rawSteering < -0.2f)
                return -discreteSteeringStrength;
            else
                return 0f;
        }
        else
        {
            // 연속적 조향 처리
            float processedSteering = rawSteering;

            // 임계값 이하는 0으로 처리
            if (Mathf.Abs(processedSteering) < steeringThreshold)
            {
                processedSteering = 0f;
            }
            else
            {
                // 임계값 이상은 배수 적용하여 강화
                processedSteering *= steeringMultiplier;

                // 부호 유지하면서 최소값 보장
                if (processedSteering > 0f && processedSteering < 0.7f)
                    processedSteering = 0.7f;
                else if (processedSteering < 0f && processedSteering > -0.7f)
                    processedSteering = -0.7f;
            }

            return Mathf.Clamp(processedSteering, -1f, 1f);
        }
    }

    private void ProcessSteeringRewards(float rawSteering, float finalSteering)
    {
        // 강한 조향에 대한 보상
        float steeringIntensity = Mathf.Abs(finalSteering);

        if (steeringIntensity > 0.7f)
        {
            AddReward(rewardForStrongSteering * Time.fixedDeltaTime);

            // 조향 지속 시간 추적
            float currentSteerDirection = Mathf.Sign(finalSteering);

            if (Mathf.Approximately(currentSteerDirection, lastSteerDirection))
            {
                continuousSteerTime += Time.fixedDeltaTime;
            }
            else
            {
                continuousSteerTime = 0f;
            }

            lastSteerDirection = currentSteerDirection;

            // 지속적인 강한 조향에 추가 보상
            if (continuousSteerTime > 1f)
            {
                float bonusReward = 0.02f * (continuousSteerTime - 1f);
                AddReward(bonusReward * Time.fixedDeltaTime);

                if (StepCount % 150 == 0)
                {
                    Debug.Log($"🎯 지속적 강한 조향! {continuousSteerTime:F1}초, 각도: {steeringIntensity * 20:F0}도");
                }
            }
        }
        else
        {
            // 조향 중단 시 리셋
            if (continuousSteerTime > 0.5f)
            {
                AddReward(0.01f);
            }
            continuousSteerTime = 0f;
            lastSteerDirection = 0f;
        }

        // 약한 조향에 약간의 페널티
        if (Mathf.Abs(rawSteering) > 0.1f && Mathf.Abs(finalSteering) < 0.3f)
        {
            AddReward(-0.005f * Time.fixedDeltaTime);
        }
    }

    private void ProcessObstacleDetection()
    {
        bool nearObstacle = false;
        float minObstacleDistance = sensorRange;

        // 전방 장애물 감지
        for (int i = 0; i < sensorCount; i++)
        {
            float angle = (i - sensorCount / 2) * 30f;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            if (Physics.Raycast(sensorOrigin.position, direction, out RaycastHit hit, sensorRange, obstacleLayer))
            {
                // Price 태그는 장애물로 취급하지 않음
                if (hit.collider.CompareTag("Price"))
                    continue;

                float obstacleDistance = hit.distance;
                if (obstacleDistance < minObstacleDistance)
                {
                    minObstacleDistance = obstacleDistance;
                }

                if (obstacleDistance < sensorRange * 0.4f)
                {
                    nearObstacle = true;
                }
            }
        }

        // 장애물 근처에서 강한 조향 시 추가 보상
        if (nearObstacle && carController?.im != null && Mathf.Abs(carController.im.steer) > 0.7f)
        {
            AddReward(0.03f * Time.fixedDeltaTime);

            if (StepCount % 100 == 0)
            {
                Debug.Log($"🚧 장애물 회피 조향! 거리: {minObstacleDistance:F1}m, 조향: {carController.im.steer:F2}");
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var discreteActionsOut = actionsOut.DiscreteActions;

        if (carController?.im != null)
        {
            float rawSteer = carController.im.steer;
            float processedSteer = ProcessSteeringInput(rawSteer);

            continuousActionsOut[0] = carController.im.throttle;
            continuousActionsOut[1] = rawSteer;
            discreteActionsOut[0] = carController.im.brake ? 1 : 0;

            if (Time.frameCount % 300 == 0)
            {
                Debug.Log($"🎮 수동 모드 - Raw Steer: {rawSteer:F2} → Processed: {processedSteer:F2}");
            }
        }
    }

    private void CalculateRewards()
    {
        if (target == null) return;

        // 타겟 방향으로 이동 보상
        float currentDistanceToTarget = Vector3.Distance(transform.position, target.position);

        if (currentDistanceToTarget < previousDistanceToTarget)
        {
            float progress = previousDistanceToTarget - currentDistanceToTarget;
            float reward = rewardForMovingTowardsTarget * progress * Time.fixedDeltaTime;
            AddReward(reward);

            if (progress > 0.5f && StepCount % 50 == 0)
            {
                Debug.Log($"✅ 목표 접근! 진전: {progress:F1}m, 거리: {currentDistanceToTarget:F1}m");
            }
        }
        else
        {
            AddReward(penaltyForMovingAwayFromTarget * 0.5f * Time.fixedDeltaTime);
        }

        previousDistanceToTarget = currentDistanceToTarget;

        // 속도 기반 보상 - Null 체크 추가
        if (carController?.rb != null)
        {
            float speed = carController.rb.linearVelocity.magnitude;

            if (speed < 1f)
            {
                AddReward(-0.02f * Time.fixedDeltaTime);
            }
            else if (speed > 3f && speed < 15f)
            {
                AddReward(0.01f * Time.fixedDeltaTime);
            }

            // 움직임 보상
            if (speed > 0.5f)
            {
                AddReward(0.003f * Time.fixedDeltaTime);
            }
        }
    }

    private void CheckEndConditions()
    {
        // 타겟 도달 확인
        if (target != null && Vector3.Distance(transform.position, target.position) < 2f)
        {
            Debug.Log($"🎯 목표 도달! Step: {StepCount}");
            AddReward(rewardForReachingTarget);

            Debug.Log("🔄 목표 도달로 인한 즉시 리셋 실행");
            ResetCarPosition();

            EndEpisode();
        }

        // 충돌 확인
        if (hasCollided)
        {
            Debug.Log($"💥 충돌! Step: {StepCount}");
            AddReward(penaltyForCollision);
            EndEpisode();
        }

        // 맵 밖으로 떨어짐
        if (transform.position.y < -10f)
        {
            Debug.Log($"🕳️ 맵에서 떨어짐!");
            AddReward(penaltyForCollision);
            EndEpisode();
        }

        // 디버그: 리셋 직전 상황 로그
        if (StepCount > 4950)
        {
            Debug.Log($"⏰ Max Step 근처! Step: {StepCount}/5000");
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log($"🚗 접촉: {other.name}, 태그: {other.tag}");

        // 도로/바닥은 무시
        if (other.name.Contains("Freeway") || other.name.Contains("Road") ||
            other.name.Contains("Ground") || other.name.Contains("Curve"))
        {
            return;
        }

        if (other.CompareTag("Target") || other == target)
            {
                Debug.Log($"🎯 목표 도달! (Trigger) Step: {StepCount}");
                AddReward(rewardForReachingTarget);
                Debug.Log("🔄 목표 도달로 인한 즉시 리셋 실행");
                ResetCarPosition();
                EndEpisode();
                return;
            }


        // Price 태그 오브젝트
        if (other.CompareTag("Price"))
        {
            AddReward(rewardForPriceCollection);
            Debug.Log($"💰 Price 획득! 보상: +{rewardForPriceCollection}");
            other.gameObject.SetActive(false);
            return;
        }

        if (other.CompareTag("Car") || other.CompareTag("Vehicle") || other.CompareTag("AI_Car"))
            {
                Debug.Log($"🚗💥 차량 충돌! 상대방: {other.name}");
                AddReward(penaltyForCollision); // -15 패널티
                Debug.Log($"💀 차량 충돌 패널티: {penaltyForCollision}");

                // 즉시 리셋 후 에피소드 종료
                Debug.Log("🔄 차량 충돌로 인한 즉시 리셋 실행");
                ResetCarPosition();
                EndEpisode();
                return;
            }

        // 장애물 충돌
        if (other.CompareTag("Obstacle") || other.CompareTag("Wall"))
        {
            Debug.Log($"⚠️ 장애물 충돌: {other.name}");
            hasCollided = true;
        }
    }

    private void ResetCarPosition()
    {
        Debug.Log("🔄 차량 리셋");

        // Rigidbody 안전성 체크
        if (carController?.rb == null)
        {
            Debug.LogWarning("⚠️ CarController 또는 Rigidbody가 없습니다!");
            return;
        }

        transform.position = initialPosition;
        transform.rotation = initialRotation;

        // Rigidbody 안전 초기화
        try
        {
            carController.rb.linearVelocity = Vector3.zero;
            carController.rb.angularVelocity = Vector3.zero;
            carController.rb.Sleep();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"⚠️ Rigidbody 리셋 중 에러: {e.Message}");
        }

        // 모든 추적 변수들 리셋
        ResetAllVariables();

        StartCoroutine(WakeUpRigidbody());
    }

    private void ResetAllVariables()
    {
        // 지속적 회전 변수들 리셋
        continuousSteerTime = 0f;
        lastSteerDirection = 0f;

        // 차선 추적 변수들 리셋
        currentLanePosition = 0f;
        isNearVehicle = false;
        isChangingLane = false;
        laneChangeStartTime = 0f;
        timeInLane = 0f;
        lastLaneViolationTime = 0f;
        wasInEmergency = false;

        // 차선 위반 추적 변수들 리셋
        wasInLaneBoundary = true;
        laneViolationStartTime = 0f;
        totalLaneViolations = 0;

        // 브레이킹 관련 변수 리셋
        wasInDanger = false;
        dangerStartTime = 0f;
    }

    private IEnumerator WakeUpRigidbody()
    {
        yield return new WaitForFixedUpdate();
        if (carController?.rb != null)
        {
            carController.rb.WakeUp();
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (sensorOrigin != null)
        {
            // 장애물 감지 Ray (빨간색)
            Gizmos.color = Color.red;
            for (int i = 0; i < sensorCount; i++)
            {
                float angle = (i - sensorCount / 2) * 30f;
                Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
                Gizmos.DrawRay(sensorOrigin.position, direction * sensorRange);
            }

            // 차선 감지 Ray (파란색)
            Gizmos.color = Color.blue;
            Vector3 leftDirection = -transform.right;
            Vector3 rightDirection = transform.right;
            Gizmos.DrawRay(sensorOrigin.position, leftDirection * laneDetectionRange);
            Gizmos.DrawRay(sensorOrigin.position, rightDirection * laneDetectionRange);

            // 차량 감지 Ray (초록색)
            Gizmos.color = Color.green;
            Gizmos.DrawRay(sensorOrigin.position, transform.forward * laneDetectionRange);
        }
    }

    // 차선 추종 처리 함수 (간소화)
    private void ProcessLaneFollowing()
    {
        // 차선 중앙 위치 계산
        UpdateLanePosition();

        // 차선 넘어감 감지
        bool currentlyInLane = Mathf.Abs(currentLanePosition) <= 0.4f;

        // 차선 위반 시작 감지
        if (wasInLaneBoundary && !currentlyInLane)
        {
            totalLaneViolations++;
            laneViolationStartTime = Time.time;
            wasInLaneBoundary = false;

            Debug.Log($"🚨 [{totalLaneViolations}번째] 차선 위반 시작! 위치: {currentLanePosition:F3}");
        }
        // 차선 복귀 감지
        else if (!wasInLaneBoundary && currentlyInLane)
        {
            float violationDuration = Time.time - laneViolationStartTime;
            wasInLaneBoundary = true;
            Debug.Log($"✅ 차선 복귀! 지속시간: {violationDuration:F2}초");
        }

        // 차선 내 시간 누적
        if (currentlyInLane)
        {
            timeInLane += Time.fixedDeltaTime;
        }
        else
        {
            timeInLane = 0f;
        }

        // === 차선 유지 보상 시스템 ===

        // 완벽한 차선 중앙 유지
        if (Mathf.Abs(currentLanePosition) < 0.1f && !isChangingLane)
        {
            float totalReward = laneKeepingReward + perfectLaneReward;
            AddReward(totalReward * Time.fixedDeltaTime);

            // 장기간 완벽 유지 시 보너스
            if (timeInLane > 15f)
            {
                AddReward(superPerfectBonus * Time.fixedDeltaTime);
                if (StepCount % 200 == 0)
                {
                    Debug.Log($"🌟 SUPER PERFECT! {timeInLane:F1}초 완벽 유지!");
                }
            }
        }
        // 매우 좋은 차선 유지
        else if (Mathf.Abs(currentLanePosition) < 0.2f && !isChangingLane)
        {
            float reward = laneKeepingReward + (perfectLaneReward * 0.5f);
            AddReward(reward * Time.fixedDeltaTime);
        }
        // 기본 차선 유지
        else if (Mathf.Abs(currentLanePosition) < 0.4f && !isChangingLane)
        {
            AddReward(laneKeepingReward * Time.fixedDeltaTime);
        }

        // === 차선 변경 관리 ===
        bool isEmergencyNow = IsEmergencySituation();

        // 적절한 차선 변경 시작
        if ((isNearVehicle || isEmergencyNow) && !isChangingLane &&
            carController?.im != null && Mathf.Abs(carController.im.steer) > 0.5f)
        {
            isChangingLane = true;
            laneChangeStartTime = Time.time;

            if (isEmergencyNow)
            {
                AddReward(emergencyLaneChangeReward);
                Debug.Log($"🚨 응급 차선 변경 시작!");
            }
            else
            {
                AddReward(laneChangeReward);
                Debug.Log($"🚗 필요한 차선 변경 시작!");
            }
        }

        // 차선 변경 완료 체크
        if (isChangingLane)
        {
            float changeElapsed = Time.time - laneChangeStartTime;

            if (changeElapsed > 1f && Mathf.Abs(currentLanePosition) < 0.3f)
            {
                isChangingLane = false;

                if (changeElapsed < 3f)
                {
                    AddReward(rapidReturnReward);
                    Debug.Log($"⚡ 빠른 차선 복귀! {changeElapsed:F1}초");
                }
            }
            else if (changeElapsed > 5f)
            {
                AddReward(wrongLaneReward * 0.5f * Time.fixedDeltaTime);
            }
        }

        // === 차선 위반 페널티 ===
        if (Mathf.Abs(currentLanePosition) > 0.7f && !isNearVehicle && !isEmergencyNow && !isChangingLane)
        {
            float violationSeverity = Mathf.Abs(currentLanePosition) - 0.7f;
            float penalty = wrongLaneReward * (1f + violationSeverity * 2f);
            AddReward(penalty * Time.fixedDeltaTime);

            if (StepCount % 100 == 0)
            {
                Debug.Log($"🚫 심각한 차선 위반! 위치: {currentLanePosition:F2}");
            }
        }
        else if (Mathf.Abs(currentLanePosition) > 0.5f && !isChangingLane)
        {
            AddReward(wrongLaneReward * 0.6f * Time.fixedDeltaTime);
        }
        else if (Mathf.Abs(currentLanePosition) > 0.4f && !isChangingLane)
        {
            AddReward(wrongLaneReward * 0.3f * Time.fixedDeltaTime);
        }
    }

    // 응급 상황 판단 함수
    private bool IsEmergencySituation()
    {
        float nearestDistance = GetNearestObstacleDistance();

        if (carController?.rb != null)
        {
            float currentSpeed = carController.rb.linearVelocity.magnitude;
            return nearestDistance < currentSpeed * 1.5f && nearestDistance < 8f;
        }

        return nearestDistance < 5f;
    }

    // 차선 위치 업데이트
    private void UpdateLanePosition()
    {
        Vector3 leftDirection = -transform.right;
        Vector3 rightDirection = transform.right;

        bool leftLaneFound = Physics.Raycast(sensorOrigin.position, leftDirection, out RaycastHit leftHit, 10f, laneLayer);
        bool rightLaneFound = Physics.Raycast(sensorOrigin.position, rightDirection, out RaycastHit rightHit, 10f, laneLayer);

        if (leftLaneFound && rightLaneFound)
        {
            float totalWidth = leftHit.distance + rightHit.distance;
            currentLanePosition = (rightHit.distance - leftHit.distance) / totalWidth;
        }
        else if (leftLaneFound)
        {
            currentLanePosition = Mathf.Lerp(currentLanePosition, 0.5f, Time.fixedDeltaTime);
        }
        else if (rightLaneFound)
        {
            currentLanePosition = Mathf.Lerp(currentLanePosition, -0.5f, Time.fixedDeltaTime);
        }

        currentLanePosition = Mathf.Clamp(currentLanePosition, -1f, 1f);
    }

    // 브레이크 판단 함수
    private bool ShouldBrake(float throttle, float steering)
    {
        if (carController?.rb == null) return false;

        float currentSpeed = carController.rb.linearVelocity.magnitude;
        float nearestObstacleDistance = GetNearestObstacleDistance();

        bool isDangerous = IsDangerousSituation(nearestObstacleDistance, currentSpeed);

        if (isDangerous)
        {
            if (!wasInDanger)
            {
                wasInDanger = true;
                dangerStartTime = Time.time;
            }

            // 매우 위험한 상황
            if (nearestObstacleDistance < currentSpeed * 1.0f)
            {
                if (StepCount % 50 == 0)
                {
                    Debug.Log($"🚨 응급 브레이크! 거리: {nearestObstacleDistance:F1}m");
                }
                return true;
            }

            // 위험한 상황
            if (nearestObstacleDistance < currentSpeed * 2.0f && throttle > 0.3f)
            {
                if (StepCount % 100 == 0)
                {
                    Debug.Log($"⚠️ 안전 브레이크! 거리: {nearestObstacleDistance:F1}m");
                }
                return true;
            }
        }
        else
        {
            if (wasInDanger)
            {
                wasInDanger = false;
                Debug.Log($"✅ 위험 상황 해결!");
            }
        }

        return false;
    }

    // 가장 가까운 장애물까지의 거리 계산
    private float GetNearestObstacleDistance()
    {
        float nearestDistance = sensorRange;

        for (int i = 0; i < sensorCount; i++)
        {
            float angle = (i - sensorCount / 2) * 30f;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            RaycastHit[] hits = Physics.RaycastAll(sensorOrigin.position, direction, sensorRange, obstacleLayer);

            foreach (RaycastHit hit in hits)
            {
                if (hit.collider.CompareTag("Price") ||
                    hit.collider.name.Contains("Freeway") || hit.collider.name.Contains("Road") ||
                    hit.collider.name.Contains("Ground") || hit.collider.name.Contains("Curve"))
                    continue;

                if (hit.distance < nearestDistance)
                {
                    nearestDistance = hit.distance;
                }
            }
        }

        return nearestDistance;
    }

    // 위험 상황 판단
    private bool IsDangerousSituation(float obstacleDistance, float speed)
    {
        float brakingDistance = (speed * speed) / (2 * 8f);
        float safetyMargin = 3f;
        return obstacleDistance < (brakingDistance + safetyMargin);
    }

    // 브레이킹 행동 평가 함수
    private void ProcessBrakingBehavior(bool isBraking, float throttle)
    {
        if (carController?.rb == null) return;

        float currentSpeed = carController.rb.linearVelocity.magnitude;
        float nearestObstacle = GetNearestObstacleDistance();
        bool isDangerous = IsDangerousSituation(nearestObstacle, currentSpeed);

        if (isBraking)
        {
            if (isDangerous)
            {
                if (nearestObstacle < currentSpeed * 1.0f)
                {
                    AddReward(emergencyBrakeReward * Time.fixedDeltaTime);
                    if (StepCount % 100 == 0)
                    {
                        Debug.Log($"🏆 응급 브레이크 보상!");
                    }
                }
                else
                {
                    AddReward(safeBrakingReward * Time.fixedDeltaTime);
                }
            }
            else
            {
                AddReward(-0.01f * Time.fixedDeltaTime);
            }
        }
        else
        {
            if (isDangerous && throttle > 0.5f)
            {
                AddReward(recklessDrivingPenalty * Time.fixedDeltaTime);
                if (StepCount % 100 == 0)
                {
                    Debug.Log($"💀 무모한 운전 페널티!");
                }
            }
        }

        // 충돌 회피 성공 보상
        if (wasInDanger && !IsDangerousSituation(nearestObstacle, currentSpeed))
        {
            AddReward(crashAvoidanceReward);
            Debug.Log($"🎯 충돌 회피 성공!");
        }
    }

    // 에피소드 통계 로그
    private void LogEpisodeStats()
    {
        if (CompletedEpisodes > 0)
        {
            Debug.Log($"📈 에피소드 {CompletedEpisodes} 차선 위반 통계");
            Debug.Log($"🔢 총 위반 횟수: {totalLaneViolations}번");

            if (totalLaneViolations > 0)
            {
                float avgViolationsPerStep = (float)totalLaneViolations / StepCount * 1000f;
                Debug.Log($"📊 위반 빈도: {avgViolationsPerStep:F2}회/1000스텝");

                if (totalLaneViolations > 10)
                    Debug.Log($"🚨 위반이 너무 많습니다!");
                else if (totalLaneViolations < 3)
                    Debug.Log($"🏆 우수한 차선 준수!");
                else
                    Debug.Log($"👍 적당한 차선 준수");
            }
            else
            {
                Debug.Log($"🌟 PERFECT! 차선 위반 0회!");
            }
        }
    }
}