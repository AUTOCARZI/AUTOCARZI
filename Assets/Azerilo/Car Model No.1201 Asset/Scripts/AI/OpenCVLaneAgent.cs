using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Net.Sockets;
using System.Text;
using System;
using System.IO;

public class OpenCVLaneAgent : Agent
{
    [Header("Car Components")]
    public Rigidbody carRigidbody;
    public Transform[] wheels;
    public Camera carCamera;

    [Header("Camera Settings")]
    public int cameraWidth = 640;
    public int cameraHeight = 480;

    [Header("Image Saving Settings")]
    public bool saveImages = true;
    public string saveDirectory = "CameraImages";
    public int saveInterval = 30; // 30프레임마다 저장
    public bool saveAllFrames = false; // 모든 프레임 저장 여부

    [Header("Python Communication")]
    public string pythonServerIP = "127.0.0.1";
    public int pythonServerPort = 65432;
    private TcpClient tcpClient;
    private NetworkStream stream;

    [Header("Car Physics")]
    public float motorForce = 1000f;
    public float steerForce = 30f;
    public float maxSpeed = 20f;

    [Header("Training Parameters")]
    public Transform[] checkpoints;

    // 차선 검출 결과
    private float laneDeviation = 0f; // -1(왼쪽) ~ 1(오른쪽)
    private bool laneDetected = false;
    private float confidence = 0f;

    private int currentCheckpoint = 0;
    private Vector3 lastPosition;
    private Texture2D cameraTexture;

    // 이미지 저장 관련
    private int frameCount = 0;
    private int savedImageCount = 0;
    private string fullSaveDirectory;

    public override void Initialize()
    {
        carRigidbody = GetComponent<Rigidbody>();
        lastPosition = transform.position;

        // 카메라 렌더 텍스처 자동 설정
        if (carCamera != null)
        {
            if (carCamera.targetTexture == null)
            {
                Debug.Log("카메라에 Target Texture가 없어서 자동 생성합니다.");
                RenderTexture renderTexture = new RenderTexture(cameraWidth, cameraHeight, 24);
                renderTexture.format = RenderTextureFormat.RGB565;
                carCamera.targetTexture = renderTexture;
                Debug.Log($"Render Texture 자동 생성: {cameraWidth}x{cameraHeight}");
            }
            else
            {
                Debug.Log($"기존 Target Texture 사용: {carCamera.targetTexture.width}x{carCamera.targetTexture.height}");
            }
        }
        else
        {
            Debug.LogError("Car Camera가 할당되지 않았습니다!");
        }

        cameraTexture = new Texture2D(cameraWidth, cameraHeight, TextureFormat.RGB24, false);

        // 저장 폴더 생성
        SetupSaveDirectory();

        // Python 서버 연결
        ConnectToPythonServer();
    }

    private void SetupSaveDirectory()
    {
        // Unity 프로젝트 폴더 내에 저장 폴더 생성
        fullSaveDirectory = Path.Combine(Application.dataPath, "..", saveDirectory);

        if (!Directory.Exists(fullSaveDirectory))
        {
            Directory.CreateDirectory(fullSaveDirectory);
            Debug.Log($"이미지 저장 폴더 생성: {fullSaveDirectory}");
        }
        else
        {
            Debug.Log($"기존 이미지 저장 폴더 사용: {fullSaveDirectory}");
        }
    }

    private void ConnectToPythonServer()
    {
        try
        {
            tcpClient = new TcpClient(pythonServerIP, pythonServerPort);
            stream = tcpClient.GetStream();
            Debug.Log("Python 서버에 연결됨!");
        }
        catch (Exception e)
        {
            Debug.LogError($"Python 서버 연결 실패: {e.Message}");
        }
    }

    public override void OnEpisodeBegin()
    {
        carRigidbody.linearVelocity = Vector3.zero;
        carRigidbody.angularVelocity = Vector3.zero;

        currentCheckpoint = 0;
        lastPosition = transform.position;
        laneDeviation = 0f;
        laneDetected = false;
        frameCount = 0;

        // 카메라 위치 조정
        //carCamera.transform.localPosition = new Vector3(0, 1.2f, 0.8f);
        //carCamera.transform.localRotation = Quaternion.Euler(10, 0, 0);
    }

    // 테스트용 Update 메서드 추가
    private void Update()
    {
        // 테스트: 5초마다 이미지 캡처 강제 실행
        if (Time.time % 5f < 0.1f && frameCount < 50) // 처음 50프레임만
        {
            Debug.Log("강제 이미지 캡처 테스트 실행!");
            CaptureAndDetectLanes();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 카메라 이미지 캡처 및 차선 검출
        CaptureAndDetectLanes();

        // 차선 검출 결과
        sensor.AddObservation(laneDeviation);
        sensor.AddObservation(laneDetected ? 1f : 0f);
        sensor.AddObservation(confidence);

        // 자동차 상태 정보
        Vector3 localVelocity = transform.InverseTransformDirection(carRigidbody.linearVelocity);
        sensor.AddObservation(localVelocity.x / maxSpeed);
        sensor.AddObservation(localVelocity.z / maxSpeed);
        sensor.AddObservation(transform.rotation.y);

        // 체크포인트 방향
        if (checkpoints != null && checkpoints.Length > 0)
        {
            Vector3 directionToCheckpoint = (checkpoints[currentCheckpoint].position - transform.position).normalized;
            Vector3 localDirection = transform.InverseTransformDirection(directionToCheckpoint);
            sensor.AddObservation(localDirection.x);
            sensor.AddObservation(localDirection.z);
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(1f); // 앞으로 가도록
        }

        // 현재 조향각
        if (wheels.Length > 0)
        {
            sensor.AddObservation(wheels[0].localRotation.y / steerForce);
        }
        else
        {
            sensor.AddObservation(0f);
        }
    }

    private void CaptureAndDetectLanes()
    {
        // 카메라가 설정되어 있는지 확인
        if (carCamera == null || carCamera.targetTexture == null)
        {
            Debug.LogError("카메라 또는 타겟 텍스처가 설정되지 않음!");
            return;
        }

        try
        {
            // 카메라 이미지 캡처
            RenderTexture.active = carCamera.targetTexture;
            cameraTexture.ReadPixels(new Rect(0, 0, cameraWidth, cameraHeight), 0, 0);
            cameraTexture.Apply();
            RenderTexture.active = null;

            // 이미지를 바이트 배열로 변환
            byte[] imageBytes = cameraTexture.EncodeToJPG();

            Debug.Log($"프레임 {frameCount}: 이미지 캡처 완료, 크기: {imageBytes.Length} bytes");

            // Unity에서 이미지 저장 (Python 전송 전에 먼저)
            if (saveImages)
            {
                SaveImageToFile(imageBytes);
            }

            // Python 서버로 이미지 전송 및 결과 수신
            SendImageToPython(imageBytes);
        }
        catch (Exception e)
        {
            Debug.LogError($"이미지 캡처 실패: {e.Message}");
        }

        frameCount++;
    }

    private void SaveImageToFile(byte[] imageBytes)
    {
        // 이미지 데이터 유효성 확인
        if (imageBytes == null || imageBytes.Length == 0)
        {
            Debug.LogError("저장할 이미지 데이터가 없음!");
            return;
        }

        bool shouldSave = false;
        string saveReason = "";

        // 테스트를 위해 처음 몇 프레임은 무조건 저장
        if (frameCount < 10)
        {
            shouldSave = true;
            saveReason = "테스트저장";
        }
        else if (saveAllFrames)
        {
            shouldSave = true;
            saveReason = "모든프레임";
        }
        else if (frameCount % saveInterval == 0)
        {
            shouldSave = true;
            saveReason = "정기저장";
        }

        Debug.Log($"프레임 {frameCount}: 저장 조건 확인 - shouldSave: {shouldSave}, saveImages: {saveImages}, 이미지크기: {imageBytes.Length}");

        if (shouldSave)
        {
            try
            {
                // 폴더 존재 확인
                if (!Directory.Exists(fullSaveDirectory))
                {
                    Directory.CreateDirectory(fullSaveDirectory);
                    Debug.Log($"저장 폴더 재생성: {fullSaveDirectory}");
                }

                // 파일명 생성 (타임스탬프 + 프레임 정보)
                string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss_fff");
                string status = laneDetected ? "detected" : "not_detected";
                string fileName = $"unity_{timestamp}_frame{frameCount:D6}_{status}_dev{laneDeviation:F3}.jpg";
                string filePath = Path.Combine(fullSaveDirectory, fileName);

                Debug.Log($"이미지 저장 시도: {filePath}");

                // 파일 저장
                File.WriteAllBytes(filePath, imageBytes);
                savedImageCount++;

                // 로그 출력
                Debug.Log($"✅ Unity 이미지 저장 성공 #{savedImageCount}: {fileName} ({saveReason})");

                // 파일이 실제로 저장되었는지 확인
                if (File.Exists(filePath))
                {
                    FileInfo fileInfo = new FileInfo(filePath);
                    Debug.Log($"저장된 파일 크기: {fileInfo.Length} bytes, 전체 경로: {Path.GetFullPath(filePath)}");
                }
                else
                {
                    Debug.LogError("파일이 저장되지 않았음!");
                }

                // 3장마다 통계 출력
                if (savedImageCount % 3 == 0)
                {
                    Debug.Log($"=== Unity 이미지 저장 통계 ===");
                    Debug.Log($"총 프레임: {frameCount}, 저장된 이미지: {savedImageCount}");
                    Debug.Log($"차선 검출 상태: {(laneDetected ? "검출됨" : "검출 안됨")}, 편차: {laneDeviation:F3}");
                    Debug.Log($"저장 폴더: {fullSaveDirectory}");
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"❌ Unity 이미지 저장 실패: {e.Message}");
                Debug.LogError($"스택 트레이스: {e.StackTrace}");
            }
        }
    }

    private void SendImageToPython(byte[] imageBytes)
    {
        if (stream == null || !tcpClient.Connected)
        {
            // 연결이 끊어진 경우 재연결 시도
            ConnectToPythonServer();
            return;
        }

        try
        {
            // 이미지 크기 전송
            byte[] sizeBytes = BitConverter.GetBytes(imageBytes.Length);
            stream.Write(sizeBytes, 0, 4);

            // 이미지 데이터 전송
            stream.Write(imageBytes, 0, imageBytes.Length);

            // 결과 수신
            byte[] resultBuffer = new byte[12]; // float 3개 (deviation, detected, confidence)
            int bytesRead = stream.Read(resultBuffer, 0, 12);

            if (bytesRead == 12)
            {
                laneDeviation = BitConverter.ToSingle(resultBuffer, 0);
                laneDetected = BitConverter.ToSingle(resultBuffer, 4) > 0.5f;
                confidence = BitConverter.ToSingle(resultBuffer, 8);

                // Python 결과 로그 (가끔씩만)
                if (frameCount % 60 == 0) // 60프레임마다
                {
                    Debug.Log($"Python 차선 검출 결과 - 편차: {laneDeviation:F3}, 검출: {laneDetected}, 신뢰도: {confidence:F3}");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Python 통신 오류: {e.Message}");
            laneDetected = false;
            confidence = 0f;
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float motor = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
        float steering = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);

        // 자동차 제어
        ApplyMotor(motor);
        ApplySteering(steering);

        // 보상 계산
        CalculateReward();

        // 에피소드 종료 조건 확인
        CheckEpisodeEnd();
    }

    private void ApplyMotor(float motorInput)
    {
        float motor = motorInput * motorForce;

        if (carRigidbody.linearVelocity.magnitude < maxSpeed)
        {
            carRigidbody.AddForce(transform.forward * motor);
        }
    }

    private void ApplySteering(float steeringInput)
    {
        float steering = steeringInput * steerForce;

        // 앞바퀴 회전
        if (wheels.Length >= 2)
        {
            wheels[0].localRotation = Quaternion.Euler(0, steering, 0);
            wheels[1].localRotation = Quaternion.Euler(0, steering, 0);
        }

        // 자동차 회전
        carRigidbody.AddTorque(Vector3.up * steering * carRigidbody.linearVelocity.magnitude);
    }

    private void CalculateReward()
    {
        if (laneDetected)
        {
            // 차선이 검출된 경우
            float centerReward = (1f - Mathf.Abs(laneDeviation)) * confidence;
            AddReward(centerReward * 0.02f);

            // 높은 신뢰도 보상
            AddReward(confidence * 0.01f);

            // 차선 이탈 패널티
            if (Mathf.Abs(laneDeviation) > 0.7f)
            {
                AddReward(-0.1f * confidence);
            }
        }
        else
        {
            // 차선이 검출되지 않은 경우 패널티
            AddReward(-0.05f);
        }

        // 전진 보상
        float forwardDistance = Vector3.Dot(transform.forward, (transform.position - lastPosition).normalized);
        AddReward(forwardDistance * 0.01f);

        // 속도 보상
        float speedRatio = carRigidbody.linearVelocity.magnitude / maxSpeed;
        if (speedRatio > 0.3f && speedRatio < 0.8f)
        {
            AddReward(0.005f);
        }

        // 체크포인트 보상 (체크포인트가 있는 경우만)
        if (checkpoints != null && checkpoints.Length > 0)
        {
            float distanceToCheckpoint = Vector3.Distance(transform.position, checkpoints[currentCheckpoint].position);
            if (distanceToCheckpoint < 5f)
            {
                AddReward(1f);
                currentCheckpoint = (currentCheckpoint + 1) % checkpoints.Length;
            }
        }

        lastPosition = transform.position;
    }

    private void CheckEpisodeEnd()
    {
        // 차선을 완전히 벗어난 경우
        if (laneDetected && Mathf.Abs(laneDeviation) > 0.95f && confidence > 0.7f)
        {
            AddReward(-2f);
            EndEpisode();
        }

        // 속도가 너무 느린 경우
        if (carRigidbody.linearVelocity.magnitude < 0.1f)
        {
            AddReward(-1f);
            EndEpisode();
        }

        // 뒤로 가는 경우
        if (Vector3.Dot(transform.forward, carRigidbody.linearVelocity.normalized) < -0.5f)
        {
            AddReward(-1f);
            EndEpisode();
        }

        // 충돌 감지
        if (Physics.CheckSphere(transform.position, 1f, LayerMask.GetMask("Obstacle")))
        {
            AddReward(-3f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
    }

    private void OnDrawGizmos()
    {
        // 차선 검출 결과 시각화
        if (Application.isPlaying && laneDetected)
        {
            Gizmos.color = confidence > 0.7f ? Color.green : Color.yellow;
            Vector3 deviationPos = transform.position + transform.right * laneDeviation * 2f;
            Gizmos.DrawWireSphere(deviationPos, 0.5f);
        }

        // 체크포인트 표시
        if (checkpoints != null && checkpoints.Length > 0)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(checkpoints[currentCheckpoint].position, 2f);
        }
    }

    // Unity 에디터에서 저장 설정을 실시간으로 확인할 수 있는 정보
    private void OnGUI()
    {
        if (Application.isPlaying)
        {
            GUILayout.BeginArea(new Rect(10, 10, 300, 150));
            GUILayout.Box("Unity 이미지 저장 상태");
            GUILayout.Label($"프레임: {frameCount}");
            GUILayout.Label($"저장된 이미지: {savedImageCount}");
            GUILayout.Label($"차선 검출: {(laneDetected ? "O" : "X")}");
            GUILayout.Label($"편차: {laneDeviation:F3}");
            GUILayout.Label($"신뢰도: {confidence:F3}");
            GUILayout.Label($"저장 폴더: {saveDirectory}");
            GUILayout.EndArea();
        }
    }

    private void OnDestroy()
    {
        // 연결 정리
        if (stream != null)
        {
            stream.Close();
        }
        if (tcpClient != null)
        {
            tcpClient.Close();
        }

        Debug.Log($"세션 종료 - 총 {savedImageCount}장의 이미지가 저장됨");
    }
}