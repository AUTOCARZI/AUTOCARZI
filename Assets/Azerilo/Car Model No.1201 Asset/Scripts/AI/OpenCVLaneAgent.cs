using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Net.Sockets;
using System;

public class OpenCVLaneAgent : Agent
{
    [Header("Camera Settings")]
    public Camera frontCamera;
    public int cameraWidth = 640;
    public int cameraHeight = 480;

    [Header("Python Communication")]
    public string pythonServerIP = "127.0.0.1";
    public int pythonServerPort = 65432;
    private TcpClient tcpClient;
    private NetworkStream stream;

    private float laneDeviation = 0f;
    private bool laneDetected = false;
    private float confidence = 0f;

    private Texture2D frontCameraTexture;
    private int frameCount = 0;

    public override void Initialize()
    {
        Debug.Log("[OpenCVLaneAgent] Initialize 시작");

        SetupCamera(frontCamera, "Front Camera");

        Debug.Log("[OpenCVLaneAgent] 카메라 텍스처 초기화 중");
        frontCameraTexture = new Texture2D(cameraWidth, cameraHeight, TextureFormat.RGB24, false);

        Debug.Log("[OpenCVLaneAgent] Python 서버 연결 시도 중");
        ConnectToPythonServer();

        Debug.Log("[OpenCVLaneAgent] 초기화 완료");
    }

    private void SetupCamera(Camera camera, string cameraName)
    {
        if (camera != null)
        {
            if (camera.targetTexture == null)
            {
                Debug.Log($"{cameraName}에 Target Texture가 없어서 자동 생성합니다.");
                RenderTexture renderTexture = new RenderTexture(cameraWidth, cameraHeight, 24);
                renderTexture.format = RenderTextureFormat.RGB565;
                camera.targetTexture = renderTexture;
                Debug.Log($"{cameraName} Render Texture 자동 생성: {cameraWidth}x{cameraHeight}");
            }
            else
            {
                Debug.Log($"{cameraName} 기존 Target Texture 사용: {camera.targetTexture.width}x{camera.targetTexture.height}");
            }
        }
        else
        {
            Debug.LogError($"{cameraName}가 할당되지 않았습니다!");
        }
    }

    private void ConnectToPythonServer()
    {
        try
        {
            Debug.Log($"[OpenCVLaneAgent] Python 서버 연결 시도: {pythonServerIP}:{pythonServerPort}");
            tcpClient = new TcpClient(pythonServerIP, pythonServerPort);
            stream = tcpClient.GetStream();
            Debug.Log("[OpenCVLaneAgent] Python 서버에 연결됨!");
        }
        catch (Exception e)
        {
            Debug.LogError($"[OpenCVLaneAgent] Python 서버 연결 실패: {e.Message}");
        }
    }

    public override void OnEpisodeBegin()
    {
        laneDeviation = 0f;
        laneDetected = false;
        frameCount = 0;

        Debug.Log("[OpenCVLaneAgent] 새 에피소드 시작");
    }

    void Update()
    {
        Debug.Log($"[OpenCVLaneAgent] Update 실행 - Frame: {frameCount}");
        CaptureAndDetectLanes();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        CaptureAndDetectLanes();

        sensor.AddObservation(laneDeviation);
        sensor.AddObservation(laneDetected ? 1f : 0f);
        sensor.AddObservation(confidence);
    }

    private void CaptureAndDetectLanes()
    {
        try
        {
            byte[] frontImageBytes = CaptureCamera(frontCamera, frontCameraTexture, "Front");

            if (frontImageBytes != null)
            {
                if (frameCount == 0)
                {
                    Debug.Log($"이미지 캡처 성공 - Front: {frontImageBytes.Length} bytes");
                }
                SendImageToPython(frontImageBytes);
            }
            else
            {
                Debug.LogWarning("이미지 캡처 실패");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"이미지 캡처 실패: {e.Message}");
        }

        frameCount++;
    }

    private byte[] CaptureCamera(Camera camera, Texture2D texture, string cameraName)
    {
        if (camera == null || camera.targetTexture == null)
        {
            Debug.LogError($"{cameraName} 카메라 또는 타겟 텍스처가 설정되지 않음!");
            return null;
        }

        try
        {
            RenderTexture.active = camera.targetTexture;
            texture.ReadPixels(new Rect(0, 0, cameraWidth, cameraHeight), 0, 0);
            texture.Apply();
            RenderTexture.active = null;

            byte[] imageBytes = texture.EncodeToJPG();
            return imageBytes;
        }
        catch (Exception e)
        {
            Debug.LogError($"{cameraName} 카메라 캡처 실패: {e.Message}");
            return null;
        }
    }

    private void SendImageToPython(byte[] imageBytes)
    {
        if (stream == null || !tcpClient.Connected)
        {
            Debug.LogWarning("[OpenCVLaneAgent] 연결 끊어짐. 재연결 시도");
            ConnectToPythonServer();
            return;
        }

        try
        {
            Debug.Log($"[OpenCVLaneAgent] Python으로 이미지 전송 - Size: {imageBytes.Length}B");

            byte[] sizeBytes = BitConverter.GetBytes(imageBytes.Length);
            stream.Write(sizeBytes, 0, 4);
            stream.Write(imageBytes, 0, imageBytes.Length);

            Debug.Log("[OpenCVLaneAgent] 이미지 전송 완료. 응답 대기");

            byte[] resultBuffer = new byte[12];
            int bytesRead = stream.Read(resultBuffer, 0, 12);

            if (bytesRead == 12)
            {
                laneDeviation = BitConverter.ToSingle(resultBuffer, 0);
                laneDetected = BitConverter.ToSingle(resultBuffer, 4) > 0.5f;
                confidence = BitConverter.ToSingle(resultBuffer, 8);

                Debug.Log($"[OpenCVLaneAgent] Python 응답 - 편차: {laneDeviation:F3}, 검출: {laneDetected}, 신뢰도: {confidence:F3}");
            }
            else
            {
                Debug.LogError($"[OpenCVLaneAgent] 잘못된 응답 크기: {bytesRead} bytes");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[OpenCVLaneAgent] Python 통신 오류: {e.Message}");
            laneDetected = false;
            confidence = 0f;
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (laneDetected && confidence > 0.5f)
        {
            AddReward(0.01f * confidence);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
    }

    private void OnDrawGizmos()
    {
        if (Application.isPlaying && laneDetected)
        {
            Gizmos.color = confidence > 0.7f ? Color.green : Color.yellow;
            Vector3 deviationPos = transform.position + transform.right * laneDeviation * 2f;
            Gizmos.DrawWireSphere(deviationPos, 0.5f);
        }
    }

    private void OnDestroy()
    {
        if (stream != null) stream.Close();
        if (tcpClient != null) tcpClient.Close();
        Debug.Log($"[OpenCVLaneAgent] 세션 종료 - {frameCount} 프레임 처리");
    }
}
