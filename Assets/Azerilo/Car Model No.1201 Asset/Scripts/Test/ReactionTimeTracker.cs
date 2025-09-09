using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using System;
using System.IO;
using System.Text;


[Serializable]
public class ReactionRecord
{
    public float startTime;
    public float responseTime;
    public bool hasResponse;
    public string eventType;
    public string sceneName;
    
    public ReactionRecord(float start, string type = "", string scene = "")
    {
        startTime = start;
        responseTime = -1f;
        hasResponse = false;
        eventType = type;
        sceneName = scene;
    }
    
    public float GetReactionTime()
    {
        return hasResponse ? (responseTime - startTime) : -1f;
    }
}

public class ReactionTimeTracker : MonoBehaviour
{
    [Header("Settings")]
    public string saveFileName = "ReactionTime_Records";
    
    [Header("Debug")]
    [SerializeField] private List<ReactionRecord> allRecords = new List<ReactionRecord>();
    [SerializeField] private List<string> recordedEventTypes = new List<string>();
    
    private float sessionStartTime;
    
    void Start()
    {
        sessionStartTime = Time.time;
        Debug.Log("[ReactionTimeTracker] Initialized");
    }
    
    void Update()
    {
        Keyboard keyboard = Keyboard.current;
        
        if (keyboard != null && keyboard.enterKey.wasPressedThisFrame)
        {
            Debug.Log("[ReactionTimeTracker] Enter key detected!");
            RecordEnterTime();
        }
    }
    
    // 이벤트 시작 - 첫 번째만 기록
    public void StartEvent(string eventType = "")
{
    if (recordedEventTypes.Contains(eventType))
    {
        Debug.Log($"[ReactionTimeTracker] Event type '{eventType}' already recorded, skipping");
        return;
    }
    
    float currentTime = Time.time;
    string currentScene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name;
    ReactionRecord record = new ReactionRecord(currentTime, eventType, currentScene);
    allRecords.Add(record);
    recordedEventTypes.Add(eventType);
}

    public void SaveRecordsToFile(string customFileName = null)
{
    try
    {
        string fileName = customFileName ?? saveFileName;
        string fullFileName = $"{fileName}.txt";  // 항상 같은 파일에 저장
        string filePath = Path.Combine(Application.persistentDataPath, fullFileName);

        StringBuilder sb = new StringBuilder();

        foreach (var record in allRecords)
        {
            if (!record.hasResponse) continue; // 응답 없는 건 건너뛰기

            string responseTime = record.responseTime.ToString("F3");
            string reactionTime = record.GetReactionTime() > 0 ? record.GetReactionTime().ToString("F3") : "N/A";
            string hasResponse = record.hasResponse ? "Yes" : "No";

            sb.AppendLine($"\t{record.eventType}\t{record.sceneName}\t{record.startTime:F3}\t{responseTime}\t{reactionTime}\t{hasResponse}");
        }

        // 맨 밑줄에 추가
        File.AppendAllText(filePath, sb.ToString(), Encoding.UTF8);
        Debug.Log($"[ReactionTimeTracker] Records appended to: {filePath}");

#if UNITY_EDITOR
        string editorPath = Path.Combine(Application.dataPath, "ReactionTime_Records");
        if (!Directory.Exists(editorPath))
        {
            Directory.CreateDirectory(editorPath);
        }
        string editorFilePath = Path.Combine(editorPath, fullFileName);
        File.AppendAllText(editorFilePath, sb.ToString(), Encoding.UTF8);
        UnityEditor.AssetDatabase.Refresh();
#endif
    }
    catch (Exception e)
    {
        Debug.LogError($"[ReactionTimeTracker] SAVE FAILED: {e.Message}");
    }
}


    private void RecordEnterTime()
{
    float currentTime = Time.time;
    Debug.Log($"[ReactionTimeTracker] Enter pressed at: {currentTime:F3}s");

    for (int i = allRecords.Count - 1; i >= 0; i--)
    {
        if (!allRecords[i].hasResponse)
        {
            allRecords[i].responseTime = currentTime;
            allRecords[i].hasResponse = true;

            float reactionTime = allRecords[i].GetReactionTime();

            AppendRecordToFile(allRecords[i]);
            break;
        }
    }
}

private void AppendRecordToFile(ReactionRecord record)
{
    try
    {
        string fileName = "UserData.txt";   // 항상 같은 파일
        string filePath = Path.Combine(Application.persistentDataPath, fileName);

        string responseTime = record.responseTime.ToString("F3");
        string reactionTime = record.GetReactionTime() > 0 ? record.GetReactionTime().ToString("F3") : "N/A";
        string hasResponse = record.hasResponse ? "Yes" : "No";

        string line =
            $"{record.sceneName}\t" +
            $"{record.startTime:F3}\t" +
            $"{responseTime}\t" +
            $"{reactionTime}\t" +
            $"{hasResponse}\n";

        File.AppendAllText(filePath, line, Encoding.UTF8);
        Debug.Log($"[ReactionTimeTracker] Record appended to: {filePath}");

#if UNITY_EDITOR
        string editorPath = Path.Combine(Application.dataPath, "ReactionTime_Records");
        if (!Directory.Exists(editorPath))
        {
            Directory.CreateDirectory(editorPath);
        }
        string editorFilePath = Path.Combine(editorPath, fileName);
        File.AppendAllText(editorFilePath, line, Encoding.UTF8);
        UnityEditor.AssetDatabase.Refresh();
#endif
    }
    catch (Exception e)
    {
        Debug.LogError($"[ReactionTimeTracker] APPEND FAILED: {e.Message}");
    }
}



}