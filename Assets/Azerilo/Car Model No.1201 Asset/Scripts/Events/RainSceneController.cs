using UnityEngine;
using UnityEngine.SceneManagement;

public class RainSceneController : MonoBehaviour
{
    [Header("Rain Event Settings")]
    public bool enableAutoRainEvent = true;
    public float delayBeforeEvent = 2.0f; // 씬 로딩 후 대기 시간
    
    private bool rainEventActive = false;
    
    void Start()
    {
        if (enableAutoRainEvent && IsRainScene())
        {
            Debug.Log("[RainSceneController] Rain scene detected, triggering rain-slowing-down event immediately");
            // 한 프레임 대기 후 실행하여 HUDManager가 초기화되도록 함
            StartCoroutine(TriggerRainEventAfterFrame());
        }
    }
    
    private System.Collections.IEnumerator TriggerRainEventAfterFrame()
    {
        yield return null; // 한 프레임 대기
        TriggerRainSlowingDownEvent();
    }
    
    void OnDestroy()
    {
        if (rainEventActive)
        {
            // 씬 종료 시 이벤트 정리
            EventManager.Publish(new HUDControlEvent("rain-slowing-down", false));
            Debug.Log("[RainSceneController] Rain scene ending, stopping rain-slowing-down event");
        }
    }
    
    private bool IsRainScene()
    {
        string currentSceneName = SceneManager.GetActiveScene().name;
        string scenePath = SceneManager.GetActiveScene().path;
        
        // Rain 폴더에 있는 씬인지 확인
        bool isInRainFolder = scenePath.Contains("/Rain/");
        
        Debug.Log($"[RainSceneController] Current scene: {currentSceneName}, Path: {scenePath}, Is Rain Scene: {isInRainFolder}");
        
        return isInRainFolder;
    }
    
    private void TriggerRainSlowingDownEvent()
    {
        EventManager.Publish(new HUDControlEvent("rain-slowing-down", true));
        rainEventActive = true;
        Debug.Log("[RainSceneController] Rain-slowing-down event triggered for Rain scene");
    }
    
    public void StopRainEvent()
    {
        if (rainEventActive)
        {
            EventManager.Publish(new HUDControlEvent("rain-slowing-down", false));
            rainEventActive = false;
            Debug.Log("[RainSceneController] Rain-slowing-down event manually stopped");
        }
    }
    
    public void StartRainEvent()
    {
        if (!rainEventActive && IsRainScene())
        {
            TriggerRainSlowingDownEvent();
        }
    }
}