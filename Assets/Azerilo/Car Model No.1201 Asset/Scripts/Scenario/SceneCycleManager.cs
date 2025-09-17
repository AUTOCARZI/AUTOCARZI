using UnityEngine;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class SceneCycleManager : MonoBehaviour
{
    private static SceneCycleManager instance;

    // Build Settings의 경로 형식에 맞춰서 설정
    // 무작위로 섞은 24개 씬 순서 (중복 없이 모든 씬 포함)
    private readonly List<string> sceneOrder = new List<string>
    {
        "Scenes/A/RoadScene-Scenario-A",
        "Scenes/B/Rain/RoadScene-Scenario-B-H",
        "Scenes/D/RoadScene-Scenario-D-L",
        "Scenes/C/Rain/RoadScene-Scenario-C",
        "Scenes/A/RoadScene-Scenario-A-H",
        "Scenes/D/Rain/RoadScene-Scenario-D-H",
        "Scenes/B/RoadScene-Scenario-B",
        "Scenes/C/RoadScene-Scenario-C-L",
        "Scenes/A/Rain/RoadScene-Scenario-A-L",
        "Scenes/D/RoadScene-Scenario-D",
        "Scenes/B/Rain/RoadScene-Scenario-B-L",
        "Scenes/C/Rain/RoadScene-Scenario-C-H",
        "Scenes/D/Rain/RoadScene-Scenario-D-L",
        "Scenes/B/RoadScene-Scenario-B-H",
        "Scenes/C/RoadScene-Scenario-C",
        "Scenes/A/Rain/RoadScene-Scenario-A",
        "Scenes/D/RoadScene-Scenario-D-H",
        "Scenes/B/Rain/RoadScene-Scenario-B",
        "Scenes/C/Rain/RoadScene-Scenario-C-L",
        "Scenes/A/RoadScene-Scenario-A-L",
        "Scenes/D/Rain/RoadScene-Scenario-D",
        "Scenes/B/RoadScene-Scenario-B-L",
        "Scenes/C/RoadScene-Scenario-C-H",
        "Scenes/A/Rain/RoadScene-Scenario-A-H",
    };

    private int currentSceneIndex = -1;
    private bool isPaused = false;
    private bool isLoadingScene = false;
    private float lastSceneLoadTime = 0f;
    private const float sceneLoadCooldown = 0.5f;  // 씬 로드 최소 간격

    void Awake()
    {
        if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(gameObject);
            InitializeSceneCycle();
        }
        else if (instance != this)
        {
            Destroy(gameObject);
        }
    }

    void InitializeSceneCycle()
    {
        string currentSceneName = SceneManager.GetActiveScene().name;
        currentSceneIndex = -1;

        for (int i = 0; i < sceneOrder.Count; i++)
        {
            // 정확한 씬 이름 매칭
            string sceneNameFromPath = System.IO.Path.GetFileNameWithoutExtension(sceneOrder[i]);
            if (sceneNameFromPath == currentSceneName)
            {
                currentSceneIndex = i;
                break;
            }
        }

        if (currentSceneIndex == -1)
        {
            currentSceneIndex = 0;
        }

        // 씬 로드 완료 이벤트 등록
        SceneManager.sceneLoaded += OnSceneLoaded;
    }

    void Update()
    {
        // 싱글톤 체크
        if (instance != this)
        {
            Destroy(gameObject);
            return;
        }

        // Tab 키: 씬 전환
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            if (!isLoadingScene && Time.time - lastSceneLoadTime > sceneLoadCooldown)
            {
                lastSceneLoadTime = Time.time;
                LoadNextScene();
            }
        }

        // Enter 키: 일시정지 토글
        if (Input.GetKeyDown(KeyCode.Return))
        {
            TogglePause();
        }
    }

    void OnTriggerEnter(Collider other)
    {
        // 특정 태그를 가진 오브젝트가 트리거에 들어왔을 때 일시정지
        if (other.CompareTag("Player"))
        {
            TogglePause();
        }
    }

    void LoadNextScene()
    {
        if (isLoadingScene) return;

        currentSceneIndex++;

        if (currentSceneIndex >= sceneOrder.Count)
        {
            EndGame();
            return;
        }

        string scenePath = sceneOrder[currentSceneIndex];
        isLoadingScene = true;

        // 씬 전환 시 일시정지 해제
        if (isPaused)
        {
            isPaused = false;
            Time.timeScale = 1f;
        }

        SceneManager.LoadSceneAsync(scenePath);
    }

    void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        isLoadingScene = false;
    }

    void TogglePause()
    {
        isPaused = !isPaused;
        Time.timeScale = isPaused ? 0f : 1f;
    }

    void EndGame()
    {
#if UNITY_EDITOR
        EditorApplication.isPlaying = false;
#else
        Application.Quit();
#endif
    }

    void OnDestroy()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;

        if (instance == this)
        {
            instance = null;
        }

        if (Time.timeScale != 1f)
        {
            Time.timeScale = 1f;
        }
    }
}