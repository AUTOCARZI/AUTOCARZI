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
        "Scenes/A/RoadScene-Scenario-A",
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

    void Awake()
    {
        if (instance == null)
        {
            instance = this;
            DontDestroyOnLoad(gameObject);
            InitializeSceneCycle();
        }
        else
        {
            Destroy(gameObject);
        }
    }


    void InitializeSceneCycle()
    {
        string currentSceneName = SceneManager.GetActiveScene().name;

        for (int i = 0; i < sceneOrder.Count; i++)
        {
            // 씬 이름으로 매칭 (경로의 파일명 부분과 비교)
            if (sceneOrder[i].Contains(currentSceneName))
            {
                currentSceneIndex = i;
                break;
            }
        }

        if (currentSceneIndex == -1)
        {
            currentSceneIndex = -1;
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            LoadNextScene();
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            TogglePause();
        }
    }

    void LoadNextScene()
    {
        currentSceneIndex++;

        if (currentSceneIndex >= sceneOrder.Count)
        {
            EndGame();
            return;
        }

        string scenePath = sceneOrder[currentSceneIndex];


        if (isPaused)
        {
            Time.timeScale = 1f;
            isPaused = false;
        }

        // Build Settings의 경로 그대로 사용
        SceneManager.LoadScene(scenePath);
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
        if (Time.timeScale != 1f)
        {
            Time.timeScale = 1f;
        }
    }
}