using UnityEngine;
using System.Collections;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class ScenarioBController : MonoBehaviour
{
    [Header("RainMode")]
    public bool isRainMode = false;

    private Rigidbody rb;
    private AutonomousDrivingController autonomousController;
    private HUDManager hudManager;
    private bool isWallHit = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        autonomousController = GetComponent<AutonomousDrivingController>();
        hudManager = FindFirstObjectByType<HUDManager>();

        if (hudManager == null)
        {
            Debug.LogError("[ScenarioB] HUDManager를 찾을 수 없습니다!");
        }
    }

    void Update()
    {
        if (isRainMode && !isWallHit)
        {
            float steerInput = 0f;
            var movementEvent = new MovementControlEvent(0.4f, steerInput, false);
            EventManager.Publish(movementEvent);
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall") && !isWallHit)
        {
            isWallHit = true; // Stop the forward movement
            
            if (autonomousController != null && autonomousController.isAutonomousMode)
            {
                autonomousController.SetAutonomousMode(false);
            }

            // Stop the car
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            
            // Send stop command
            var stopEvent = new MovementControlEvent(0f, 0f, true);
            EventManager.Publish(stopEvent);

            Destroy(other.gameObject);
        }
        else if (other.CompareTag("Finish Point"))
        {
            if (autonomousController != null && autonomousController.isAutonomousMode)
            {
                autonomousController.SetAutonomousMode(false);
            }

            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            
            // Send stop command
            var stopEvent = new MovementControlEvent(0f, 0f, true);
            EventManager.Publish(stopEvent);

            EndGame();
        }
    }

    // Add a public method to enable rain mode
    public void SetRainMode(bool enabled)
    {
        isRainMode = enabled;
        Debug.Log($"[ScenarioB] Rain mode: {enabled}");
    }

    void EndGame()
    {
#if UNITY_EDITOR
        EditorApplication.isPlaying = false;
#else
        Application.Quit();
#endif
    }
}