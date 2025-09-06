using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class ScenarioBController : MonoBehaviour
{
    private Rigidbody rb;
    private AutonomousDrivingController autonomousController;
    private HUDManager hudManager;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        autonomousController = GetComponent<AutonomousDrivingController>();
        hudManager = FindFirstObjectByType<HUDManager>();

        if (hudManager == null)
        {
            Debug.LogError("[ScenarioB] HUDManager를 찾을 수 없습니다!");
        }

        if (autonomousController != null)
        {
            autonomousController.SetAutonomousMode(true);
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            if (autonomousController != null && autonomousController.isAutonomousMode)
            {
                autonomousController.SetAutonomousMode(false);
            }

            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

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

            EndGame();
        }
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