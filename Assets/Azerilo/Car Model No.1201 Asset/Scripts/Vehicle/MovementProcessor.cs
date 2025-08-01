using UnityEngine;

public class MovementProcessor : MonoBehaviour
{
    private CarController carController;

    void Start()
    {
        carController = GetComponent<CarController>();
        EventManager.Subscribe<MovementControlEvent>(OnMovementControl);
    }

    void OnDestroy()
    {
        EventManager.Unsubscribe<MovementControlEvent>(OnMovementControl);
    }

    private void OnMovementControl(MovementControlEvent movementEvent)
    {
        ApplyThrottle(movementEvent);
        ApplySteering(movementEvent);
        UpdateWheelMeshes();
    }

    private void ApplyThrottle(MovementControlEvent movementEvent)
    {
        foreach (WheelCollider wheel in carController.throttleWheels)
        {
            if (movementEvent.brake)
            {
                wheel.motorTorque = 0f;
                wheel.brakeTorque = carController.brakeStrength * Time.deltaTime;
            }
            else
            {
                wheel.motorTorque = carController.strengthCoefficient * Time.deltaTime * movementEvent.throttle;
                wheel.brakeTorque = 0f;
            }
        }
    }

    private void ApplySteering(MovementControlEvent movementEvent)
    {
        foreach (GameObject wheel in carController.steeringWheels)
        {
            float steerAngle = carController.maxTurn * movementEvent.steer;
            wheel.GetComponent<WheelCollider>().steerAngle = steerAngle;
            wheel.transform.localEulerAngles = new Vector3(0f, movementEvent.steer * carController.maxTurn, 0f);
        }
    }

    private void UpdateWheelMeshes()
    {
        Rigidbody rb = carController.GetComponent<Rigidbody>();
        foreach (GameObject mesh in carController.meshes)
        {
            mesh.transform.Rotate(rb.linearVelocity.magnitude *
                (carController.transform.InverseTransformDirection(rb.linearVelocity).z >= 0 ? 1 : -1) /
                (2 * Mathf.PI * 0.33f), 0f, 0f);
        }
    }
}