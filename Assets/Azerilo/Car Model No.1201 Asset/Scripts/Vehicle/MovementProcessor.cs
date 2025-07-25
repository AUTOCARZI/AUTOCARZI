using UnityEngine;

[RequireComponent(typeof(CarController))]
public class MovementProcessor : MonoBehaviour
{
    private CarController carController;

    void Start()
    {
        // CarController 안전하게 획득
        carController = GetComponent<CarController>();
        if (carController == null)
        {
            Debug.LogError($"MovementProcessor: {gameObject.name}에 CarController가 없습니다!");
            return;
        }

        // 이벤트 구독
        EventManager.Subscribe<MovementControlEvent>(OnMovementControl);
        Debug.Log("MovementProcessor 초기화 완료");
    }

    void OnDestroy()
    {
        EventManager.Unsubscribe<MovementControlEvent>(OnMovementControl);
    }

    private void OnMovementControl(MovementControlEvent movementEvent)
    {
        // carController null 체크
        if (carController == null)
        {
            Debug.LogError("MovementProcessor: carController가 null입니다!");
            return;
        }

        // throttleWheels null 체크
        if (carController.throttleWheels == null || carController.throttleWheels.Count == 0)
        {
            Debug.LogError("MovementProcessor: throttleWheels가 설정되지 않았습니다!");
            return;
        }

        // steeringWheels null 체크
        if (carController.steeringWheels == null || carController.steeringWheels.Count == 0)
        {
            Debug.LogError("MovementProcessor: steeringWheels가 설정되지 않았습니다!");
            return;
        }

        try
        {
            // Apply throttle
            foreach (WheelCollider wheel in carController.throttleWheels)
            {
                if (wheel == null) continue;

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

            // Apply steering
            foreach (GameObject wheel in carController.steeringWheels)
            {
                if (wheel == null) continue;

                WheelCollider wheelCollider = wheel.GetComponent<WheelCollider>();
                if (wheelCollider == null) continue;

                wheelCollider.steerAngle = carController.maxTurn * movementEvent.steer;
                wheel.transform.localEulerAngles = new Vector3(0f, movementEvent.steer * carController.maxTurn, 0f);
            }

            // Update wheel meshes
            if (carController.meshes != null)
            {
                Rigidbody rb = carController.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    foreach (GameObject mesh in carController.meshes)
                    {
                        if (mesh == null) continue;

                        mesh.transform.Rotate(rb.linearVelocity.magnitude *
                            (carController.transform.InverseTransformDirection(rb.linearVelocity).z >= 0 ? 1 : -1) /
                            (2 * Mathf.PI * 0.33f), 0f, 0f);
                    }
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"MovementProcessor 오류: {e.Message}");
        }
    }
}