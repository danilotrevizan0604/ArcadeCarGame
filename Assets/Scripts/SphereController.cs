using UnityEngine;

public class SphereRotator : MonoBehaviour
{
    public float turnSpeed = 100f; // Speed of turning
    public float accelerationForce = 10f; // Acceleration force for moving the car
    public float decelerationForce = 20f; // Deceleration force to bring the car to a stop
    public float minimumSpeedToTurn = 0.1f; // Minimum speed at which the car can turn

    private Rigidbody rb; // Reference to the Rigidbody component
    private float horizontalInput;
    private float verticalInput;
    private bool isBraking;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        horizontalInput = Input.GetAxis("Horizontal");
        verticalInput = Input.GetAxis("Vertical");
        isBraking = Input.GetKey(KeyCode.Space);
    }

    void FixedUpdate()
    {
        HandleTurning();
        HandleMovement();
    }

    private void HandleTurning()
    {
        float currentSpeed = rb.velocity.magnitude;
        bool isMoving = currentSpeed > minimumSpeedToTurn;
        float movementDirectionDot = Vector3.Dot(rb.velocity.normalized, transform.forward);

        // Adjust turn direction based on movement direction and only if the car is moving
        if (isMoving && Mathf.Abs(horizontalInput) > 0)
        {
            float turnAdjustment = movementDirectionDot > 0 ? horizontalInput : -horizontalInput;
            transform.Rotate(0, turnAdjustment * turnSpeed * Time.fixedDeltaTime, 0);
        }
    }

    private void HandleMovement()
    {
        if (isBraking)
        {
            // Apply deceleration force to bring the car to a stop
            rb.AddForce(-rb.velocity.normalized * decelerationForce, ForceMode.Acceleration);
        }
        else if (verticalInput > 0) // Accelerate forward
        {
            rb.AddForce(transform.forward * verticalInput * accelerationForce, ForceMode.Acceleration);
        }
        else if (verticalInput < 0) // Reverse or decelerate
        {
            float movementDirectionDot = Vector3.Dot(rb.velocity.normalized, transform.forward);
            if (movementDirectionDot > 0) // Still moving forward, need to decelerate first before reversing
            {
                rb.AddForce(-rb.velocity.normalized * decelerationForce, ForceMode.Acceleration);
            }
            else // Apply reverse acceleration
            {
                rb.AddForce(transform.forward * verticalInput * accelerationForce, ForceMode.Acceleration);
            }
        }
    }
}
