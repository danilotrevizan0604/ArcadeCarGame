using UnityEngine;

public class RayCastWheel : MonoBehaviour
{

    Rigidbody rb;
    private float[] currentSteerAngles;

    [Header("Wheel Transforms")]
    public Transform[] wheel;

    // Suspension
    [Header("Suspension")]
    public float springStrength;
    public float springRestLength;
    public float wheelRadius;
    public float dampStrength;

    // Motor
    [Header("Motor")]
    public float accelerationForce;
    public float carTopSpeed;
    public AnimationCurve powerCurve;
    public float downforceCoefficient = 10f;
    public float maxDownforce = 50f;

    // Steering
    [Header("Steering")]
    public float maxSteerAngle = 30f;
    public float wheelBase = 2.5f;
    public float wheelWidth = 1.6f;
    public float steerVel;
    public float turnSmoothTime = 0.1f;
    public float baseMaxSteerAngle = 30f; // The max steering angle at low speed or stationary
    public float speedSensitivity = 0.1f; // Determines how much the steer angle decreases with speed
    float maxLateralForce = 10f;


    // Tire
    [Header("Tire")]
    public float tireGripFactor;
    float tireMass;

    float maxDistance;
    float accelInput;
    float steerInput;
    

    //Main Unity Functions
    void Start()
    {
        InitializeCar();
        currentSteerAngles = new float[wheel.Length];
    }

    void FixedUpdate()
    {
        for (int i = 0; i < wheel.Length; i++)
        {
            ProcessWheel(i);
            
        }
        ProcessSteering(); // Process steering based on input


    }

    void InitializeCar()
    {
        rb = GetComponent<Rigidbody>();
        tireMass = rb.mass / 4;
        maxDistance = wheelRadius + springRestLength; // Ensure this is set correctly
        
    }

    void ProcessWheel(int i)
    {
        RaycastHit hit;
        bool isGrounded = Physics.Raycast(wheel[i].transform.position, Vector3.down, out hit, maxDistance);

        if (isGrounded && hit.collider.CompareTag("Ground"))
        {
            ApplySuspensionForce(i, hit);
            ApplySteeringForce(i, hit);
            ApplyAccelerationForce(i, hit);
            
            // Optionally, apply any other forces related to the wheel here
        }
    }

    void ApplySuspensionForce(int i, RaycastHit hit)
    {
        // Calculate the spring distance offset
        float offset = maxDistance - hit.distance;
        float springForce = offset * springStrength;

        // Calculate wheel velocity for damping
        Vector3 wheelVelo = rb.GetPointVelocity(wheel[i].position);
        Vector3 springDir = Vector3.up; // Assuming the suspension always pushes up against gravity

        // Damping force calculation
        float dampForce = Vector3.Dot(wheelVelo, springDir) * dampStrength;

        // Final suspension force
        float suspensionForce = (springForce - dampForce) * Time.deltaTime;

        // Apply the force at the wheel's position
        rb.AddForceAtPosition(springDir * suspensionForce, wheel[i].position);

        // Optional: Visualize the suspension force for debugging
        Debug.DrawRay(wheel[i].position, springDir * suspensionForce, Color.green);
    }

    void ProcessSteering()
    {
        steerInput = Input.GetAxis("Horizontal"); // Get steering input (-1 to 1)

        float currentSpeed = rb.velocity.magnitude;

        float currentMaxSteerAngle = Mathf.Max(baseMaxSteerAngle - (currentSpeed * speedSensitivity), 10f); // Ensures there's always some ability to steer, clamped at 10 degrees
        
        // Calculate turn radius based on steering input
        // Note: Mathf.Tan returns a value in radians, and maxSteerAngle * steerInput is in degrees.
        // Mathf.Deg2Rad is used to convert degrees to radians for the Mathf.Tan function.
        // Use 'currentMaxSteerAngle' to calculate the turn radius and steering angles
        float turnRadius = wheelBase / Mathf.Tan(Mathf.Deg2Rad * currentMaxSteerAngle * steerInput);

        // Calculate steering angles for left and right wheels based on Ackermann steering principle
        float leftWheelSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius - (wheelWidth / 2)));
        float rightWheelSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius + (wheelWidth / 2)));

        // Apply calculated steering angles to the front wheels
        // wheel[0] is the left front wheel, and wheel[1] is the right front wheel
        
            ApplySteering(0, leftWheelSteerAngle);
            ApplySteering(1, rightWheelSteerAngle);
        
    }

    void ApplySteering(int i, float targetSteerAngle)
    {

        // Smoothly damp the angle from current to target
        currentSteerAngles[i] = Mathf.SmoothDampAngle(currentSteerAngles[i], targetSteerAngle, ref steerVel, turnSmoothTime);

        // Apply the smoothed angle to the wheel's rotation
        wheel[i].localEulerAngles = new Vector3(wheel[i].localEulerAngles.x, currentSteerAngles[i], wheel[i].localEulerAngles.z);
    }


    void ApplySteeringForce(int i, RaycastHit hit)
    {
        Vector3 wheelVelo = rb.GetPointVelocity(wheel[i].position);

        //world-space direction of the spring force
        Vector3 steeringDir = wheel[i].transform.right;

        //calculate the velocity in the direction os sliding
        float steeringVel = Vector3.Dot(steeringDir, wheelVelo);
        
        //change in velocity that we would like to control
        float desiredVelChange = -steeringVel * tireGripFactor;

        // turn change velocity into acceleration
        float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

        // Calculate the lateral force
        float lateralForceMagnitude = tireMass * desiredAccel;
        
        lateralForceMagnitude = Mathf.Min(lateralForceMagnitude, maxLateralForce);

        rb.AddForceAtPosition(steeringDir * lateralForceMagnitude, wheel[i].position, ForceMode.Force);

        // Debugging
        Debug.Log($"Wheel {i}: DirMult {desiredVelChange}, LateralForce {lateralForceMagnitude}");

        // Visualize the force direction and magnitude
        // The ray length is scaled by a factor (e.g., 0.1) for better visibility
        Debug.DrawRay(wheel[i].position, steeringDir * lateralForceMagnitude * 0.1f, Color.red);

        // Optionally, visualize the wheel velocity direction
        Debug.DrawRay(wheel[i].position, wheelVelo.normalized * 2f, Color.blue);
    }

    bool ShouldApplyAcceleration(int i)
    {
        // Your logic to determine if acceleration should be applied to this wheel
        // For example, return true only for wheels that are part of the drive train
        return true; // Placeholder: adjust based on your vehicle setup
    }

    void ApplyAccelerationForce(int i, RaycastHit hit)
    {
        // Check if this wheel should contribute to acceleration/braking
        if (ShouldApplyAcceleration(i))
        {
            // Calculate acceleration force based on user input, car's current speed, etc.
            // Use hit and wheel[i] information if needed, for example, to adjust force based on wheel position
            Vector3 accelDir = wheel[i].transform.forward;
            accelInput = Input.GetAxisRaw("Vertical") * accelerationForce;
            float carSpeed = Vector3.Dot(transform.forward, rb.velocity);
            float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
            float availableTorque = powerCurve.Evaluate(normalizedSpeed) * accelInput;

            rb.AddForceAtPosition(accelDir * availableTorque, wheel[i].position);

            ApplyDownforce(carSpeed);
        }
    }

    private void ApplyDownforce(float speed)
    {
        // Calculate downforce based on current speed (squared for a more noticeable effect at high speeds, if desired)
        float downforceValue = Mathf.Min(speed * speed * downforceCoefficient, maxDownforce);

        // Apply the downforce evenly across the car's Rigidbody
        rb.AddForce(-transform.up * downforceValue, ForceMode.Force);
    }
}




