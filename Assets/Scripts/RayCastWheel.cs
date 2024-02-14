using UnityEngine;

public class RayCastWheel : MonoBehaviour
{

    Rigidbody rb;

    private float[] currentSteerAngles;
    private float[] currentSuspensionCompression;
    private Vector3[] originalWheelPositions;
    private float leftWheelSteerAngle = 0f;
    private float rightWheelSteerAngle = 0f;
    private Vector3[] wheelVelocities;

    [Header("Wheel Transforms")]
    public Transform[] wheel;
    public Transform[] wheelMeshes; // Array to hold the wheel mesh Transforms

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
    public float maxLateralForce = 10f;

    // Tire
    [Header("Tire")]
    public float tireGripFactor;
    float tireMass;

    float maxDistance;
    float accelInput;
    float steerInput;
    float currentSpeed;


    //Main Unity Functions
    void Start()
    {
        InitializeCar();

        currentSteerAngles = new float[wheel.Length];
        currentSuspensionCompression = new float[wheel.Length];
        wheelVelocities = new Vector3[wheel.Length];

        // Initialize the array to match the number of wheels
        originalWheelPositions = new Vector3[wheel.Length];

        // Store the original local positions of each wheel
        for (int i = 0; i < wheel.Length; i++)
        {
            originalWheelPositions[i] = wheel[i].localPosition;
        }
    }

    private void Update()
    {
        UpdateWheelMeshes();
    }

    void FixedUpdate()
    {
        UpdateWheelVelocities();

        currentSpeed = rb.velocity.magnitude;


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
        

    }

    void UpdateWheelVelocities()
    {
        for (int i = 0; i < wheel.Length; i++)
        {
            wheelVelocities[i] = rb.GetPointVelocity(wheel[i].position);
        }
    }

    void ProcessWheel(int i)
    {
        //SPHERECAST METHOD

        maxDistance = wheelRadius + springRestLength; // Ensure this is set correctly

        RaycastHit hitInfo;
        bool hit = Physics.SphereCast(wheel[i].transform.position, wheelRadius, Vector3.down, out hitInfo, maxDistance);

        if (hit && hitInfo.collider.CompareTag("Ground"))
        {
            ApplySuspensionForce(i, hitInfo, wheelRadius);
            ApplySteeringForce(i, hitInfo);
            ApplyAccelerationForce(i, hitInfo);

            // Calculate and store suspension compression
            float compression = (springRestLength + wheelRadius) - hitInfo.distance;
            currentSuspensionCompression[i] = Mathf.Max(0, compression); // Ensure compression is not negative

        }
        else
        {
            currentSuspensionCompression[i] = 0; // No compression if not grounded
        }

        // Visual Debugging for SphereCast
        DebugDrawSphereCast(wheel[i].transform.position, Vector3.down * maxDistance, wheelRadius);

    }

    void ApplySuspensionForce(int i, RaycastHit hit, float sphereRadius)
    {
        // Calculate how much the suspension is compressed based on the hit distance
        float suspensionCompression = maxDistance - hit.distance;

        // Ensure that we don't apply forces if the suspension isn't actually compressed
        if (suspensionCompression > 0)
        {
            // Get the normal of the hit point to adjust force application direction
            Vector3 groundNormal = hit.normal;
            Vector3 springDirection = groundNormal.normalized; // Assuming up is the direction of suspension force

            // Use the cached velocity
            Vector3 wheelVelocity = wheelVelocities[i];


            // Calculate spring force using the offset
            float springForce = suspensionCompression * springStrength;

            // Calculate damping strength
            float dampingForce = Vector3.Dot(wheelVelocity, springDirection) * dampStrength;


            // Final suspension force calculation
            float totalSuspensionForce = (springForce - dampingForce) * Time.fixedDeltaTime;

            // Apply the force at the wheel's position
            rb.AddForceAtPosition(springDirection * totalSuspensionForce, wheel[i].position);

            // Optional: Visual debugging
            Debug.DrawRay(wheel[i].position, springDirection * totalSuspensionForce, Color.green);
        }

    }

    void ProcessSteering()
    {
        steerInput = Input.GetAxis("Horizontal"); // Get steering input (-1 to 1)

        float Speed = currentSpeed;

        // Ensures there's always some ability to steer, clamped at 10 degrees
        float currentMaxSteerAngle = Mathf.Max(baseMaxSteerAngle - (Speed * speedSensitivity), 10f);

        // Calculate turn radius based on steering input
        // Note: Mathf.Tan returns a value in radians, and maxSteerAngle * steerInput is in degrees.
        // Mathf.Deg2Rad is used to convert degrees to radians for the Mathf.Tan function.
        // Use 'currentMaxSteerAngle' to calculate the turn radius and steering angles
        float turnRadius = wheelBase / Mathf.Tan(Mathf.Deg2Rad * currentMaxSteerAngle * steerInput);

        // Calculate steering angles for left and right wheels based on Ackermann steering principle
        leftWheelSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius - (wheelWidth / 2)));
        rightWheelSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius + (wheelWidth / 2)));

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
        // Use the cached velocity
        Vector3 wheelVelocity = wheelVelocities[i];

        //world-space direction of the spring force
        Vector3 steeringDir = wheel[i].transform.right;

        //calculate the velocity in the direction os sliding
        float steeringVel = Vector3.Dot(steeringDir, wheelVelocity);

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
        Debug.DrawRay(wheel[i].position, wheelVelocity.normalized * 2f, Color.blue);
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

            // Use the normal of the hit point to adjust the force application direction
            Vector3 groundNormal = hit.normal;

            // Calculate the forward direction of the car relative to the ground normal
            Vector3 forwardOnSurface = Vector3.Cross(groundNormal, wheel[i].transform.forward).normalized;

            // Now, get the direction that's "forward" for the wheel, but still perpendicular to the ground normal
            Vector3 accelDir = Vector3.Cross(forwardOnSurface, groundNormal).normalized;

            accelInput = Input.GetAxisRaw("Vertical") * accelerationForce;
            float carSpeed = Vector3.Dot(transform.forward, rb.velocity);
            float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
            float availableTorque = powerCurve.Evaluate(normalizedSpeed) * accelInput;

            rb.AddForceAtPosition(accelDir * availableTorque, wheel[i].position);

            ApplyDownforce(carSpeed);

            Debug.DrawRay(wheel[i].position, accelDir * 2, Color.white); // Visualize acceleration direction
        }
    }

    private void ApplyDownforce(float speed)
    {
        // Calculate downforce based on current speed (squared for a more noticeable effect at high speeds, if desired)
        float downforceValue = Mathf.Min(speed * speed * downforceCoefficient, maxDownforce);

        // Apply the downforce evenly across the car's Rigidbody
        rb.AddForce(-transform.up * downforceValue, ForceMode.Force);
    }

    void DebugDrawSphereCast(Vector3 start, Vector3 direction, float radius)
    {
        // Draw the path of the sphere
        Debug.DrawLine(start, start + direction, Color.yellow);

        // Draw the start sphere
        DebugDrawSphere(start, radius, Color.green);

        // Draw the end sphere
        DebugDrawSphere(start + direction, radius, Color.red);
    }

    void DebugDrawSphere(Vector3 center, float radius, Color color)
    {
        float angleStep = 10f; // Decrease this value for more precision, increase for performance
        Vector3 prevPoint = center + Quaternion.Euler(0, 0, 0) * Vector3.forward * radius;
        Vector3 nextPoint = Vector3.zero;

        for (int angle = 0; angle <= 360; angle += (int)angleStep)
        {
            nextPoint.x = center.x + Mathf.Sin(Mathf.Deg2Rad * angle) * radius;
            nextPoint.z = center.z + Mathf.Cos(Mathf.Deg2Rad * angle) * radius;
            nextPoint.y = center.y;

            Debug.DrawLine(prevPoint, nextPoint, color);
            prevPoint = nextPoint;
        }

        // This draws a circle in the XY plane. To fully represent a sphere, you would need to repeat this
        // for the YZ and XZ planes, adjusting the calculation for nextPoint accordingly.
    }

    void UpdateWheelMeshes()
    {
        float Speed = currentSpeed; // Get the speed of the car

        for (int i = 0; i < wheelMeshes.Length; i++)
        {
            // Copy position and rotation from the logical wheel transform to the visual wheel mesh
            // This assumes wheel[] and wheelMeshes[] are matched in order and size
            Transform wheelTransform = wheel[i];
            Transform wheelMeshTransform = wheelMeshes[i];

            // For position - you might still want to adjust for suspension compression
            float compression = currentSuspensionCompression[i];
            Vector3 adjustedPosition = wheelTransform.localPosition;
            adjustedPosition.y += (compression + 0.65f); // Adjust Y position based on suspension compression

            wheelMeshTransform.localPosition = adjustedPosition;

            // For rotation - directly copy
            // Steering (Y rotation) and potentially wheel rolling (Z rotation) are copied directly
            wheelMeshTransform.localRotation = wheelTransform.localRotation;

            // Additional rotation to simulate wheel rolling
            float rollRotation = CalculateRollRotation(Speed);
            wheelMeshTransform.Rotate(Vector3.right, rollRotation, Space.Self);

        }


        float CalculateRollRotation(float speed)
        {
            // Calculate the wheel's circumference (in meters, assuming wheelRadius is in meters)
            float wheelCircumference = 2 * Mathf.PI * wheelRadius;

            // Calculate the distance the car has traveled since the last frame
            // 'speed' is the car's speed in meters per second. Multiply by Time.deltaTime to get distance per frame.
            float distanceTraveled = speed * Time.deltaTime;

            // Calculate how many rotations the wheel made based on the distance traveled
            float wheelRotations = distanceTraveled / wheelCircumference;

            // Convert rotations to degrees (1 rotation = 360 degrees)
            float rotationInDegrees = wheelRotations * 360;

            return rotationInDegrees;
        }

    }
}
