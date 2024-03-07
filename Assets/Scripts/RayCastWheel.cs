using UnityEngine;

public class RayCastWheel : MonoBehaviour
{

    Rigidbody rb;

    private float[] currentSteerAngles;
    private Vector3[] originalWheelPositions;
    private float leftWheelSteerAngle = 0f;
    private float rightWheelSteerAngle = 0f;
    private Vector3[] wheelVelocities;

    [Header("Wheel Transforms")]
    public Transform[] wheel;
    public Transform[] wheelMeshes; // Array to hold the wheel mesh Transforms

    // Suspension
    [Header("Suspension")]
    [Tooltip("The k constant in the Hooke's law for the suspension. High values make the spring hard to compress. Make this larger for heavier vehicles. Recommended: 5x car mass.")]
    /// <summary>
    /// The k constact in the Hooke's law for the suspension. High values make the spring hard to compress. Make this higher for heavier vehicles
    /// </summary>
    public float springStrength;

    [Tooltip("How far the spring expands when it is in air.")]
    /// <summary>
    /// How far the spring expands when it is in air.
    /// </summary>
    public float springLength = .25f;

    [Tooltip("The radius of the wheel")]
    /// <summary>
    /// The radius of the wheel
    /// </summary>
    public float wheelRadius;

    [Tooltip("Damping applied to the wheel. Higher values allow the car to negotiate bumps easily. Recommended: 0.25. Values outside [0, 0.5f] are unnatural")]
    /// <summary>
    /// Damping applied to the wheel. Higher values allow the car to negotiate bumps easily. Recommended: 0.25. Values outside [0, 0.5f] are unnatural
    /// </summary>
    public float dampStrength;

    [Tooltip("The rate (m/s) at which the spring relaxes to maximum length when the wheel is not on the ground. Recommended: suspension distance/2")]
    /// <summary>
    /// The rate (m/s) at which the spring relaxes to maximum length when the wheel is not on the ground.
    /// </summary>
    public float springRelaxRate = .125f;

    /// <summary>
    /// The distance to which the spring of the wheel is compressed
    /// </summary>
    public float compressionDistance { get; private set; }
    float m_PrevCompressionDist;

    /// <summary>
    /// The ratio of compression distance and suspension distance
    /// 0 when the wheel is entirely uncompressed, 
    /// 1 when the wheel is entirely compressed
    /// </summary>
    public float compressionRatio { get; private set; }

    public float sprungMass => suspensionForce.magnitude / 9.81f;

    /// <summary>
    /// The force the spring exerts on the rigidbody
    /// </summary>
    public Vector3 suspensionForce { get; private set; }

    [Header("Raycasting")]
    /// <summary>
    /// The layers used for wheel raycast
    /// </summary>
    public LayerMask m_RaycastLayers;

    /// <summary>
    /// The raycast hit point of the wheel
    /// </summary>
    public RaycastHit hit { get { return m_Hit; } }
    RaycastHit m_Hit;

    public bool isGrounded;

    Ray m_Ray;
    public const float k_ExtraRayLength = 1;
    public float RayLength => springLength + wheelRadius + k_ExtraRayLength;

    // Motor
    [Header("Motor")]
    public float accelerationForce;
    public float carTopSpeed;
    public AnimationCurve powerCurve;
    public float downforceCoefficient = 10f;
    public float maxDownforce = 50f;

    // Steering
    [Header("Steering")]
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
    Vector3 currentVelocity;


    //Main Unity Functions
    void Start()
    {
        InitializeCar();

        currentSteerAngles = new float[wheel.Length];
        wheelVelocities = new Vector3[wheel.Length];

        // Initialize the array to match the number of wheels
        originalWheelPositions = new Vector3[wheel.Length];

        // Store the original local positions of each wheel
        for (int i = 0; i < wheel.Length; i++)
        {
            originalWheelPositions[i] = wheel[i].localPosition;
        }
    }

    void FixedUpdate()
    {
        steerInput = Input.GetAxisRaw("Horizontal"); // Get steering input (-1 to 1)
        accelInput = Input.GetAxisRaw("Vertical"); // Get acceleration input (-1 to 1)

        currentSpeed = rb.velocity.magnitude; //rb speed of the vehicle
        currentVelocity = rb.velocity; //rb velocity of the vehicle
                                       //
        for (int i = 0; i < wheel.Length; i++) //iteration for each wheel suspension
        {
            m_Ray.direction = -wheel[i].transform.up;
            m_Ray.origin = wheel[i].transform.position + wheel[i].transform.up * k_ExtraRayLength;

            maxDistance = wheelRadius + springLength;
            wheelVelocities[i] = rb.GetPointVelocity(wheel[i].position);

            isGrounded = WheelRaycast(RayLength, ref m_Hit);

            // Get the normal of the hit point to adjust force application direction
            Vector3 groundNormal = hit.normal;

            if (!isGrounded)
            {
                m_PrevCompressionDist = compressionDistance;
                compressionDistance = compressionDistance - Time.fixedDeltaTime * springRelaxRate;
                compressionDistance = Mathf.Clamp(compressionDistance, 0, springLength);
                return;
            }

            var force = 0.0f;
            compressionDistance = RayLength - hit.distance;
            compressionDistance = Mathf.Clamp(compressionDistance, 0, springLength);
            compressionRatio = compressionDistance / springLength;

            // Calculate the force from the springs compression using Hooke's Law
            float compressionForce = springStrength * compressionRatio;
            force += compressionForce;

            // Calculate the damping force based on compression rate of the spring
            float rate = (m_PrevCompressionDist - compressionDistance) / Time.fixedDeltaTime;
            m_PrevCompressionDist = compressionDistance;

            float dampingForce = rate * springStrength * dampStrength;
            force -= dampingForce;

            suspensionForce = wheel[i].transform.up * force;

            // Apply suspension force
            GetComponent<Rigidbody>().AddForceAtPosition(suspensionForce, wheel[i].transform.position);

            // WHEEL MESH
            Vector3 adjustedPosition = wheel[i].localPosition;
            adjustedPosition.y += (compressionDistance + 0.65f); // Adjust Y position based on suspension compression

            wheelMeshes[i].localPosition = adjustedPosition;

            //GRIP FORCE CALCULATION
            Vector3 slideDir = wheel[i].transform.right; //world-space direction of the slide force

            float slidingVel = Vector3.Dot(slideDir, wheelVelocities[i]); //calculate the velocity in the direction os sliding

            slidingVel *= -tireGripFactor; //change in velocity that we would like to control

            float desiredAccel = slidingVel / Time.fixedDeltaTime; // turn change velocity into acceleration

            float lateralForceMagnitude = tireMass * desiredAccel; // Calculate the lateral force

            lateralForceMagnitude = Mathf.Min(lateralForceMagnitude, maxLateralForce);

            rb.AddForceAtPosition(slideDir * lateralForceMagnitude, wheel[i].position);

            bool WheelRaycast(float maxDistance, ref RaycastHit hit)
            {
                if (Physics.SphereCast(m_Ray.origin, wheelRadius, m_Ray.direction, out hit, maxDistance, m_RaycastLayers))
                    return true;
                return false;
            }

            if (i < 2) //FRONT WHEELS
            {
                //ACCELERATION FORCE CALCULATION

                Vector3 forwardDir = wheel[i].transform.forward;

                Vector3 forwardOnSurface = Vector3.Cross(groundNormal, forwardDir); // Calculate the forward direction of the car relative to the ground normal

                Vector3 accelDir = Vector3.Cross(forwardOnSurface, groundNormal).normalized; // Now, get the direction that's "forward" for the wheel, but still perpendicular to the ground normal

                //Debug.Log(accelDir.magnitude);

                float carSpeed = currentVelocity.z;
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * accelInput * accelerationForce;

                rb.AddForceAtPosition(accelDir * availableTorque, wheel[i].position);

                //STEERING CALCULATION
                float currentMaxSteerAngle = Mathf.Max(baseMaxSteerAngle - (currentSpeed * speedSensitivity), 10f); // Ensures there's always some ability to steer, clamped at 10 degrees

                float turnRadius = wheelBase / Mathf.Tan(Mathf.Deg2Rad * currentMaxSteerAngle * steerInput); // Use 'currentMaxSteerAngle' to calculate the turn radius and steering angles

                // Calculate steering angles for left and right wheels based on Ackermann steering principle
                leftWheelSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius - (wheelWidth / 2)));
                rightWheelSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius + (wheelWidth / 2)));

                // Smoothly damp the angle from current to target
                if (i == 0)
                {
                    currentSteerAngles[i] = Mathf.SmoothDampAngle(currentSteerAngles[i], leftWheelSteerAngle, ref steerVel, turnSmoothTime);
                }
                else if (i == 1)
                {
                    currentSteerAngles[i] = Mathf.SmoothDampAngle(currentSteerAngles[i], rightWheelSteerAngle, ref steerVel, turnSmoothTime);
                }

                // Apply the smoothed angle to the wheel's rotation
                wheel[i].localEulerAngles = new Vector3(wheel[i].localEulerAngles.x, currentSteerAngles[i], wheel[i].localEulerAngles.z);
            }
        }
    }

    void InitializeCar()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.5f, 0);
        tireMass = rb.mass / 4;

    }

    /*private void ApplyDownforce(float speed)
    {
        // Calculate downforce based on current speed (squared for a more noticeable effect at high speeds, if desired)
        float downforceValue = Mathf.Min(speed * speed * downforceCoefficient, maxDownforce);

        // Apply the downforce evenly across the car's Rigidbody
        rb.AddForce(-transform.up * downforceValue, ForceMode.Force);
    }*/


}
