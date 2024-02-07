using UnityEngine;

public class RayCastWheel : MonoBehaviour
{
    Rigidbody rb;

    [Header("Wheel Transforms")]
    public Transform[] wheel;

    [Header("Suspension")]
    public float springStrength;
    public float springRestLength;
    public float wheelRadius;
    public float dampStrength;

    [Space]
    [Header("Motor")]
    public float accelerationForce;
    public float carTopSpeed;
    public AnimationCurve powerCurve;

    [Space]
    [Header("Steering")]
    public float maxSteerAngle = 30f;
    public float wheelBase = 2.5f;
    public float wheelWidth = 1.6f;
    public float steerVel; //variable used by the Mathf.SmoothDampAngle function to smooth the change in steering angle.
    public float turnSmoothTime = 0.1f; // Time taken to smooth the turning. Adjust as needed.

    private float leftCurrentSteerAngle;
    private float rightCurrentSteerAngle;

    float maxDistance;
    float accelInput;
    float steerInput;

    [Space]
    [Header("Tire")]
    public float tireGripFactor;
    float tireMass;

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();

        tireMass = rb.mass / 4;

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        RaycastHit hit;
        maxDistance = wheelRadius + springRestLength;

        for (int i = 0; i < wheel.Length; i++)
        {
            if (Physics.Raycast(wheel[i].transform.position, Vector3.down, out hit, maxDistance))
            {
                if (hit.collider.CompareTag("Ground"))
                {
                    //Suspension Force

                    // world-space direction of the spring force
                    Vector3 springDir = wheel[i].transform.up;

                    //calculate offset for the spring position
                    float offset = maxDistance - hit.distance;

                    //calculate springForce based on offset and springStrength
                    float springForce = offset * springStrength;

                    //calculate the velocity for damping, world space velocity
                    Vector3 wheelVelo = rb.GetPointVelocity(wheel[i].position);

                    //calculate the damping force
                    float dampForce = Vector3.Dot(wheelVelo, springDir) * dampStrength;

                    //calculate final suspension force
                    float suspensionForce = (springForce - dampForce) * Time.deltaTime;

                    //apply springForce at transform origin
                    rb.AddForceAtPosition(suspensionForce * springDir, wheel[i].position);

                    //Steering Force
                    //world-space direction of the spring force
                    Vector3 steeringDir = wheel[i].transform.right;

                    //calculate the velocity in the direction os sliding
                    float steeringVel = Vector3.Dot(steeringDir, wheelVelo);

                    //change in velocity that we would like to control
                    float desiredVelChange = -steeringVel * tireGripFactor;

                    // turn change velocity into acceleration ( acceleration = change in vel / time)
                    // this will produce the acceleration necessary to change the velocity by desiredVelChange in 1 physics step
                    float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

                    // Force = Mass * acceleration, multiply by the mass of the tire and apply as a force
                    rb.AddForceAtPosition(steeringDir * tireMass * desiredAccel, wheel[i].position);

                    // Acceleration/Braking
                    Vector3 accelDir = wheel[i].transform.forward;


                    accelInput = Input.GetAxisRaw("Vertical") * accelerationForce;

                    // acceleration torque
                    if (accelInput > 0.0f || accelInput < 0.0f)
                    {
                        // forward speed of the car (in the direction of driving)
                        float carSpeed = Vector3.Dot(this.transform.forward, rb.velocity);

                        // normalized car speed
                        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);

                        // available torque
                        float availableTorque = powerCurve.Evaluate(normalizedSpeed) * accelInput;

                        rb.AddForceAtPosition(accelDir * availableTorque, wheel[i].position);
                    }

                    steerInput = Input.GetAxisRaw("Horizontal");

                    // Calculate steering angles using Ackermann steering geometry
                    float turnRadius = wheelBase / Mathf.Tan(Mathf.Abs(maxSteerAngle) * Mathf.Deg2Rad) * steerInput;

                    // Calculate steering angles for each wheel Ackerman equation
                    float leftTargetSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius + (wheelWidth / 2)));
                    float rightTargetSteerAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius - (wheelWidth / 2)));

                    if (steerInput > 0 && i < 2)
                    {

                        wheel[i].localEulerAngles = new Vector3(0, rightTargetSteerAngle, 0);

                    }
                    else if (steerInput < 0 && i < 2)
                    {

                        wheel[i].localEulerAngles = new Vector3(0, leftTargetSteerAngle, 0);

                    }
                    else
                    {

                        wheel[i].localEulerAngles = new Vector3(0, 0, 0);
                    }

                }

            }
        }
    }
}
