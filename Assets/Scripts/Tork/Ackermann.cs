using UnityEngine;

namespace Adrenak.Tork
{
    /// <summary>
    /// An implementation of Ackermann steering mechanism
    /// </summary>
    public class Ackermann : MonoBehaviour
    {
        [SerializeField] Wheel m_FrontRight;
        public Wheel FrontRightWheel { get { return m_FrontRight; } }

        [SerializeField] Wheel m_FrontLeft;
        public Wheel FrontLeftWheel { get { return m_FrontLeft; } }

        [SerializeField] Wheel m_RearRight;
        public Wheel RearRightWheel { get { return m_RearRight; } }

        [SerializeField] Wheel m_RearLeft;
        public Wheel RearLeftWheel { get { return m_RearLeft; } }

        public float angle;

        public float AxleSeparation
        {
            get { return (m_FrontLeft.transform.position - m_RearLeft.transform.position).magnitude; }
        }

        public float AxleWidth
        {
            get { return (m_FrontLeft.transform.position - m_FrontRight.transform.position).magnitude; }
        }

        public float FrontRightRadius
        {
            get { return AxleSeparation / Mathf.Sin(Mathf.Abs(m_FrontRight.steerAngle)); }
        }

        public float FrontLeftRadius
        {
            get { return AxleSeparation / Mathf.Sin(Mathf.Abs(m_FrontLeft.steerAngle)); }
        }

        void Update()
        {
            var farAngle = AckermannUtils.GetSecondaryAngle(angle, AxleSeparation, AxleWidth);

            // The rear wheels are always at 0 steer in Ackermann
            m_RearLeft.steerAngle = m_RearRight.steerAngle = 0;

            if (Mathf.Approximately(angle, 0))
                m_FrontRight.steerAngle = m_FrontLeft.steerAngle = 0;

            m_FrontLeft.steerAngle = angle;
            m_FrontRight.steerAngle = farAngle;
        }

        public Vector3 GetPivot()
        {
            if (angle > 0)
                return m_RearRight.transform.position + CurrentRadii[0] * m_RearRight.transform.right;
            else
                return m_RearLeft.transform.position - CurrentRadii[0] * m_RearLeft.transform.right;
        }

        public float[] CurrentRadii
        {
            get { return AckermannUtils.GetRadii(angle, AxleSeparation, AxleWidth); }
        }
    }
}
