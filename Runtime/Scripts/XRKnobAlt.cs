using System;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.XR.Interaction.Toolkit;

namespace Unity.VRTemplate
{
    public enum KnobAxis
    {
        X,
        Y,
        Z
    }

    /// <summary>
    /// An interactable knob that directly controls the rotation of a robot joint in VR.
    /// </summary>
    public class XRKnobAlt : UnityEngine.XR.Interaction.Toolkit.Interactables.XRBaseInteractable
    {
        [SerializeField]
        [Tooltip("A unique identifier for this knob instance")]
        int m_UniqueID = 0;
        
        [SerializeField]
        [Tooltip("The object that is visually grabbed and manipulated")]
        Transform m_Handle = null;

        [SerializeField]
        [Tooltip("The robot joint that this knob controls")]
        Transform m_RobotJoint = null;

        [SerializeField]
        [Tooltip("The minimum angle for the robot joint")]
        float m_JointMinAngle = -360.0f;

        [SerializeField]
        [Tooltip("The maximum angle for the robot joint")]
        float m_JointMaxAngle = 360.0f;

        [SerializeField]
        [Tooltip("Angle increments to support, if greater than '0'")]
        float m_AngleIncrement = 0.0f;

        [SerializeField]
        [Tooltip("The axis to rotate around.")]
        KnobAxis m_RotationAxis = KnobAxis.Y;

        [SerializeField]
        [Tooltip("Events to trigger when the knob is rotated")]
        public UnityEvent<float> m_OnValueChange = new UnityEvent<float>();
        
        UnityEngine.XR.Interaction.Toolkit.Interactors.IXRSelectInteractor m_Interactor;
        float m_Value = 0.0f;
        
        public int uniqueID
        {
            get => m_UniqueID;
            set => m_UniqueID = value;
        }
        
        public Transform handle
        {
            get => m_Handle;
            set => m_Handle = value;
        }

        public float jointAngle
        {
            get => m_Value;
            set
            {
                m_Value = Mathf.Clamp(value, m_JointMinAngle, m_JointMaxAngle);
                SetKnobRotation(m_Value);
                UpdateRobotJointAngle(m_Value);
                m_OnValueChange.Invoke(m_Value);
            }
        }
        
        public float jointMinAngle
        {
            get => m_JointMinAngle;
            set
            {
                m_JointMinAngle = value;
                jointAngle = Mathf.Clamp(jointAngle, m_JointMinAngle, m_JointMaxAngle);
            }
        }

        public float jointMaxAngle
        {
            get => m_JointMaxAngle;
            set
            {
                m_JointMaxAngle = value;
                jointAngle = Mathf.Clamp(jointAngle, m_JointMinAngle, m_JointMaxAngle);
            }
        }

        public KnobAxis rotationAxis
        {
            get => m_RotationAxis;
            set => m_RotationAxis = value;
        }

        void Start()
        {
            SetKnobRotation(m_Value);
            UpdateRobotJointAngle(m_Value);
        }

        protected override void OnEnable()
        {
            base.OnEnable();
            selectEntered.AddListener(StartGrab);
            selectExited.AddListener(EndGrab);
        }

        protected override void OnDisable()
        {
            selectEntered.RemoveListener(StartGrab);
            selectExited.RemoveListener(EndGrab);
            base.OnDisable();
        }

        void StartGrab(SelectEnterEventArgs args)
        {
            m_Interactor = args.interactorObject;
        }

        void EndGrab(SelectExitEventArgs args)
        {
            m_Interactor = null;
        }

        public override void ProcessInteractable(XRInteractionUpdateOrder.UpdatePhase updatePhase)
        {
            base.ProcessInteractable(updatePhase);

            if (updatePhase == XRInteractionUpdateOrder.UpdatePhase.Dynamic && isSelected)
            {
                UpdateRotation();
            }
        }

        public override Transform GetAttachTransform(UnityEngine.XR.Interaction.Toolkit.Interactors.IXRInteractor interactor)
        {
            return m_Handle;
        }

        void UpdateRotation()
        {
            var interactorTransform = m_Interactor.GetAttachTransform(this);
            float knobRotation = 0.0f;
            switch (m_RotationAxis)
            {
                case KnobAxis.X:
                    knobRotation = transform.InverseTransformDirection(interactorTransform.forward).x * 180.0f;
                    break;
                case KnobAxis.Y:
                    knobRotation = transform.InverseTransformDirection(interactorTransform.forward).y * 180.0f;
                    break;
                case KnobAxis.Z:
                    knobRotation = transform.InverseTransformDirection(interactorTransform.forward).z * 180.0f;
                    break;
                default:
                    knobRotation = 0.0f;
                    break;
            }
            jointAngle = knobRotation;
        }

        void SetKnobRotation(float angle)
        {
            if (m_AngleIncrement > 0)
            {
                angle = Mathf.Round(angle / m_AngleIncrement) * m_AngleIncrement;
            }

            if (m_Handle != null) 
            {
                switch (m_RotationAxis)
                {
                    case KnobAxis.X:
                        m_Handle.localEulerAngles = new Vector3(angle, 0.0f, 0.0f);
                        break;
                    case KnobAxis.Y:
                        m_Handle.localEulerAngles = new Vector3(0.0f, angle, 0.0f);
                        break;
                    case KnobAxis.Z:
                        m_Handle.localEulerAngles = new Vector3(0.0f, 0.0f, angle);
                        break;
                    default:
                        m_Handle.localEulerAngles = new Vector3(0.0f, angle, 0.0f);
                        break;
                }
            }
        }

        void UpdateRobotJointAngle(float angle)
        {
            if (m_RobotJoint != null)
            {
                switch (m_RotationAxis)
                {
                    case KnobAxis.X:
                        m_RobotJoint.localEulerAngles = new Vector3(angle, 0.0f, 0.0f); // Assuming rotation around X-axis
                        break;
                    case KnobAxis.Y:
                        m_RobotJoint.localEulerAngles = new Vector3(0.0f, angle, 0.0f); // Assuming rotation around Y-axis
                        break;
                    case KnobAxis.Z:
                        m_RobotJoint.localEulerAngles = new Vector3(0.0f, 0.0f, angle); // Assuming rotation around Z-axis
                        break;
                    default:
                        m_RobotJoint.localEulerAngles = new Vector3(0.0f, angle, 0.0f); // Assuming rotation around Y-axis
                        break;
                }
                m_OnValueChange.Invoke(angle);
            }
        }

        void OnValidate()
        {
            m_Value = Mathf.Clamp(m_Value, m_JointMinAngle, m_JointMaxAngle);
            SetKnobRotation(m_Value);
        }
    }
}
