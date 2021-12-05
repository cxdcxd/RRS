using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddForceInformationMono : MonoBehaviour
{
    FixedJoint fixedJoint;

    public float targetWeight = 0.0f;
    public GameObject target;
    public GameObject workspace;
    public GameObject coupled_gripper;

    private float pourerMeasuredWeight;
    private float pouredMeasuredWeight;

    public float getPourerMeasuredWeight()
    {
        return this.pourerMeasuredWeight;
    }

    public void setPourerMeasuredWeight(float measuredWeight)
    {
        this.pourerMeasuredWeight = measuredWeight;
    }

    public float getPouredMeasuredWeight()
    {
        return this.pouredMeasuredWeight;
    }

    public void setPouredMeasuredWeight(float measuredWeight)
    {
        this.pouredMeasuredWeight = measuredWeight;
    }

    private void weightMeasurement() {
        float weight = 0.0f;
        fixedJoint = gameObject.GetComponent<FixedJoint>();
        float forceMagnitude = fixedJoint.currentForce.magnitude;
        weight = forceMagnitude / Physics.gravity.magnitude;
        float workspaceMass = this.workspace == null ? 0.0f : this.workspace.GetComponent<Rigidbody>().mass;
        float coupledGripperMass = this.coupled_gripper == null ? 0.0f : this.coupled_gripper.GetComponent<Rigidbody>().mass;
        weight = weight - (workspaceMass + coupledGripperMass);
        bool pourer = fixedJoint.gameObject.name.Contains("mounting (1)") ? true : false;
        if (pourer)
            this.setPourerMeasuredWeight(weight);
        else
            this.setPouredMeasuredWeight(weight);
    }

    // Start is called before the first frame update
    void Start()
    {
        fixedJoint = gameObject.GetComponent<FixedJoint>();
    }

    void FixedUpdate()
    {
        this.weightMeasurement();
    }

    private void OnGUI() {
        GUI.contentColor = Color.black;
        if (target != null)
        {
            if (fixedJoint.gameObject.name.Contains("mounting (1)")) {
                GUI.Label(new Rect(10, 10, 1000, 20), "Liquid in source: " + this.getPourerMeasuredWeight());
            }
            if (fixedJoint.gameObject.name.Contains("mounting"))
            {
                GUI.Label(new Rect(10, 40, 1000, 20), "Target: " + targetWeight);
                GUI.Label(new Rect(10, 80, 1000, 20), "Filled weight: " + this.getPouredMeasuredWeight());
            }
        }
    }
}
