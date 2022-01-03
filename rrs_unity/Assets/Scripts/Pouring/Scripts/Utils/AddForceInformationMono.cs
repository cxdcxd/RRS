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

    private void OnGUI() 
    {

        GUI.skin.label.fontSize = 25;
        GUI.contentColor = Color.black;

        if (target != null)
        {
            if (fixedJoint.gameObject.name.Contains("mounting (1)")) {
                GUI.Label(new Rect(950, 50 + 400, 2000, 100), "Liquid in source: " + (this.getPourerMeasuredWeight() * 1000).ToString("N5") + " g");
            }
            if (fixedJoint.gameObject.name.Contains("mounting"))
            {
                if (targetWeight != 0)
                    GUI.Label(new Rect(950, 10 + 400, 2000, 100), "Target: " + (targetWeight * 1000) + " g");
                
                if (this.getPouredMeasuredWeight() != 0)
                GUI.Label(new Rect(950, 90 + 400, 2000, 100), "Liquid in target: " + (this.getPouredMeasuredWeight() * 1000).ToString("N5") + " g");
            }
        }
    }
}
