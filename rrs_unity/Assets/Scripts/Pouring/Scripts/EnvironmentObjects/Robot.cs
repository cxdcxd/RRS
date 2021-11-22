using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

/// <summary>
/// Class to manage robot gameobject in it's end2end life cycle. 
/// </summary>
public class Robot : MonoBehaviour {
    private GameObject robotHand; // Gameobject of robot hand. Composed of bolts and 2 parallel jaws.
    private GameObject fingerA; // First finger of the parallel jaw gripper.
    private GameObject fingerB; // Second finger of the parallel jaw gripper.
    private GameObject bolts; // Bolts of the robot hand.
    private Solid heldObject;
    private GameObject workspace;

    public GameObject getWorkspace()
    {
        return this.workspace;
    }

    public void setWorkspace(GameObject workspace)
    {
        this.workspace = workspace;
    }


    public Solid getHeldObject()
    {
        return this.heldObject;
    }

    public void setHeldObject(Solid heldObject)
    {
        this.heldObject = heldObject;
    }
    // Object grasped.
    private bool collision = false; // state of collision with other objects.

    public GameObject getRobotHand()
    {
        return this.robotHand;
    }

    private void setRobotHand(GameObject robotHand)
    {
        this.robotHand = Instantiate(robotHand, Vector3.zero, robotHand.transform.rotation);
    }

    public GameObject getFingerA()
    {
        return this.fingerA;
    }

    private void setFingerA(GameObject fingerA)
    {
        this.fingerA = fingerA;
    }

    public GameObject getFingerB()
    {
        return this.fingerB;
    }

    private void setFingerB(GameObject fingerB)
    {
        this.fingerB = fingerB;
    }

    public GameObject getBolts()
    {
        return this.bolts;
    }

    private void setBolts(GameObject bolts)
    {
        this.bolts = bolts;
    }

    public bool isCollision()
    {
        return this.collision;
    }

    public void setCollision(bool collision)
    {
        this.collision = collision;
    }

    public Vector3 positionInWorldFrame(Vector3 position) {
        return this.getRobotHand().transform.TransformPoint(position);
    }

    public void createRobot(GameObject prefab, PhysicMaterial material=null, bool addCollider=false) {
        this.setRobotHand(prefab);

        if (addCollider) {
            this.getRobotHand().AddComponent<MeshCollider>();
            this.getRobotHand().GetComponent<MeshCollider>().sharedMesh = prefab.GetComponent<MeshFilter>().mesh;
            this.getRobotHand().GetComponent<MeshCollider>().convex = true;
            if (material != null)
                this.getRobotHand().GetComponent<MeshCollider>().material = material;
            this.getRobotHand().AddComponent<RigidBodyCollision>();
            this.getRobotHand().GetComponent<RigidBodyCollision>().robot = this;
        }
        Transform[] children = prefab.GetComponentsInChildren<Transform>();
        for (int i = 0; i < children.Length; i++) {
            string name = children[i].gameObject.name;
            if (!name.Contains("Hand"))
            {
                switch (name)
                {
                    case "Bolts":
                        this.setBolts(children[i].gameObject);
                        break;
                    case "FingerA":
                        this.setFingerA(children[i].gameObject);
                        break;
                    case "FingerB":
                        this.setFingerB(children[i].gameObject);
                        break;
                    case "HandE":
                        break;
                    default:
                        throw new Exception("Got unknown component" + name);

                }
            }
        }
    }

    public float maxJawWidth() {
        if (this.getFingerA() == null || this.getFingerB() == null) {
            throw new Exception("Robot components not assembled yet.");
        }

        return Vector3.Distance(this.getFingerA().transform.position, this.getFingerB().transform.position);
    }

    public Vector3 jawCenter() {
        if (this.getFingerA() == null || this.getFingerB() == null) {
            throw new Exception("Robot components not assembled yet.");
        }

        return (0.5f * (this.getFingerA().transform.position + this.getFingerB().transform.position));
    }

    public void changeTransformations(Vector3 position, Quaternion rotation, Vector3 scale) {
        this.getRobotHand().transform.position = position;
        this.getRobotHand().transform.rotation = rotation;
        this.getRobotHand().transform.localScale = scale;
    }

    public void holdObject(Solid solid, GameObject workspace, bool useJoint=false, bool useMeasurer=false, bool isTarget=false) {
        Bounds solidBounds = solid.getSolidObject().GetComponent<Renderer>().bounds;
        float x = UnityEngine.Random.Range(-6.0f * solidBounds.extents.x, 6.0f * solidBounds.extents.x);
        float y = 8.0f * solidBounds.extents.y;
        float z = 0.0f;
        
        if (!isTarget)
        {
            this.getRobotHand().transform.position = new Vector3(x, y, z);
            Vector3 fingerAPosition = this.getRobotHand().transform.position + this.getFingerA().transform.position;
            Vector3 fingerBPosition = this.getRobotHand().transform.position + this.getFingerB().transform.position;
            Vector3 newC = 0.5f * (fingerAPosition + fingerBPosition);
            Vector3 offset = new Vector3(-solidBounds.extents.x, 0.0f, 0.0f); 
            // if (solid.getSolidObject().name.Contains("mug"))
            //     offset = new Vector3(-solidBounds.extents.x, 0.0f, -solidBounds.extents.z);
            Vector3 newSourcePosition = newC + offset;
            solid.getSolidObject().transform.position = newSourcePosition;
        } else {
            Bounds workspaceBounds = workspace.GetComponent<Renderer>().bounds;
            this.getRobotHand().transform.rotation = Quaternion.Euler(90, 90, 0);
            this.getRobotHand().transform.position = workspace.transform.position + new Vector3(-2 * workspaceBounds.extents.x, 0, 0);
            workspace.SetActive(true);
            solid.getSolidObject().transform.position = workspace.transform.position + new Vector3(0.0f, 2 * workspaceBounds.extents.y, 0.0f);
            solid.getSolidObject().GetComponent<Rigidbody>().freezeRotation = true;
        }

        if (useJoint) {
            FixedJoint fs = this.getRobotHand().AddComponent<FixedJoint>();
            if (!isTarget)
                fs.connectedBody = solid.getSolidObject().GetComponent<Rigidbody>();
            else
                fs.connectedBody = workspace.GetComponent<Rigidbody>();

            this.getRobotHand().GetComponent<Rigidbody>().isKinematic = true;
            this.getRobotHand().GetComponent<Rigidbody>().useGravity = false;
            fs.enableCollision = true;
            if (useMeasurer) {
                ForceInformation fInfo = this.getRobotHand().AddComponent<ForceInformation>();
                fInfo.target = solid.getSolidObject();
                if (isTarget) {
                    fInfo.workspace = workspace;
                }
            }
        }
        this.setHeldObject(solid);
    }
    public void remove() {
        Destroy(this.getRobotHand().GetComponent<ForceInformation>());
        Destroy(this);
    }
}