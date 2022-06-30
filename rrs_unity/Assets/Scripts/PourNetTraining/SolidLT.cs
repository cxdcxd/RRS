using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

/// <summary>
/// Class to control solid gameobjects in it's end2end life cycle. 
/// </summary>
public class SolidLT : MonoBehaviour {
    private GameObject solidObject;
    private bool collisionOccured;
    private Vector3 position; // Position of solid in scene.
    private Quaternion rotation; // Rotation of the solid in scene.
    private Vector3 localScale; // Scale of solid in the scene.
    private GameObject childOf; // Parent gameobject.
    private GameObject referenceGO; // Reference GO on which this is modeled.
    private string solidName; // Name of this solid object.
    private FlexContainer flexContainer; // Flex container for the solid.
    private FlexSolidAsset flexSolidAsset; // Flex solid asset for this object.
    private const float UNITY_TO_WORLD_CONVERSION_FACTOR = (1.0f / 18.0f);
    public FlexSolidAsset getFlexSolidAsset()
    {
        return this.flexSolidAsset;
    }

    public void setFlexSolidAsset(FlexSolidAsset flexSolidAsset)
    {
        this.flexSolidAsset = flexSolidAsset;
    }

    public FlexContainer getFlexContainer()
    {
        return this.flexContainer;
    }

    public void setFlexContainer(FlexContainer flexContainer)
    {
        this.flexContainer = flexContainer;
    }



    public string getName()
    {
        return this.solidName;
    }

    public void setName(string name)
    {
        this.solidName = name;
    }

    public GameObject getReferenceGO()
    {
        return this.referenceGO;
    }

    public void setReferenceGO(GameObject referenceGO)
    {
        this.referenceGO = referenceGO;
    }


    public GameObject getChildOf()
    {
        return this.childOf;
    }

    public void setChildOf(GameObject childOf)
    {
        this.childOf = childOf;
        this.getSolidObject().transform.parent = childOf.transform;
    }

    public Vector3 getPosition()
    {
        return this.position;
    }

    public void setPosition(Vector3 position)
    {
        this.position = position;
    }

    public Quaternion getRotation()
    {
        return this.rotation;
    }

    public void setRotation(Quaternion rotation)
    {
        this.rotation = rotation;
    }

    public Vector3 getLocalScale()
    {
        return this.localScale;
    }

    public void setLocalScale(Vector3 localScale)
    {
        this.localScale = localScale;
    }

    public float getObjectDepth() {
        return (this.getSolidObject().GetComponent<Renderer>().bounds.extents.y * 2 * UNITY_TO_WORLD_CONVERSION_FACTOR);
    }

    public float getObjectDiameter() {
        return (this.getSolidObject().GetComponent<Renderer>().bounds.extents.x * 2 * UNITY_TO_WORLD_CONVERSION_FACTOR);
    }

    /// <summary>
    /// Get collision status of this object.
    /// </summary>
    /// <returns>True if collision occured with another solid object</returns>
    public bool getCollisionOccured()
    {
        return this.collisionOccured;
    }

    /// <summary>
    /// Sets collision status if it occurs.
    /// </summary>
    /// <param name="collisionOccured">Collision status</param>
    public void setCollisionOccured(bool collisionOccured)
    {
        this.collisionOccured = collisionOccured;
    }


    /// <summary>
    /// Gets the gameobject for this solid
    /// </summary>
    /// <returns>This solid's game object</returns>
    public GameObject getSolidObject()
    {
        return this.solidObject;
    }

    private void setSolidObject() {
        this.solidObject = new GameObject();   
    }

    /// <summary>
    /// Create solid object with requested properties and kinemetics data
    /// </summary>
    /// <param name="setRigidBody">If true, solid has a rigid body</param>
    /// <param name="isKinematic">If true, rigidbody is kinematic</param>
    /// <param name="addCollider">If true, add mesh collider</param>
    /// <param name="mass"> Mass of the solid gameobject. Default 1. </param>
    public void createSolid(bool setRigidBody=false, bool isKinematic=false, bool addCollider=false,
                             float mass = 1.0f) {
        // 1. Spawn the game object
        this.setSolidObject();

        this.getSolidObject().name = this.getName();
        this.getSolidObject().transform.localScale = this.getLocalScale();
        this.setCollisionOccured(false);

        // 2. Add Components
        if (this.getChildOf() != null)
        {
            this.getSolidObject().transform.parent = this.getChildOf().transform;
        } else {
            this.getSolidObject().transform.position = this.getPosition();
        }
        
        this.getSolidObject().transform.rotation = this.getRotation();
        this.getSolidObject().AddComponent<MeshFilter>();
        MeshRenderer mr = this.getSolidObject().AddComponent<MeshRenderer>();
        mr.sharedMaterial = this.getReferenceGO().GetComponent<MeshRenderer>().material;
        this.getSolidObject().GetComponent<MeshFilter>().sharedMesh = this.getReferenceGO().GetComponent<MeshFilter>().mesh;
        if (addCollider)
        {
            this.getSolidObject().AddComponent<MeshCollider>();
            this.getSolidObject().GetComponent<MeshCollider>().sharedMesh = this.getReferenceGO().GetComponent<MeshFilter>().mesh;
            this.getSolidObject().GetComponent<MeshCollider>().convex = true;
            this.getSolidObject().AddComponent<RigidBodyCollisionLT>();
            this.getSolidObject().GetComponent<RigidBodyCollisionLT>().solid = this;
        }
        if (setRigidBody)
        {
            this.getSolidObject().AddComponent<Rigidbody>();
            this.getSolidObject().GetComponent<Rigidbody>().mass = mass;
            if (isKinematic) {
                this.getSolidObject().GetComponent<Rigidbody>().isKinematic = true;
                this.getSolidObject().GetComponent<Rigidbody>().useGravity = false;
            } else {
                this.getSolidObject().GetComponent<Rigidbody>().isKinematic = false;
                this.getSolidObject().GetComponent<Rigidbody>().useGravity = true;
            }
        }    
    }
    public void remove() {
        Destroy(this);
    }
}