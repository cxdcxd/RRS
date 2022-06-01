using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticlePositionStateLT : MonoBehaviour
{
    private GameObject source; // Original solid gameobject where particles were initially contained.
    private GameObject target; // Original sold gameobject where particles will finally land.
    private GameObject workspace; // Workspace object where target container is resting

    public GameObject getWorkspace()
    {
        return this.workspace;
    }

    public void setWorkspace(GameObject workspace)
    {
        this.workspace = workspace;
    }

    public GameObject getSource()
    {
        return this.source;
    }

    public void setSource(GameObject source)
    {
        this.source = source;
    }

    public GameObject getTarget()
    {
        return this.target;
    }

    public void setTarget(GameObject target)
    {
        this.target = target;
    }

    public bool containedInSolid(Vector3 particlePosition, bool isSource) {
        MeshCollider collider = null;

        if (isSource)
        {
            // Get bounds for source actor. 
            collider = this.getSource().GetComponent<MeshCollider>();
        } else {
            collider = this.getTarget().GetComponent<MeshCollider>();
        }

        Bounds solidBounds = collider.bounds;

        return (particlePosition.x > solidBounds.min.x && particlePosition.x < solidBounds.max.x) &&
        (particlePosition.y > solidBounds.min.y && particlePosition.y < solidBounds.max.y) &&
        (particlePosition.z > solidBounds.min.z && particlePosition.z < solidBounds.max.z);
    }

    public bool movingBetweenSolids(Vector3 particlePosition) {
        
        MeshCollider sourceCollider = this.getSource().GetComponent<MeshCollider>();
        MeshCollider targetCollider = this.getTarget().GetComponent<MeshCollider>();

        Bounds sourceBounds = sourceCollider.bounds;
        Bounds targetBounds = targetCollider.bounds;

        Vector3 relativeTargetFromSourcePosition = this.getSource().transform.InverseTransformPoint(this.getTarget().transform.position);

        bool containedBetweenXAxis = false;
        bool containedBetweenZAxis = false;
        
        if (relativeTargetFromSourcePosition.x >= 0) {
            containedBetweenXAxis = (particlePosition.x > sourceBounds.min.x && particlePosition.x < targetBounds.max.x) &&
                                    (particlePosition.y > targetBounds.min.y && particlePosition.y < sourceBounds.max.y);
        } else {
            containedBetweenXAxis = (particlePosition.x > targetBounds.min.x && particlePosition.x < sourceBounds.max.x) &&
                                    (particlePosition.y > targetBounds.min.y && particlePosition.y < sourceBounds.max.y);
        }

        if (relativeTargetFromSourcePosition.z >= 0) {
            containedBetweenZAxis = (particlePosition.z > sourceBounds.min.z && particlePosition.z < targetBounds.max.z) &&
                                    (particlePosition.y > targetBounds.min.y && particlePosition.y < sourceBounds.max.y);
        } else {
            containedBetweenZAxis = (particlePosition.z > targetBounds.min.z && particlePosition.z < sourceBounds.max.z) &&
                                    (particlePosition.y > targetBounds.min.y && particlePosition.y < sourceBounds.max.y);
        }

        return containedBetweenXAxis && containedBetweenZAxis;
    }

    public bool restingOnWorkspace(Vector3 particlePosition) {
        Renderer workspaceRenderer = this.getWorkspace().GetComponent<Renderer>();

        Bounds workspaceBounds = workspaceRenderer.bounds;
        bool containedBetweenXAxis = false;
        bool containedBetweenZAxis = false;
        bool containedOnTopOfYXAxis = false;

        containedBetweenXAxis = particlePosition.x < workspaceBounds.max.x && particlePosition.x > workspaceBounds.min.x;
        containedBetweenZAxis = particlePosition.z < workspaceBounds.max.z && particlePosition.z > workspaceBounds.min.z;
        containedOnTopOfYXAxis = (particlePosition.y - workspaceBounds.max.z) < 0.01f;
        return containedBetweenXAxis && containedBetweenZAxis && containedOnTopOfYXAxis;
    }
}
