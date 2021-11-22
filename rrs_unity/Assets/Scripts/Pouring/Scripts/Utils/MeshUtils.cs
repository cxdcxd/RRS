using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MeshUtils : MonoBehaviour {

    public Mesh mesh;
    public Mesh rescaleMesh(float scale) {
        var baseVertices = mesh.vertices;
        var vertices = new Vector3[baseVertices.Length];
        for (var i=0;i<vertices.Length;i++)
        {
            var vertex = baseVertices[i];
            vertex.x = vertex.x * scale;
            vertex.y = vertex.y * scale;
            vertex.z = vertex.z * scale;
            vertices[i] = vertex;
        }
 
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

} 