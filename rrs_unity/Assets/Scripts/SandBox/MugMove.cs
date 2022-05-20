using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

public class MugMove : MonoBehaviour
{

    public FlexSolidActor cup_actor;

    public FlexSolidActor bowl_actor;

    public FlexSolidActor inputActor;

    public FlexSolidActor cup_target;

	public FlexArrayActor liquid_actor = null;
    public GameObject liquid = null;
    public FlexArrayAsset liquid_asset;
	
    public FlexContainer container0;
    public FlexContainer container1;
    public FlexContainer container2;
    public FlexContainer container3;
    public FlexContainer container4;
    public FlexContainer container5;

    public FlexContainer inputContainer;
    public GameObject PivotOfMug2;

    public GameObject Scale;


    public Material Material1;

    public Material Material2;

    public FlexSolidAsset flexSolidAsset;


    public Mesh Mesh1;

    public Mesh Mesh2;

    public Mesh Mesh3;

    public Mesh Mesh4;

    GameObject inputActor2;

    ParticleSystem particle_system;

    ParticleSystemRenderer psr;

    public static string actorName = "cup";

    public static string targetName = "cup";


    public static string containerName = "type1";
    public static bool typeButtonPressed = false;

    // Start is called before the first frame update
    private float m_timer=0;

    public int incup_particles = 0;
    Vector3 pos3 = new Vector3(0,0,0); 

    private Quaternion currentRot;
    
    
    private void createLiquidActor(FlexContainer container)
    {
        if (liquid_actor != null) {
            Destroy(liquid);
        }

        Vector3 pos3 = cup_actor.GetComponent<MeshRenderer>().bounds.center;

        liquid = new GameObject("liquid");

        liquid.transform.position = pos3+new Vector3(0.4f,2,0.2f);
        liquid.transform.rotation = PivotOfMug2.transform.rotation; 
      
        liquid_actor = liquid.AddComponent<FlexArrayActor>();
        liquid_actor.container = container;
        liquid_actor.asset = liquid_asset;
        liquid_actor.fluid = true;

        liquid_actor.drawParticles = false;

        liquid_actor.enabled = false;
        liquid_actor.enabled = true;
        liquid_actor.massScale = 1;
	    liquid.AddComponent<FlexFluidRenderer>();


        particle_system = liquid.AddComponent<ParticleSystem>();
        var coll = particle_system.collision;

        coll.enabled = true;
        //coll.bounce = 3.0f;
        coll.type = ParticleSystemCollisionType.World;
        coll.colliderForce = 0;
        coll.multiplyColliderForceByParticleSpeed = false;
        coll.multiplyColliderForceByCollisionAngle = false;
        coll.multiplyColliderForceByParticleSize = false;
        coll.sendCollisionMessages = true;
        coll.enableDynamicColliders = true;

        //var force_over_lifetime = particle_system.forceOverLifetime;
        //force_over_lifetime.enabled = true;
        //force_over_lifetime.space = ParticleSystemSimulationSpace.World;
        //force_over_lifetime.y = 1;

        liquid.GetComponent<ParticleSystemRenderer>().enabled = false;
        liquid.AddComponent<FlexParticleController>();

        var detector = liquid.AddComponent<ParticleCollisionDetector>();
        detector.Scale = Scale;
    }

    private void containerType(){
        
        if(containerName == "type1"){

            inputContainer = container0;
        }
        else if(containerName == "type2"){
            inputContainer = container1;
        }
        else if (containerName == "type3")
        {
            inputContainer = container2;
        }
        else if (containerName == "type4")
        {
            inputContainer = container3;
        }
        else if (containerName == "type5")
        {
            inputContainer = container4;
        }
        else if (containerName == "type6")
        {
            inputContainer = container5;
        }
      

    }

    void ConfigureActorComponents(GameObject recipient,string actorName){
    MeshRenderer mr = recipient.AddComponent<MeshRenderer>();

    if(actorName == "cup"){
        // Configure mr the way you want it...
    recipient.transform.position = new Vector3(-5.0f,3.5f,0.04f);
    recipient.transform.localScale = new Vector3(1.5f,1.5f,1.5f);
    mr.sharedMaterial = Material1;

    recipient.GetComponent<MeshFilter>().sharedMesh = Mesh1;
    recipient.GetComponent<MeshCollider>().sharedMesh = Mesh3;
    // recipient.GetComponent<MeshCollider>().convex = true;
    
    // recipient.transform.position = new Vector3(4.5f,3.5f,0.04f);

    }
    else if(actorName == "bowl"){
        
    recipient.transform.position = new Vector3(-5.0f,5.0f,0.0f);
    recipient.transform.localScale = new Vector3(1.5f,1.5f,1.5f);

    mr.sharedMaterial = Material2;
    recipient.GetComponent<MeshFilter>().sharedMesh = Mesh2;
    recipient.GetComponent<MeshCollider>().sharedMesh = Mesh4;

    }
    }

    void ConfigureTargetComponents(GameObject recipient,string targetName){
    MeshRenderer mr = recipient.AddComponent<MeshRenderer>();

    if(targetName == "cup"){
        // Configure mr the way you want it...
    recipient.transform.position = new Vector3(-0.8600509f, 2.0f,0.04f);
    recipient.transform.localScale = new Vector3(1.5f,1.5f,1.5f);

    mr.sharedMaterial = Material1;
     recipient.GetComponent<MeshFilter>().sharedMesh = Mesh1;
    recipient.GetComponent<MeshCollider>().sharedMesh = Mesh3;
    // recipient.transform.position = new Vector3(4.5f,3.5f,0.04f);
    recipient.GetComponent<MeshCollider>().convex = true;


    }

    else if(targetName == "bowl"){
        
    recipient.transform.position = new Vector3(-0.0f,3.90f,0.0f);
    recipient.transform.localScale = new Vector3(1.5f,1.5f,1.5f);

    mr.sharedMaterial = Material2;
    recipient.GetComponent<MeshFilter>().sharedMesh = Mesh2;
    recipient.GetComponent<MeshCollider>().sharedMesh = Mesh4;
    recipient.GetComponent<MeshCollider>().convex = true;

    }
    }
    
    

    private void createActor(string actorName){
        GameObject inputActor1;
         //spawn object
         inputActor1 = new GameObject(actorName);
         inputActor1.transform.parent = PivotOfMug2.transform;
         //Add Components
         inputActor1.AddComponent<MeshFilter>();
         inputActor1.AddComponent<MeshCollider>();
        ConfigureActorComponents(inputActor1,actorName);
    }

     private void createTarget(string targetName){
        
         inputActor2 = new GameObject(targetName);        
         inputActor2.AddComponent<MeshFilter>();
         inputActor2.AddComponent<MeshCollider>();
         inputActor2.AddComponent<Rigidbody>();
         inputActor2.GetComponent<Rigidbody>().useGravity =true;
         inputActor2.GetComponent<Rigidbody>().isKinematic =false;
         inputActor2.GetComponent<Rigidbody>().mass = 10.0f;
         ConfigureTargetComponents(inputActor2,targetName);

    }


    private static Vector3 getRelativePosition(Transform  origin, Vector3 position)
    {
        Vector3 distance = position - origin.position;
        Vector3 relativePosition = Vector3.zero;
        Debug.Log("origin position :"+ origin.position);
        Debug.Log("particle position :"+ position);
        // this is the relative position in the "origin frame"
        relativePosition.x = Vector3.Dot(distance, origin.right.normalized);
        relativePosition.y = Vector3.Dot(distance, origin.up.normalized);
        relativePosition.z = Vector3.Dot(distance, origin.forward.normalized);

        return relativePosition;
    }

    private void nbOfParticlesWithin(ref GameObject inputActor2)
    {
        //float max_y_position = -3.0f;
        incup_particles = 0;
        // the list of the index of the particle
        List<int> inCupIndices = new List<int>();

        // Vector3 inputActor2Relative = inputActor2.transform.TransformPoint(transform.position);
        

        ParticleSystem part = liquid.GetComponent<ParticleSystem>();

        ParticleSystem.Particle[] particles = new ParticleSystem.Particle[part.particleCount];
        int n = part.GetParticles(particles);

        

        for (int i = 0; i < n; i++)
        {
            // print("particle position from system:" +particles[i].position);

            Vector3 rel_pos = inputActor2.transform.InverseTransformPoint(particles[i].position);
            // Vector3 rel_pos = getRelativePosition(inputActor2.transform, liquid_asset.particles[i]);
            if(rel_pos.y<-10.0f){
                continue;
            }
            // print("particle rel_pos:" +rel_pos);
            
            if(targetName == "cup"){
                // rel_pos += new Vector3(9.5f,-1.0f,0);

                 if (rel_pos.x >= -1.5f && rel_pos.x <= 1.5f &&
                rel_pos.z >= -1.5f && rel_pos.z <= 1.5f &&
                rel_pos.y >= -1.65f && rel_pos.y <= 1.65f)
            {
                
                    incup_particles += 1;
                    inCupIndices.Add(i);
                   
            }
            else{
                //print("particle outside:" +rel_pos);
            }

            }

            if(targetName == "bowl"){
                

                // max_x= Mathf.Max(max_x,rel_pos.x);
                // max_y= Mathf.Max(max_y,rel_pos.y);

                // max_z= Mathf.Max(max_z,rel_pos.z);
                 //print("particle rel_pos rejusted:" +rel_pos);
           
            if (rel_pos.x >= -1.6f && rel_pos.x <= 1.6f &&
                rel_pos.z >= -1.6f && rel_pos.z <= 1.6f &&
                rel_pos.y >= -1.65f && rel_pos.y <= 1.65f)
            {
                
                    incup_particles += 1;
                    inCupIndices.Add(i);
                   
            }
            else{
                //print("particle outside:" +rel_pos);
            }



            }
           

        }
        // print("max_x:" +max_x+"max_y:" +max_y+"max_z:" +max_z);

        //print("incup_particles:" +incup_particles);
    }


    

    private void reset()
    {

        createLiquidActor(container0);

    }
    void Start()
    {   

        bowl_actor.drawParticles =false;
        createActor(actorName);
        createTarget(targetName);
        containerType();
        createLiquidActor(inputContainer);

    }

    // Update is called once per frame
    void Update()
    {
        
        m_timer+=Time.deltaTime;
		if(m_timer>3){
                if(PivotOfMug2.transform.rotation.z>-0.78f){
             PivotOfMug2.transform.Rotate(Vector3.back,45*Time.deltaTime,Space.Self);
        }
        }

        if(m_timer>6){
            nbOfParticlesWithin(ref inputActor2);
        }


      

    }





}
