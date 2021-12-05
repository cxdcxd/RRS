using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;

/// <summary>
/// Class to control liquid in it's end2end life cycle. 
/// </summary>
public class Liquid : MonoBehaviour {
    private FlexContainer flexParticleContainer; // Flex particle container associated with this liquid.
    private FlexArrayAsset liquidAsset; // Flex liquid array asset.
    private FlexArrayActor liquidActor; // Flex array actor for liquid.
    private FlexParticleController liquidPC; // Flex particle controller for liquid.
    private ParticleSystem.Particle[] m_particles; // Array to keep each particle in liquid particle system.
    private GameObject liquid; // Gameobject of this liquid.
    private Robot liquidHandler; // Container which holds liquid.
    private string liquidName; // Name of the environment.
    private float weightOfParticle; // Weight of single liquid particle.
    private float density; // Density of this liquid in g/Ml

    public float getWeightOfParticle()
    {
        return this.weightOfParticle;
    }

    public void setWeightOfParticle(float weightOfParticle)
    {
        this.weightOfParticle = weightOfParticle;
    }

    public string getEnvironmentName()
    {
        return this.liquidName;
    }

    public void setEnvironmentName(string environmentName)
    {
        this.liquidName = "liquid_" + environmentName;
    }

    public Robot getLiquidHandler()
    {
        return this.liquidHandler;
    }

    public void setLiquidHandler(Robot liquidContainer)
    {
        this.liquidHandler = liquidContainer;
    } 

    /// <summary>
    /// Get gameobject of this liquid.
    /// </summary>
    /// <returns>Gameobject</returns>
    public GameObject getLiquid()
    {
        return this.liquid;
    }

    private void setLiquidObject()
    {
        this.liquid = new GameObject();
    }


    /// <summary>
    /// Gets all particles of this liquid. 
    /// </summary>
    /// <returns>Array of ParticleSystem.Particle</returns>
    public ParticleSystem.Particle[] getM_particles()
    {
        return this.m_particles;
    }

    /// <summary>
    /// Sets particles of this liquid.
    /// </summary>
    /// <param name="m_particles">Array ParticleSystem.Particle</param>
    public void setM_particles(ParticleSystem.Particle[] m_particles)
    {
        this.m_particles = m_particles;
    }

    /// <summary>
    /// Get property flex container.
    /// </summary>
    /// <returns>FlexContainer of this liquid.</returns>
    public FlexContainer getFlexParticleContainer()
    {
        return this.flexParticleContainer;
    }

    /// <summary>
    /// Sets property flex container.
    /// </summary>
    /// <param name="flexParticleContainer"> Flexcontainer to use with this liquid.</param>
    public void setFlexParticleContainer(FlexContainer flexParticleContainer)
    {
        this.flexParticleContainer = flexParticleContainer;
    }

    /// <summary>
    /// Get property liquid asset. 
    /// </summary>
    /// <returns>FlexArrayAsset</returns>
    public FlexArrayAsset getLiquidAsset()
    {
        return this.liquidAsset;
    }

    /// <summary>
    /// Sets property liquid asset.
    /// </summary>
    /// <param name="liquidAsset">FlexArrayAsset with appropriate boundary mesh and particle spacing</param>
    public void setLiquidAsset(FlexArrayAsset liquidAsset)
    {
        this.liquidAsset = liquidAsset;
    }

    /// <summary>
    /// Gets liquid as flex array actor.
    /// </summary>
    /// <returns>FlexArrayActor</returns>
    public FlexArrayActor getLiquidActor()
    {
        return this.liquidActor;
    }

    /// <summary>
    /// Sets liquid as flex array actor.
    /// </summary>
    /// <param name="liquidActor">FlexArrayActor</param>
    public void setLiquidActor(FlexArrayActor liquidActor)
    {
        this.liquidActor = liquidActor;
    }

    /// <summary>
    /// Gets Particle controller associated with this liquid.
    /// </summary>
    /// <returns>FlexParticleController associated</returns>
    public FlexParticleController getLiquidPC()
    {
        return this.liquidPC;
    }

    /// <summary>
    /// Sets Partucle controller to be used with this liquid.
    /// </summary>
    /// <param name="liquidPC">FlexParticleController to be attached.</param>
    public void setLiquidPC(FlexParticleController liquidPC)
    {
        this.liquidPC = liquidPC;
    }


    /// <summary>
    /// Method to get this liquid's density (in g/ml)
    /// </summary>
    /// <returns> Density of the liquid</returns>
    public float getDensity()
    {
        return this.density;
    }

    /// <summary>
    /// Method to set this liquid's density (in g/ml)
    /// </summary>
    /// <param name="density">Density to set</param>
    public void setDensity(float density)
    {
        this.density = density;
    }

    /// <summary>
    /// Creates the FLex liquid array actor.
    /// </summary>
    private void createLiquidActor()
    {
        // Add relevant components.
        FlexArrayActor liquidActor = this.getLiquid().AddComponent<FlexArrayActor>();
        liquidActor.container = this.getFlexParticleContainer();
        liquidActor.asset = this.getLiquidAsset();
        liquidActor.fluid = true;
        liquidActor.drawParticles = false;
        liquidActor.enabled = false;
        liquidActor.enabled = true;
        liquidActor.massScale = 1;
        this.getLiquid().AddComponent<FlexFluidRenderer>(); // Must for rendering fluid in the scene.
        // Set to crate array actor.
        this.setLiquidActor(liquidActor);
    }

    /// <summary>
    /// Attach a flex particle controller to this liquid to access particle level data.
    /// </summary>
    private void attachParticleSystem()
    {        
        // 1. Set particle controller
        FlexParticleController liquidPC = this.getLiquid().AddComponent<FlexParticleController>(); // Automatically adds Particle System. Provides abstraction for controlling particle system.  
        
        this.getLiquid().GetComponent<ParticleSystemRenderer>().enabled = false; // Always keep this off. Rendering handeled by FlexFluidRenderer.
        
        this.setM_particles(new ParticleSystem.Particle[this.getLiquidAsset().maxParticles]); // Initialize the particle array with length equal to maximum liquid particles.
        this.setLiquidPC(liquidPC);
        // 2. Set Particle positional state module for pouring
        this.getLiquid().AddComponent<ParticlePositionState>();
    }

    /// <summary>
    /// API to be used by client to create a liquid in scene with some desired set initial settings.
    public void createLiquid(float volume, float density = 1.0f, Solid liquid_pourer=null) {
        // Create new gameobject for liquid
        this.setLiquidObject();
        
        this.setDensity(density);
        // Provide appropriate name
        this.getLiquid().name = this.getEnvironmentName();

        // Adjust transformations and primitive shape for the spawned liquid.
        if (liquid_pourer == null)
        {
            liquid_pourer = this.getLiquidHandler().getHeldObject();
        }
        Renderer containerRenderer = liquid_pourer.getSolidObject().GetComponent<Renderer>();
        Bounds bounds = containerRenderer.bounds;
        float spawnHeight = 2.0f * bounds.extents.y;
        Vector3 liquidSpawnLocation = liquid_pourer.getSolidObject().transform.position + new Vector3(0f, spawnHeight, 0f);
        this.getLiquid().transform.position = liquidSpawnLocation;
        this.getLiquid().transform.rotation = liquid_pourer.transform.rotation;

        // Create flex array asset for this liquid.
        createLiquidActor();
        // Attach a particle system controller.
        attachParticleSystem();
        // Set particle weight
        float weight = (this.getDensity() * volume) / 1000f;
        weightOfParticle = weight / this.getLiquidAsset().maxParticles;
        this.setWeightOfParticle(weightOfParticle);
    }

    /// <summary>
    /// This method tracks relative positions of liquid particles w.r.t given source and target objects. Returns number of particles in:
    ///  1. Source object
    ///  2. Target object
    ///  3. In transition from the source to the target object.
    ///  4. Spilled i.e. not in source object, or target object or in transition from the source to the target object.
    /// </summary>
    /// <param name="sourceActor">Gameobject of the source object</param>
    /// <param name="targetActor">Gameobject of the target object</param>
    /// <param name="particleDictionary">Dictionary as argument in which result is updated.</param>
    /// <returns></returns>
    public Dictionary<string, float> getLiquidState(GameObject sourceActor, GameObject targetActor,
                                                    GameObject workspace, 
                                                    Dictionary<string, float> particleDictionary)
    {
        ParticleSystem ps = this.getLiquidPC().GetComponent<ParticleSystem>();
        
        int numParticlesAlive = ps.GetParticles(this.getM_particles());
        
        int AmtParticlesInSource = 0;
        int AmtParticlesInTarget = 0;
        int AmtParticlesBetweenSourceAndTarget = 0;
        int AmtParticlesSpilledOnWorkspace = 0;
        int AmtParticlesSpilled = 0;
        float transientVelocity = 0.0f;

        ParticlePositionState pps = this.getLiquid().GetComponent<ParticlePositionState>();
        pps.setSource(sourceActor);
        pps.setTarget(targetActor);
        pps.setWorkspace(workspace);

        if (numParticlesAlive > 0)
        {
            for (int i = 0; i < numParticlesAlive; i++)
            {
                if (pps.containedInSolid(this.getM_particles()[i].position, isSource: true))
                {
                    AmtParticlesInSource++;
                }
                else if (pps.containedInSolid(this.getM_particles()[i].position, isSource: false))
                {
                    AmtParticlesInTarget++;
                } 
                else if(pps.movingBetweenSolids(this.getM_particles()[i].position)) 
                {
                    if(this.getM_particles()[i].velocity.magnitude > 1.0f) {
                        transientVelocity += this.getM_particles()[i].velocity.magnitude;
                        AmtParticlesBetweenSourceAndTarget++;
                    } 
                }
                else if (pps.restingOnWorkspace(this.getM_particles()[i].position)) {
                    AmtParticlesSpilledOnWorkspace++;
                }
                else
                {
                    // Those not contained in either source or target are spilled.
                    AmtParticlesSpilled++;
                }
            }
        }

        // Update particle system with the alive particles.
        ps.SetParticles(this.getM_particles(), numParticlesAlive);
        
        if (AmtParticlesInSource > 0) {
            Vector3 force = AmtParticlesInSource * new Vector3(0, -this.getWeightOfParticle() * Physics.gravity.magnitude, 0);
            sourceActor.GetComponent<MeshCollider>().attachedRigidbody.AddForce(force, ForceMode.Force);
            particleDictionary["weightParticleInSource"] = AmtParticlesInSource * this.getWeightOfParticle();
        }

        if (AmtParticlesInTarget > 0) {
            Vector3 force = AmtParticlesInTarget * new Vector3(0, -this.getWeightOfParticle() * Physics.gravity.magnitude, 0);
            targetActor.GetComponent<MeshCollider>().attachedRigidbody.AddForce(force, ForceMode.Force);
            particleDictionary["weightParticleInTarget"] = AmtParticlesInTarget * this.getWeightOfParticle();
        }

        if (AmtParticlesBetweenSourceAndTarget > 0) {
            particleDictionary["weightParticlesPouring"] = AmtParticlesInTarget * this.getWeightOfParticle();
            particleDictionary["transientVelocity"] = 0f;
            if (AmtParticlesBetweenSourceAndTarget > 0)
                particleDictionary["transientVelocity"] = transientVelocity / AmtParticlesBetweenSourceAndTarget;
        }

        if (AmtParticlesSpilledOnWorkspace > 0) {
            Vector3 force = AmtParticlesSpilledOnWorkspace * new Vector3(0, -this.getWeightOfParticle() * Physics.gravity.magnitude, 0);
            workspace.GetComponent<BoxCollider>().attachedRigidbody.AddForce(force, ForceMode.Force);
            particleDictionary["weightOnWorkspace"] = AmtParticlesSpilledOnWorkspace * this.getWeightOfParticle();
        }

        if (AmtParticlesSpilled > 0) {
            particleDictionary["weightParticlesSpilled"] = AmtParticlesSpilled * this.getWeightOfParticle();
        }

        // Update results and return.
        return particleDictionary;
    }

    public void remove() {
        Destroy(this);
    }
} 