//Put this script on your blue cube.

using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class PushAgentBasic : Agent
{
    /// <summary>
    /// The ground. The bounds are used to spawn the elements.
    /// </summary>
    public GameObject ground;

    public GameObject area;
    public Haptic haptic_ref;

    /// <summary>
    /// The area bounds.
    /// </summary>
    [HideInInspector]
    public Bounds areaBounds;

    PushBlockSettings m_PushBlockSettings;

    /// <summary>
    /// The goal to push the block to.
    /// </summary>
    public GameObject goal;

    /// <summary>
    /// The block to be pushed to the goal.
    /// </summary>
    public GameObject block;

    /// <summary>
    /// Detects when the block touches the goal.
    /// </summary>
    [HideInInspector]
    public GoalDetect goalDetect;

    public bool useVectorObs;

    Rigidbody m_BlockRb;  //cached on initialization
    Rigidbody m_AgentRb;  //cached on initialization
    Material m_GroundMaterial; //cached on Awake()

    /// <summary>
    /// We will be changing the ground material based on success/failue
    /// </summary>
    Renderer m_GroundRenderer;

    EnvironmentParameters m_ResetParams;

    void Awake()
    {
        m_PushBlockSettings = FindObjectOfType<PushBlockSettings>();
    }

    public override void Initialize()
    {
        goalDetect = block.GetComponent<GoalDetect>();
        goalDetect.agent = this;

        // Cache the agent rigidbody
        m_AgentRb = GetComponent<Rigidbody>();
        // Cache the block rigidbody
        m_BlockRb = block.GetComponent<Rigidbody>();
        // Get the ground's bounds
        areaBounds = ground.GetComponent<Collider>().bounds;
        // Get the ground renderer so we can change the material when a goal is scored
        m_GroundRenderer = ground.GetComponent<Renderer>();
        // Starting material
        m_GroundMaterial = m_GroundRenderer.material;

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        SetResetParameters();
    }
    /// <summary>
    /// Added to collect Observation about the actual orientation and velocity of the agent and the actual position of the block 
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        //Agent, block and goal position
        sensor.AddObservation(this.transform.localPosition - block.transform.localPosition);
        //UnityEngine.Debug.Log(this.transform.localPosition - block.transform.localPosition);
        sensor.AddObservation(block.transform.localRotation);
        sensor.AddObservation(goal.transform.localPosition);
        //sensor.AddObservation(block.transform.localPosition);
        //if (goal.transform.localPosition.x == 0)
        //{
        //    sensor.AddObservation(goal.transform.localPosition.z - block.transform.localPosition.z);
        //}
        //else if (goal.transform.localPosition.z == 0)
        //{
        //    sensor.AddObservation(goal.transform.localPosition.x - block.transform.localPosition.x);
        //}
        //Agent velocity
        //sensor.AddObservation(m_BlockRb.velocity);
    }
    /// <summary>
    /// Use the ground's bounds to pick a random spawn position.
    /// </summary>
    public Vector3 GetRandomSpawnPos()
    {
        var foundNewSpawnLocation = false;
        var randomSpawnPos = Vector3.zero;
        while (foundNewSpawnLocation == false)
        {
            var randomPosX = Random.Range(-areaBounds.extents.x * m_PushBlockSettings.spawnAreaMarginMultiplier,
                areaBounds.extents.x * m_PushBlockSettings.spawnAreaMarginMultiplier);

            var randomPosZ = Random.Range(-areaBounds.extents.z * m_PushBlockSettings.spawnAreaMarginMultiplier,
                areaBounds.extents.z * m_PushBlockSettings.spawnAreaMarginMultiplier);
            randomSpawnPos = ground.transform.position + new Vector3(randomPosX, 1f, randomPosZ);
            if (Physics.CheckBox(randomSpawnPos, new Vector3(2.5f, 0.01f, 2.5f)) == false)
            {
                foundNewSpawnLocation = true;
            }
        }
        return randomSpawnPos;
    }

    /// <summary>
    /// Called when the agent moves the block into the goal.
    /// </summary>
    public void ScoredAGoal()
    {
        // We use a reward of 5.
        AddReward(5f);

        // By marking an agent as done AgentReset() will be called automatically.
        EndEpisode();

        // Swap ground material for a bit to indicate we scored.
        StartCoroutine(GoalScoredSwapGroundMaterial(m_PushBlockSettings.goalScoredMaterial, 0.5f));
    }

    /// <summary>
    /// Called when the agent collides with the block. 
    /// </summary>
    //public void OnCollisionEnter(Collision col)
    //{
    //    if (col.gameObject.name == "Block")
    //        // We use a reward of 0.1.
    //    {
    //AddReward(0.01f);

    //}
    /// <summary>
    /// Swap ground material, wait time seconds, then swap back to the regular material.
    /// </summary>
    IEnumerator GoalScoredSwapGroundMaterial(Material mat, float time)
    {
        m_GroundRenderer.material = mat;
        yield return new WaitForSeconds(time); // Wait for 2 sec
        m_GroundRenderer.material = m_GroundMaterial;
    }

    /// <summary>
    /// Moves the agent according to the selected action.
    /// </summary>
    public void MoveAgent(float[] act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        var action = Mathf.FloorToInt(act[0]);

        //print("Move Agent " + action);
        //print("mode " + Haptic.system_mode);
        //print("Dof " + Haptic.system_dof);
        // print("Dof " + Haptic.system_dof);

        if (Haptic.system_mode == SystemMode.TeleoperationSim || Haptic.system_mode == SystemMode.Training || Haptic.system_mode == SystemMode.TestingSim)
        {
            if (Haptic.system_dof == DoF.three)
            {
                // Goalies and Strikers have slightly different action spaces.
                switch (action)
                {
                    case 1:
                        dirToGo = transform.forward * 1f;
                        break;
                    case 2:
                        dirToGo = transform.forward * -1f;
                        break;
                    case 3:
                        rotateDir = transform.up * 1f;
                        break;
                    case 4:
                        rotateDir = transform.up * -1f;
                        break;
                    case 5:
                        dirToGo = transform.right * -0.75f;
                        break;
                    case 6:
                        dirToGo = transform.right * 0.75f;
                        break;
                }

                transform.Rotate(rotateDir, Time.fixedDeltaTime * 200f);

                m_AgentRb.AddForce(dirToGo * m_PushBlockSettings.agentRunSpeed, ForceMode.VelocityChange);

            }
            else if ( Haptic.system_dof == DoF.two)
            {
                // Goalies and Strikers have slightly different action spaces.
                switch (action)
                {
                    case 4:
                        dirToGo = transform.forward * 1f;
                        break;
                    case 3:
                        dirToGo = transform.forward * -1f;
                        break;
                    case 1:
                        dirToGo = transform.right * 1f;
                        break;
                    case 2:
                        dirToGo = transform.right * -1f;
                        break;
                }

                m_AgentRb.AddForce(dirToGo * m_PushBlockSettings.agentRunSpeed, ForceMode.VelocityChange);
            }
        }
        else if (Haptic.system_mode == SystemMode.TestingReal || Haptic.system_mode == SystemMode.TeleoperationReal)
        {
            //Real
            HapticCommand cmd = new HapticCommand();
            cmd.velocity_linear = new RVector3();
            cmd.velocity_angular = new RVector3();


            if (Haptic.system_dof == DoF.two)
            {
                // Goalies and Strikers have slightly different action spaces.
                switch (action)
                {
                    case 4:
                        cmd.velocity_linear.z = 1;
                        break;
                    case 3:
                        cmd.velocity_linear.z = -1;
                        break;
                    case 1:
                        cmd.velocity_linear.x = 1;
                        break;
                    case 2:
                        cmd.velocity_linear.x = -1;
                        break;
                }
            }
            else
            {
                //TDB
            }

            //cmd.velocity_linear.x = dirToGo.x; //right and left
            //cmd.velocity_linear.z = dirToGo.z; //up and down#

            print(cmd.velocity_linear.x + " " + cmd.velocity_linear.z);

            print("Send data to robot " );
            Statics2.network_real.sendMessage(cmd);
        }
        
    }

    /// <summary>
    /// Called every step of the engine. Here the agent takes an action.
    /// </summary>
    public override void OnActionReceived(float[] vectorAction)
    {
        // Move the agent using the action.
        MoveAgent(vectorAction);

        // Penalty given each step to encourage agent to finish task quickly.
        AddReward(-1f / MaxStep);
    }

    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = 0;
      
        if (haptic_ref.current_speed.x > 0 && haptic_ref.current_speed.z == 0)
        {
            actionsOut[0] = 1; //Right
        }
        else if (haptic_ref.current_speed.x < 0 && haptic_ref.current_speed.z == 0)
        {
            actionsOut[0] = 2; //Left
        }
        else if (haptic_ref.current_speed.z > 0 && haptic_ref.current_speed.x == 0)
        {
            actionsOut[0] = 4; //Up
        }
        else if (haptic_ref.current_speed.z < 0 && haptic_ref.current_speed.x == 0)
        {
            actionsOut[0] = 3; //Down
        }
        else
        {
            actionsOut[0] = 0; //Stop
        }
      
    }

    /// <summary>
    /// Resets the block position and velocities.
    /// </summary>
    void ResetBlock()
    {
        // Get a random position for the block.
        block.transform.position = GetRandomSpawnPos();

        // Reset block velocity back to zero.
        m_BlockRb.velocity = Vector3.zero;

        // Reset block angularVelocity back to zero.
        m_BlockRb.angularVelocity = Vector3.zero;
    }

    /// <summary>
    /// In the editor, if "Reset On Done" is checked then AgentReset() will be
    /// called automatically anytime we mark done = true in an agent script.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        //var rotation = 0;
        //rotation = Random.Range(0, 4);
        //UnityEngine.Debug.Log(rotation);
        //float rotationAngle = 0f;
        //rotationAngle = rotation * 90f;
        //UnityEngine.Debug.Log(rotationAngle);
        //goal.transform.Rotate(0.0f, 90.0f, 0.0f);

        //if (goal.transform.rotation.eulerAngles.y == 0)
        //{
        //    goal.transform.localPosition = new Vector3(0f, -0.03f, -10.3f);
        //}
        //else if (goal.transform.rotation.eulerAngles.y == 90.0f)
        //{
        //    goal.transform.localPosition = new Vector3(10.3f, -0.03f, 0f);
        //}
        //else if (goal.transform.rotation.eulerAngles.y == 180.0f)
        //{
        //    goal.transform.localPosition = new Vector3(0f, -0.03f, 10.3f);
        //}
        //else if (goal.transform.rotation.eulerAngles.y == 270.0f)
        //{
        //    goal.transform.localPosition = new Vector3(-10.3f, -0.03f, 0f);
        //}

        ResetBlock();
        transform.position = GetRandomSpawnPos();
        m_AgentRb.velocity = Vector3.zero;
        m_AgentRb.angularVelocity = Vector3.zero;
         

        SetResetParameters();
    }

    public void SetGroundMaterialFriction()
    {
        var groundCollider = ground.GetComponent<Collider>();

        groundCollider.material.dynamicFriction = m_ResetParams.GetWithDefault("dynamic_friction", 0);
        groundCollider.material.staticFriction = m_ResetParams.GetWithDefault("static_friction", 0);
    }

    public void SetBlockProperties()
    {
        //var scale = m_ResetParams.GetWithDefault("block_scale", 2);
        //Set the scale of the block
        //m_BlockRb.transform.localScale = new Vector3(scale, 0.75f, scale);

        // Set the drag of the block
        m_BlockRb.drag = m_ResetParams.GetWithDefault("block_drag", 0.5f);
    }

    void SetResetParameters()
    {
        SetGroundMaterialFriction();
        SetBlockProperties();
    }
}
