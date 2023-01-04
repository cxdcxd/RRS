using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NVIDIA.Flex;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.Barracuda;

public class PouringAgentAINoPSLT : Agent
{

    /// <summary>
    /// The objects below are to be provided in runtime.
    /// </summary>
    public GameObject robot; // Robot game object with robot parts as assets.
    public GameObject sources; // Set of source objects to hold liquid for pouring.
    public GameObject targets; // Set of target objects in which liquid is poured.
    public FlexContainer liquid_flex_container; // Flex container as a slover for all flex objects.
    public FlexArrayAsset liquidAsset; // FLex array asset for liquid object.
    public PhysicMaterial solidObjectPhysicsMaterial; // Physics material to use with source and target containers.
    public PhysicMaterial robotPhysicsMaterial; // Physics material to use with robotic hand.
    public GameObject workspace; // Workspace for target hand.
    public bool render_fluid = true;
    private int configureAgent = -1;
    public NNModel model;
    public float calibration_target_offset = 10;

    /// <summary>
    /// The objects below are created and used internally during the runtime.
    /// </summary>

    private SolidLT source; // Source container that holds the liquid.
    private SolidLT target; // Target container which receives the liquid.
    private LiquidLT liquid; // Liquid object with domain randomized properties.
    private RobotLT robotHandForSource; // Robotic hand to hold the source container.
    private RobotLT robotHandForTarget; // Robotic hand to hold the target container.

    /// <summary>
    /// Objects used during runtime for end to end state and action tracking.
    /// </summary>    
    EnvironmentParameters defaultAcademyParameters; // ML-Agent's environment parameters.
    private string sourceName; // Unique name to identify source liquid container.
    private string targetName; // Unique name to identify target liquid container.
    public float target_to_fill; // Target amount of liquid to fill.
    private float differenceFillLevel; // Fill level in the target container as a ratio of the desired target.
    private float originalWeight; // Original quantity of liquid in source liquid container.
    Dictionary<string, float> liquidState = new Dictionary<string, float>();
    private int numEpisodes = 0; // Counter to track number of episodes.
    private int numCompletedEpisodes = 0; // Counter to track number of completed episodes.
    private float timer = 0.0f;
    private bool isActionDone = false;
    private float liquidInSource = 0.0f;
    private float error = 0.0f;
    float weightOfLiquid = 0.0f;
    private static List<float> fill_targets = new List<float>();
    bool isTest = true;
    int experiment_indexer = 0;
    public bool behaviorMode = false;
    private GameObject clickedPourer;
    private static float sourceStartVolume = 250f;

    /// <summary>
    /// The methods below are used to spawn environment actors in the scene
    /// </summary>

    /// <summary>
    /// Creates Solid Game Object. Based on string provided, chooses source or target.
    /// </summary>
    /// <param name="name"> Name of the soild actor to be generated.</param>

    private void createSolidObjects(string name)
    {
        if (name == "SourceObject_" + gameObject.name)
        {
            source = SpawnObjectsLT.createLiquidContainer(source, this.gameObject, sources, name, solidObjectPhysicsMaterial, clickedPourer, randomizePositions: false);
        }
        else if (name == "TargetObject_" + gameObject.name)
        {
            target = SpawnObjectsLT.createLiquidContainer(target, this.gameObject, targets, name, solidObjectPhysicsMaterial, null, randomizePositions: false, true);
        }
        else print(name + " not configured for flex solid actor..");
    }

    private void ResetLiquidStateInformation()
    {
        if (liquidState != null)
        {
            liquidState = null;
        }

        liquidState = new Dictionary<string, float>();
    }

    /// <summary>
    /// Called once when ML agents training start. Initializes academy parameters and starts the scene.
    /// </summary>
    public override void Initialize()
    {
        defaultAcademyParameters = Academy.Instance.EnvironmentParameters;
        StartScene();
    }

    /// <summary>
    /// Called once. Initialises names, and resets the scene.
    /// </summary>
    private void StartScene()
    {
        if (isTest)
            experiment_indexer = 0;

        fill_targets.Add(0.05f);
        fill_targets.Add(0.043f);
        fill_targets.Add(0.12f);
        fill_targets.Add(0.135f);
        fill_targets.Add(0.15f);
        fill_targets.Add(0.175f);
        fill_targets.Add(0.2f);
        fill_targets.Add(0.01f);
        fill_targets.Add(0.11f);
        fill_targets.Add(0.15f);
        fill_targets.Add(0.132f);
        fill_targets.Add(0.05f);
        fill_targets.Add(0.043f);
        fill_targets.Add(0.12f);
        fill_targets.Add(0.135f);
        fill_targets.Add(0.15f);
        fill_targets.Add(0.175f);
        fill_targets.Add(0.2f);
        fill_targets.Add(0.01f);
        fill_targets.Add(0.11f);
        fill_targets.Add(0.15f);
        fill_targets.Add(0.132f);
        fill_targets.Add(0.01f);
        fill_targets.Add(0.11f);
        fill_targets.Add(0.15f);
        fill_targets.Add(0.132f);

        sourceName = "SourceObject_" + this.gameObject.name;
        targetName = "TargetObject_" + this.gameObject.name;
    }

    /// <summary>
    /// Resets the scene.
    /// </summary>
    public void ResetScene()
    {
        // Initialise goals
        differenceFillLevel = 0.0f;
        timer = 0.0f;
        isActionDone = false;
        liquidInSource = 0.0f;
        sourceTilt = 0.0f;
        weightOfLiquid = 0.0f;
        previousStepSourceLevel = 0.0f;
        discharge = 0.0f;
        retract = false;
        retractTimer = 0.0f;
        toleranceInPouringDeviation = Academy.Instance.EnvironmentParameters.GetWithDefault("tolerance", 0.0f);
        // 1. Spawn robotic arms
        robotHandForSource = SpawnObjectsLT.createRoboticArms(robotHandForSource, this.gameObject, robot, "sourceHand", robotPhysicsMaterial);
        robotHandForTarget = SpawnObjectsLT.createRoboticArms(robotHandForTarget, this.gameObject, robot, "targetHand", robotPhysicsMaterial);

        initialPosition = 0.5f * (robotHandForSource.getRobotHand().transform.TransformPoint(robotHandForSource.getFingerA().transform.localPosition) +
                                        robotHandForSource.getRobotHand().transform.TransformPoint(robotHandForSource.getFingerB().transform.localPosition));
        initialRotation = robotHandForSource.getFingerB().transform.rotation;

        // 2. Spawn solid objects
        createSolidObjects(sourceName);
        createSolidObjects(targetName);

        //source.getSolidObject().transform.position = initialPosition;


        // // 3. align robots with solid objects
        robotHandForSource.holdObject(source, workspace, true, true);
        robotHandForTarget.holdObject(target, workspace, true, true, true);

        robotHandForSource.transform.position = initialPosition;

        // 4. Create liquid
        if (isTest)
        {
            print("RUNNING INFERENCE EXPERIMENTS ON TRAINED AGENT!!!");
            if (experiment_indexer == 10)
            {
                print("completed " + experiment_indexer + " experiments!");
                //Application.Quit();

            }
            else
            {
                target_to_fill = fill_targets[experiment_indexer];
                experiment_indexer++;
                float density = 1.0f;
                //Color color = Color.clear;

                switch (liquid_flex_container.name)
                {
                    case "Water":
                        density = 1.0f;
                        // color = colors[choice];
                        //color = new Color(0.0549019608f, 0.529411765f, 0.8f);
                        break;
                    case "Ink":
                        density = 1.0081f;
                        //color = new Color(0.0274509804f, 0.0509803922f, 0.0509803922f);
                        break;
                    case "Oil":
                        density = 0.917f;
                        //color = new Color(1f, 0.996078431f, 0.250980392f);
                        break;
                    case "Honey":
                        density = 1.42f;
                        //color = new Color(255, 255, 255);
                        break;
                    case "Glycerin":
                        density = 1.26f;
                        //color = new Color(0.847058824f, 0.862745098f, 0.839215686f);
                        break;
                    case "Ketchup":
                        density = 1.092f;
                        //color = new Color(0.925490196f, 0.176470588f, 0.00392156863f);
                        break;
                    case "Milk":
                        density = 1.0f;
                        //color = new Color(0.925490196f, 0.176470588f, 0.00392156863f);
                        break;
                    case "Shampoo":
                        density = 1.26f;
                        //color = new Color(0.925490196f, 0.176470588f, 0.00392156863f);
                        break;
                    case "Handgel":
                        density = 1.3f;
                        //color = new Color(0.925490196f, 0.176470588f, 0.00392156863f);
                        break;

                }
                //color.a = 0.75f;
                liquid = SpawnObjectsLT.createLiquid(liquid, robotHandForSource, this.gameObject, liquid_flex_container, liquidAsset, sourceStartVolume, density);
                //liquid.getFlexParticleContainer().fluidMaterial.color = color;
                print("Liquid is: " + liquid.getFlexParticleContainer().name);
            }
        }
        else
        {
            // int choice = Random.Range(0, 8);
            target_to_fill = Random.Range(0.05f, 0.25f); //5 g to 250 g
            float density = Academy.Instance.EnvironmentParameters.GetWithDefault("density", 0.0f);
            liquid = SpawnObjectsLT.createLiquid(liquid, robotHandForSource, this.gameObject, liquid_flex_container, liquidAsset, sourceStartVolume, density);
            liquid.getFlexParticleContainer().fluidRest = Academy.Instance.EnvironmentParameters.GetWithDefault("fluid_rest_distance", 0.0f);
            liquid.getFlexParticleContainer().cohesion = Academy.Instance.EnvironmentParameters.GetWithDefault("cohesion", 0.0f);
            liquid.getFlexParticleContainer().surfaceTension = Academy.Instance.EnvironmentParameters.GetWithDefault("surfaceTension", 0.0f);
            liquid.getFlexParticleContainer().viscosity = Academy.Instance.EnvironmentParameters.GetWithDefault("viscosity", 0.0f);
            liquid.getFlexParticleContainer().adhesion = Academy.Instance.EnvironmentParameters.GetWithDefault("adhesion", 0.0f);
            liquid.getFlexParticleContainer().fluidMaterial.color = Color.Lerp(Color.green, Color.blue, (liquid.getFlexParticleContainer().viscosity / 200.0f));
        }
        originalWeight = 0.0f;
        robotHandForTarget.getRobotHand().GetComponent<ForceInformationLT>().targetWeight = target_to_fill;

        // 5. Reset liquid state information
        ResetLiquidStateInformation();

        // 6. 
        Statics.movo_mini_ref.right_gripper = robotHandForSource.getRobotHand();
        Statics.movo_mini_ref.left_gripper = robotHandForTarget.getRobotHand();

    }

    public void setPouringContainer(GameObject pouringContainer)
    {
        this.clickedPourer = pouringContainer;
    }

    public void setSelectedLiquid(FlexContainer container)
    {
        this.liquid_flex_container = container;
    }

    /// <summary>
    /// Tasks to do when episode begins.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        numEpisodes += 1;
        print("Episode number: " + numEpisodes);
        ResetScene();
    }
    /// <summary>
    /// Adds sensor observations used by PPO RL Agent to make actions.
    /// Overall 6 observations.
    /// </summary>
    /// <param name="sensor"> Vector sensor observations.</param>
    public override void CollectObservations(VectorSensor sensor)
    {
        if (source != null)
        {
            sensor.AddObservation(source.getObjectDiameter()); // 1 Observation for pouring object's diamater in meters.
            sensor.AddObservation(source.getObjectDepth()); // 1 observation for pouring object's height in meters.
            sensor.AddObservation(liquid.getDensity()); // 1 Observation for liquid's density.
            sensor.AddObservation(liquid.getFlexParticleContainer().cohesion); // 1 observation for cohesion of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().surfaceTension); // 1 observation for surface tension of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().viscosity); // 1 observation for normalised viscosity of fluid.
            sensor.AddObservation(liquid.getFlexParticleContainer().adhesion); // 1 observation for adhesion of the fluid.

            sensor.AddObservation(target_to_fill); // 1 objservation for target weight to fill.
            sensor.AddObservation(0); // 1 observation for weight of the liquid in source container.
            sensor.AddObservation(0); // 1 observation for mass flow rate. 
            sensor.AddObservation(weightOfLiquid); // 1 observation for the current liquid weight in the target container.
            sensor.AddObservation(differenceFillLevel); // 1 observation for current difference from the target level.
            sensor.AddObservation(sourceTilt / 360); //  1 observation for current tilt of the source container about it's axis of rotation. {Action feedback}
            if (numCompletedEpisodes > 0)
                sensor.AddObservation((error / numCompletedEpisodes)); // 1 observation for average pouring error.
            else
                sensor.AddObservation(0.0f); // 1 observation for average pouring error.
        }
    }

    /// <summary>
    /// Tasks to do when an action is received from the PPO. Actions are continuous domain. 
    /// </summary>
    /// <param name="actions"> Actions received from PPO agent.</param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Grasping at handle.
        timer += Time.deltaTime;
        liquidState = liquid.getLiquidState(source.getSolidObject(), target.getSolidObject(), workspace, liquidState);
        if (timer > 8.0f)
        {
            performPouringAction(actions);
            AddReward(-1.0f / MaxStep);
        }
    }

    /// <summary>
    /// Method to perform pouring action, track states and add rewards.
    /// </summary>

    float toleranceInPouringDeviation = 0.0f;
    float sourceTurningSpeed;
    float robotMovingSpeed;
    float sourceTilt = 0.0f;
    float previousStepSourceLevel = 0.0f;
    Vector3 initialPosition = Vector3.zero;
    Quaternion initialRotation = Quaternion.identity;
    float discharge = 0.0f;
    bool retract = false;
    float retractTimer = 0;
    private void performPouringAction(ActionBuffers actionBuffers)
    {
        bool isReset = (target == null) || (source == null);
        if (!isReset)
        {
            if (target.getSolidObject().transform.position.y < workspace.transform.position.y)
            {
                print("Workspace does not contain target. Ending!!");
                EndEpisode();
            }
            // Get actions from agent's action buffer. Overall 4 actions. Move in x, y, z and rotate speed while pouring.
            int actionIndex = -1;

            sourceTurningSpeed = 50.0f * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // rotate source by this angle about axis of rotation defined by relative position between source and target.
            robotMovingSpeed = 2.0f * Mathf.Clamp(actionBuffers.ContinuousActions[++actionIndex], -1f, 1f); // Move robotic hand by this speed.

            weightOfLiquid = robotHandForTarget.getRobotHand().GetComponent<ForceInformationLT>().getMeasuredWeight();

            if (weightOfLiquid > 0.01f)
                differenceFillLevel = Mathf.Abs(target_to_fill - weightOfLiquid);

            float filledLevel = (float)weightOfLiquid / target_to_fill;
            float similarity = Vector3.Dot(source.getSolidObject().transform.up, target.getSolidObject().transform.up);

            if (originalWeight == 0.0f)
            {
                originalWeight = robotHandForSource.getRobotHand().GetComponent<ForceInformationLT>().getMeasuredWeight(); ;
                if (originalWeight < target_to_fill)
                {
                    float targetOffset = Random.Range(0.01f, 0.025f);
                    target_to_fill = originalWeight - targetOffset;
                    if (target_to_fill < 0.01f)
                    {
                        target_to_fill = targetOffset;
                    }
                }
            }



            // if (previousStepSourceLevel > 0.0f) {
            //     discharge = previousStepSourceLevel - liquidInSource;
            // }

            // if (previousStepSourceLevel != liquidInSource) {
            //     previousStepSourceLevel = liquidInSource;
            // }

            Bounds targetBounds = target.getSolidObject().GetComponent<Renderer>().bounds;

            Vector3 effectCenter = 0.5f * (robotHandForSource.getRobotHand().transform.TransformPoint(robotHandForSource.getFingerA().transform.localPosition) +
                                           robotHandForSource.getRobotHand().transform.TransformPoint(robotHandForSource.getFingerB().transform.localPosition));

            float distance = Vector3.Distance(source.getSolidObject().transform.position, robotHandForSource.getRobotHand().transform.position);
            bool withinTargetRim = (source.getSolidObject().transform.position.x > targetBounds.min.x &&
                                    source.getSolidObject().transform.position.x < targetBounds.max.x &&
                                    source.getSolidObject().transform.position.z > targetBounds.min.z &&
                                    source.getSolidObject().transform.position.z < targetBounds.max.z);

            if (!isActionDone)
            {
                // If positive angular velocity action and not pouring, pour.
                Vector3 directionOfMovement = (target.getSolidObject().transform.position - source.getSolidObject().transform.position).normalized;
                if (!withinTargetRim)
                {
                    Vector3 newSourceLocation = Vector3.zero;
                    if (source.getSolidObject().transform.position.y - target.getSolidObject().transform.position.y >= 2.5f)
                    {
                        float deltaDistance = robotMovingSpeed > 0.0f ? (robotMovingSpeed * Time.deltaTime) : 0.0f;
                        newSourceLocation = deltaDistance * directionOfMovement;
                    }
                    else
                    {
                        directionOfMovement = (new Vector3(target.getSolidObject().transform.position.x,
                                                          source.getSolidObject().transform.position.y,
                                                          target.getSolidObject().transform.position.z) -
                                              new Vector3(source.getSolidObject().transform.position.x,
                                                          source.getSolidObject().transform.position.y,
                                                          source.getSolidObject().transform.position.z)).normalized;
                        float deltaDistance = robotMovingSpeed > 0.0f ? (robotMovingSpeed * Time.deltaTime) : 0.0f;
                        newSourceLocation = deltaDistance * directionOfMovement;
                    }
                    robotHandForSource.getRobotHand().transform.position += newSourceLocation;
                }

                else
                {
                    float deltaRotation = 0.0f;
                    // float absDischarge = Mathf.Abs(discharge);

                    deltaRotation = -1 * sourceTurningSpeed * Time.deltaTime;

                    // if (absDischarge > 0.0002f)
                    // {
                    //     deltaRotation = sourceTurningSpeed < 0 ? sourceTurningSpeed * Time.deltaTime : 0.0f;
                    float a = robotHandForSource.getRobotHand().transform.rotation.eulerAngles.z;
                    if (a < 0) a = a + 360;

                    // }
                    print(a + " " + sourceTurningSpeed);

                    //if ((sourceTurningSpeed <= 0 && a < 220) )
                    //{
                        robotHandForSource.getRobotHand().transform.RotateAround(effectCenter, Vector3.forward, deltaRotation);
                        sourceTilt += deltaRotation;
                   // }


                }

                // if (weightOfLiquid >= 2.0f * originalWeight) {
                //     // Collision case
                //     SetReward(-1f);
                //     EndEpisode();
                // }
            }


            isActionDone = (weightOfLiquid <= originalWeight) && ((weightOfLiquid > 0.01f && differenceFillLevel < 0.020f) || (weightOfLiquid >= target_to_fill));

            if (isActionDone)
            {
                if (!retract)
                {
                    retractTimer += Time.deltaTime;
                    if (retractTimer < 5.0f)
                    {
                        if (sourceTurningSpeed < 0)
                        {
                            robotHandForSource.getRobotHand().transform.RotateAround(effectCenter, Vector3.forward, sourceTurningSpeed * Time.deltaTime);
                            sourceTilt += sourceTurningSpeed * Time.deltaTime;
                        }
                    }
                    else
                    {
                        retract = true;
                    }
                }
                else
                {
                    print("Episode " + numEpisodes + ": Target: " + target_to_fill + " Filled: " + weightOfLiquid + " Difference: " + differenceFillLevel);
                    numCompletedEpisodes += 1;
                    error += differenceFillLevel;
                    float averagePouringError = ((error / numCompletedEpisodes) * 1000);
                    if (weightOfLiquid > 0.01f && differenceFillLevel <= (toleranceInPouringDeviation + 0.0025f))
                    {
                        AddReward(1.0f);
                        if ((error / numCompletedEpisodes) < 0.02f)
                        {
                            AddReward(1.0f);
                        }
                    }
                    print("Average Pouring Error: " + averagePouringError + " grams");
                    print("Episode " + numEpisodes + ": Final Reward: " + GetCumulativeReward());
                    EndEpisode();
                    if (!isTest)
                    {
                        var statsRecorder = Academy.Instance.StatsRecorder;
                        statsRecorder.Add("Avg. Pouring Error (grams)", averagePouringError);
                    }
                }
            }
        }
    }



    /// <summary>
    /// Configures the agent. Given an integer config, brain will switch from heuristics 
    /// to trained agent.
    /// </summary>
    /// <param name="config">Config.
    /// If -1 : use Heuristics.
    /// If 1:  use trained brain.
    /// </param>
    public void ConfigureAgent(int config)
    {
        configureAgent = config;
        if (configureAgent == -1)
        {
            SetModel("Heuristc", null);
            EndEpisode();
        }
        else if (configureAgent == 1)
        {
            ResetScene();
            //if (liquid.getFlexParticleContainer().viscosity < 10.0f) 
            SetModel("NoPSModel", model);
            //else
            //    SetModel("PouringBrain", highViscosityModel);
        }
    }

    public void setTargetToPour(string target)
    {
        float t = float.Parse(target);
        t = t - calibration_target_offset;
        this.target_to_fill = t / 1000f;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        float rotationAngle = 25f * Input.GetAxis("Horizontal");
        continuousActionsOut[0] = rotationAngle;

        if (Input.GetKey(KeyCode.None))
        {
            continuousActionsOut[0] = 0.0f;
        }
    }
}