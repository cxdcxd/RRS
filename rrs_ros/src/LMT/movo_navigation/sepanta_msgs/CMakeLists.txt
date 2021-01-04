cmake_minimum_required(VERSION 2.8.3)
project(sepanta_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs genmsg actionlib_msgs geometry_msgs sensor_msgs nav_msgs message_generation)

add_message_files(
    FILES 


    arm.msg 
    coordinate.msg
    head.msg 
    irsensor.msg
    joint.msg 
    motortorques.msg 
    omnidata.msg 
    user.msg 
    users.msg
    upperbodymotors.msg
    upperbodymotorsfeedback.msg
    motor.msg
    motorfeedback.msg
    GestureDetect.msg
    facepos.msg
    faceposition.msg
    nearestfacepos.msg
    imudata.msg
    forcedata.msg
    Object.msg
    Objects.msg
    log.msg
    led.msg
    wave_hand.msg
    object_fetch.msg
    nameobjects.msg
    ramp_pos.msg
    patterns_name.msg
    ack_packages.msg
    log_Packages.msg
    lbm.msg
    NodeEvent.msg
    nodestatus.msg
    nodestatuslist.msg
    modelsName.msg
    voltagedata.msg
    Person.msg
    PersonArray.msg
    Leg.msg
    LegArray.msg
    Activities.msg
    Appearance.msg
    Audio.msg
    Body.msg
    BodyArray.msg
    Expressions.msg
    JointOrientationAndType.msg
    JointPositionAndState.msg
    Lean.msg
)

add_service_files(
    FILES 

    sound.srv
    command.srv 
    maptools.srv 
    stop.srv
    windows.srv
    follow.srv
    motorreset.srv
    skeletonSrv.srv
    motortorque.srv
    recordSrv.srv
    FaceDetectionSrv.srv
    FaceTrackingSrv.srv
    set_iterationSrv.srv
    set_FaceDetectionSrv.srv
    face_headtrackerSrv.srv
    CreateModel_continueSrv.srv
    CreateModelNameSrv.srv
    CreateModelSrv.srv
    waveSrv.srv
    motorpid.srv
    ObjectFetchSrv.srv
    setthreshSrv.srv
    run_objectrecognitionSrv.srv
    down.srv
    faceobstacleSrv.srv
    faceobstacleconfigSrv.srv
    facealighnSrv.srv
    rampSrv.srv
    listnamemodels_Srv.srv
    RampPatternsID.srv
    kill_marker.srv
    ListAvailable.srv
    ListLoaded.srv
    NodeAction.srv
    managercommand.srv
    speechkill.srv
    stopsrv.srv
    grip.srv

)

add_action_files(
    FILES 

    LookForObjects.action
    avoidThat.action
    avoidthataction.action
    grip.action
    grip_emergency.action
    grip_party.action
    logic_grip.action
    object.action
    object_emergency.action
    gesture.action
    object_party.action
    pickAndPlace.action
    sepanta.action
    slamaction.action
    whatDidYouSay.action
    grip.action
    Fibonacci.action
    Master.action

)

generate_messages(
    DEPENDENCIES 
    std_msgs 
    actionlib_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package()
