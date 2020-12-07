using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RRS.Tools.Protobuf
{
 
    public enum AutoCommand
    {
        NONE,
        RESET,
        ADD_PARAMETER,
        REMOVE_PARAMETER
    }
    public enum RobotOperationType
    {
        MANUAL,
        AI
    }

    public enum RobotRestart
    {
        PC,
        NODES,
        BOARD,
        ALL
    }

    public enum DeviceType
    {
        SERVICEROBOT,
        GREETINGROBOT,
        ZONE,
        GAMEPAD,
        REVROBOT,
        PARROTROBOT,
        NOTINIT
    }

    public enum TableState
    {
        FREE,
        USER,
        WAIT,
        EAT,
        CLEAN
    };

    public enum JointState
    {
        NORMAL = 0,
        REACHED_MIN = 1,
        REACHED_MAX = 2
    }

    public enum OrderState
    {
        NEW = 0,
        READY = 1,
        GO_FOR_PICKING = 2,
        PICKEDUP = 4,
        TOWARD_TO_COSTUMER = 8,
        DELIVERED = 16,
        COOKING = 32
    }

    public enum RobotState
    {
        IDLE = 1,
        DAMAGED = 2,
        MOVING = 4,
        WAITING = 8,
        DELIVERING = 16,
        PICKINGUP = 32,
        CHARGING = 64,
        PICKEDUP = 128,
        DELIVERED = 256,
        GO_TO_REST = 512,
        RESTING = 1024
    };

    public enum CollisionType
    {
        BODY = 0,
        TRAY = 1
    };

    public enum DataType
    {
        FLOAT,
        INT,
        UINT,
        BOOL
    }

    public enum NodeType
    {
        MAINBOARD,
        BASE,
        DCMOTOR,
        LED,
        ENV_NODE
    }

    public enum ActuatorType
    {
        POSITION,
        VELOCITY,
        RFID,
        DISPLAY,
        IR,
        LED,
        RELAY,
        SOUND,
        MOSFET,
        ANIMATION
    }

    public enum SensorType
    {
        POSITION,
        VELOCITY,
        TEMPERATURE,
        CURRENT,
        VOLTAGE,
        RFID,
        IR,
        DISPLAY,
        RELAY  
    }

   
    public enum FieldZoneType
    {
        RFID_DISPLAY
    }


    public enum NodeCommandMode
    {
        MANUAL,
        AUTO
    }

}
