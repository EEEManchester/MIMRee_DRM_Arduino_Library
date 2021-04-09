- [1. LHM](#1-lhm)
  - [1.1. Roadmap](#11-roadmap)
  - [1.2. Mavlink message](#12-mavlink-message)
    - [1.2.1. Send commands to LHM](#121-send-commands-to-lhm)
    - [1.2.2. LHM command feedback](#122-lhm-command-feedback)
    - [1.2.3. LHM status report](#123-lhm-status-report)
    - [1.2.4. LHM Command and status](#124-lhm-command-and-status)
- [2. OLAM](#2-olam)
  - [2.1. Roadmap](#21-roadmap)


# 1. LHM

## 1.1. Roadmap

| Item                        | State       | To do                   |
| --------------------------- | ----------- | ----------------------- |
| Servo control               | Completed   |                         |
| Hook control                | Completed   |                         |
| Hinge functions             | Completed   |                         |
| Serial port communication   | Completed   |                         |
| Mavlink communiation        | In progress | - test with RHUL system |
| Status visual aid           | In progress | - add more LED stats    |
| Onboard button functions    | Completed   |                         |
| Lab test                    | Not started |                         |
| Field test                  | Not started |                         |
| LHM mission finish feedback | Not started |                         |

## 1.2. Mavlink message

### 1.2.1. Send commands to LHM

Mavlink protocol is used to handle communication between the LHM and the GCS. The *Component ID* of LHM is *MAV_COMP_ID_USER31 = 55*.

To request LHM to perform certain task, GCS needs to send a COMMAND_INT ([#75](https://mavlink.io/en/messages/common.html#COMMAND_INT)) message.


[Table 1](#table.1)
| Field Name       | Type     | Values                    | Description                                              |
| ---------------- | -------- | ------------------------- | -------------------------------------------------------- |
| target_system    | uint8_t  | 1                         | System ID of LHM                                         |
| target_component | uint8_t  | 55                        | Component ID of LHM                                      |
| frame            | uint8_t  | 0                         | The coordinate system of the COMMAND. (Not used for LHM) |
| command          | uint16_t | MAV_CMD_DO_SET_MODE (176) | Use MAV_CMD_DO_SET_MODE only.                            |
| current          | uint8_t  | 0                         | Not used.                                                |
| autocontinue     | uint8_t  | 0                         | Not used (set 0).                                        |
| param1           | float    | [LHM Command](#table.1)   | The command to be sent.                                  |
| param2           | float    | 0                         | Not used by LHM.                                         |
| param3           | float    | 0                         | Not used by LHM.                                         |
| param4           | float    | 0                         | Not used by LHM.                                         |
| x                | int32_t  | 0                         | Not used by LHM.                                         |
| y                | int32_t  | 0                         | Not used by LHM.                                         |
| z                | float    | 0                         | Not used by LHM.                                         |

### 1.2.2. LHM command feedback

LHM sends command feedback on two occassions: 
1. Upon reception of COMMAND_INT
2. Once LHM system starts to execute the command.

Command feedback is reported using COMMAND_ACK ([#77](https://mavlink.io/en/messages/common.html#COMMAND_ACK)).

| Field Name          | Type     | Values                  | Description                                                                                                                                |
| ------------------- | -------- | ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| command             | uint16_t | [LHM Command](#table.1) | The command to be acknowledged.                                                                                                            |
| result              | uint8_t  | 0, 1                    | Result of command, 0 is failed and 1 is successful.                                                                                        |
| progress **         | uint8_t  | 0, 1, 2, 3              | WIP: 0: Error (not implemented), 1: command_int received and interpreted, 2: mission started, 3: mission finished (not implemented)        |
| result_param2 **    | int32_t  |                         | Not used.                                                                                                                                  |
| target_system **    | uint8_t  |                         | System ID of the target recipient. This is the ID of the system that sent the command for which this COMMAND_ACK is an acknowledgement.    |
| target_component ** | uint8_t  |                         | Component ID of the target recipient. This is the ID of the system that sent the command for which this COMMAND_ACK is an acknowledgement. |

### 1.2.3. LHM status report

LHM reports its status using DEBUG_VECT ([#250](https://mavlink.io/en/messages/common.html#DEBUG_VECT)) at a rate of 1Hz (can be configured).

| Field Name | Type     | Values                       | Description                                                                                                                                                                                       |
| ---------- | -------- | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| name       | char[10] | "LHMS"                       | Name - used to identify different DEBUG_VECT.                                                                                                                                                     |
| time_usec  | uint64_t |                              | Timestamp (unit: us) (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. |
| x          | float    | [LHM Hinge status](#table.2) | Status of the hinge.                                                                                                                                                                              |
| y          | float    | [LHM Hook status](#table.2)  | Status of the hook.                                                                                                                                                                               |
| z          | float    | 0, 1                         | Payload engagement. 0 - not engaged, 1 - engaged.                                                                                                                                                 |

### 1.2.4. LHM Command and status

<a name="table.1"></a>*Table 1 LHM Command*
| Value | Command                      | Description                                                      |
| ----- | ---------------------------- | ---------------------------------------------------------------- |
| 1     | Hinge - torque off           | Turn off servo holding torque                                    |
| 2     | Hinge - take-off mode        | Commands the LHM  to prepare for take off.                       |
| 3     | Hinge - landing mode         | Commands the LHM to prepare for landing.                         |
| 4     | Hinge - swing reduction mode | Commands the LHM to reduce swing during flight.                  |
| 10    | Hook - torque off            | Turn off holding torque for the hook servo                       |
| 11    | Hook - close                 | Lock the hook in closed position                                 |
| 12    | Hook - open                  | Unlock the hook so it opens freely by the weight of the payload. |
| 90    | Reset                        | Reset the controller                                             |
| 98    | Lock LHM                     | Lock LHM in payload bay                                          |
| 99    | Jettison                     | Jettison the LHM in an emergency                                 |

<a name="table.2"></a>*Table 2 LHM Hinge status*
| Value | Status                     | Description                                    |
| ----- | -------------------------- | ---------------------------------------------- |
| 0     | Unknown                    | Placeholder currently not reachable.           |
| 1     | Offline                    | At least one of the two servos is not powered. |
| 2     | Error                      | Servo command not fully executed.              |
| 10    | Motor torque off           | Turn off servo torque.                         |
| 20    | Take-off mode              | Hinge is in take-off mode.                     |
| 30    | Landing mode in transition | Hinge is moving into position for landing.     |
| 31    | Landing mode in position   | Hinge is prepared for landing.                 |
| 40    | Swing reduction mode       | Hinge is in swing reduction mode.              |

<a name="table.3"></a>*Table 3 LHM Hook status*
| Value | Status                                             | Description                                                                                                                                                                                                      |
| ----- | -------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | Unknown                                            | Placeholder, currently not reachable.                                                                                                                                                                            |
| 1     | Offline                                            | At least one device is not powered (microswitches, PE sensor, hook servo)                                                                                                                                        |
| 2     | Error                                              | Can be trigger by one of the followings:<br>1\. Servo command not fully executed.<br>2\. Combination of the microswitch states are invalid (both microswitches are triggered)<br>3\. PE sensor state is invalid. |
| 10    | Fully closed                                       | Hook is locked in closed state, payload cannot drop out.                                                                                                                                                         |
| 11    | Fully open                                         | Lock released, hook can be opened by slight force or the weight of the payload, used to drop the payload.                                                                                                        |
| 12    | Loose (static, in between the previous two states) | Not fully closed nor fully open and hook motor is also static. This is usually unexpected.                                                                                                                       |
| 20    | In transition - Closing                            | Hook is loose but is being closed and locked.                                                                                                                                                                    |
| 21    | In transition - Opening                            | Hook is loose but is being opened.                                                                                                                                                                               |


# 2. OLAM
## 2.1. Roadmap

| Item                             | State       | To do |
| -------------------------------- | ----------- | ----- |
| Servo control                    | Completed   |       |
| Tension control                  | Completed   |       |
| More intelligent tension control | Completed   |       |
| Serial port communication        | Completed   |       |
| Mavlink communiation             | Not started |       |
| Status visual aid                | Not started |       |
| Onboard button functions         | Completed   |       |
| Lab test                         | Not started |       |
| Field test                       | Not started |       |
 