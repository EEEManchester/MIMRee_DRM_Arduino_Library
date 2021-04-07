- [1. LHM](#1-lhm)
  - [1.1. Roadmap](#11-roadmap)
  - [1.2. Mavlink message](#12-mavlink-message)
- [2. OLAM](#2-olam)
  - [2.1. Roadmap](#21-roadmap)


# 1. LHM

## 1.1. Roadmap

| Item                      | State       | To do                                                |
| ------------------------- | ----------- | ---------------------------------------------------- |
| Servo control             | Completed   |                                                      |
| Hook control              | Completed   |                                                      |
| Hinge functions           | Completed   |                                                      |
| Serial port communication | Completed   |                                                      |
| Mavlink communiation      | In progress | - compid is stuck in 1 <br/> - test with RHUL system |
| Status visual aid         | In progress | - add more LED stats                                 |
| Onboard button functions  | Completed   |                                                      |
| Lab test                  | Not started |                                                      |
| Field test                | Not started |                                                      |
|                           |             |                                                      |

## 1.2. Mavlink message

Communication between the LHM and the UAV is enabled by serial using the Mavlink protocol. The Mavlink message used is DEBUG_VECT ([#250](https://mavlink.io/en/messages/common.html#DEBUG_VECT)).

Table 1 summarises the DEBUG_VECT messages that are used by LHMController. Table 2-4 list the parameters used in DEBUG_VECT messages.

*Table 1 DEBUG_VECT messages used by LHMController*
| Message type                           | Description                                                                                               | name  | x                                  | y                                    | z                                        |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------- | ----- | ---------------------------------- | ------------------------------------ | ---------------------------------------- |
| Command in                             | A message indicates the command (an action) that needs to be executed by LHM                              | LHMC  | Command unique code<br>(Table 2)   |                                      |                                          |
| Feedback - Command reception           | A feedback message from LHM indicating a command type message has been received and recognised            | LHMFB | Command unique code<br>(Table 2)   | 1 - successful \*<br>0 - failed \*\* |                                          |
| Feedback - Command sent to be executed | A feedback message from LHM indicating a command has been successfully sent to sub devices to be executed | LHMFB | Command unique code<br>(Table 2)   |                                      | 1 - successful \*<br>0 - failed \*\*     |
| Status                                 | Hinge/hook/payload status report from LHM                                                                 | LHMS  | Hinge status unique code (Table 3) | Hook status unique code (Table 4)    | 1 - payload attached<br>0 - not attached |
|                                        |                                                                                                           |       |                                    |                                      |                                          |

\* It is important that the GCS does not assume the requested commands being fully executed. The feedback only indicates the commands have been passed onto subsystem controllers and the controllers have confirmed reception and recognition of the commands. The commands should only be assumed fully executed when status report shows such. 

** If 0 is received, it means there is a problem somewhere in the chain of execution and the requested commands are not being fully executed.

<a name="table.2"></a>*Table 2 Command unique code*
| Unique code | Command                      | Description                                                      |
| ----------- | ---------------------------- | ---------------------------------------------------------------- |
| 1           | Hinge - torque off           | Turn off servo holding torque                                    |
| 2           | Hinge - take-off mode        | Commands the LHM  to prepare for take off.                       |
| 3           | Hinge - landing mode         | Commands the LHM to prepare for landing.                         |
| 4           | Hinge - swing reduction mode | Commands the LHM to reduce swing during flight.                  |
| 10          | Hook - torque off            | Turn off holding torque for the hook servo                       |
| 11          | Hook - close                 | Lock the hook in closed position                                 |
| 12          | Hook - open                  | Unlock the hook so it opens freely by the weight of the payload. |
| 90          | Reset                        | Reset the controller                                             |
| 98          | Lock LHM                     | Lock LHM in payload bay                                          |
| 99          | Jettison                     | Jettison the LHM in an emergency                                 |
|             |                              |                                                                  |

*Table 3 Hinge status unique code*
| Unique code | Status                     | Description                                    |
| ----------- | -------------------------- | ---------------------------------------------- |
| 0           | Unknown                    | Placeholder currently not reachable.           |
| 1           | Offline                    | At least one of the two servos is not powered. |
| 2           | Error                      | Servo command not fully executed.              |
| 10          | Motor torque off           | Turn off servo torque.                         |
| 20          | Take-off mode              | Hinge is in take-off mode.                     |
| 30          | Landing mode in transition | Hinge is moving into position for landing.     |
| 31          | Landing mode in position   | Hinge is prepared for landing.                 |
| 40          | Swing reduction mode       | Hinge is in swing reduction mode.              |
|             |                            |                                                |

*Table 4 Hook status unique code*
| Unique code | Status                                             | Description                                                                                                                                                                                                      |
| ----------- | -------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0           | Unknown                                            | Placeholder, currently not reachable.                                                                                                                                                                            |
| 1           | Offline                                            | At least one device is not powered (microswitches, PE sensor, hook servo)                                                                                                                                        |
| 2           | Error                                              | Can be trigger by one of the followings:<br>1\. Servo command not fully executed.<br>2\. Combination of the microswitch states are invalid (both microswitches are triggered)<br>3\. PE sensor state is invalid. |
| 10          | Fully closed                                       | Hook is locked in closed state, payload cannot drop out.                                                                                                                                                         |
| 11          | Fully open                                         | Lock released, hook can be opened by slight force or the weight of the payload, used to drop the payload.                                                                                                        |
| 12          | Loose (static, in between the previous two states) | Not fully closed nor fully open and hook motor is also static. This is usually unexpected.                                                                                                                       |
| 20          | In transition - Closing                            | Hook is loose but is being closed and locked.                                                                                                                                                                    |
| 21          | In transition - Opening                            | Hook is loose but is being opened.                                                                                                                                                                               |
|             |                                                    |                                                                                                                                                                                                                  |


# 2. OLAM
## 2.1. Roadmap

| Item                             | State       | To do |
| -------------------------------- | ----------- | ----- |
| Servo control                    | Completed   |
| Tension control                  | Completed   |       |
| More intelligent tension control | Completed   |       |
| Serial port communication        | Completed   |       |
| Mavlink communiation             | Not started |       |
| Status visual aid                | Not started |       |
| Onboard button functions         | Completed   |       |
| Lab test                         | Not started |       |
| Field test                       | Not started |       |
|                                  |             |       |
 