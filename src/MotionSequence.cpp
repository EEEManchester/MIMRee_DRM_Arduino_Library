#include "MotionSequence.h"
using namespace ControlTableItem;

MotionSequence::MotionSequence(MotionSequenceType sequenceType, DXLMotor *_motors[], const uint16_t sequence[])
    : _sequenceType(sequenceType),
      motors(_motors),
      sequence(sequence)
{
    currentStage = Stage();
}

MotionSequenceStatusType MotionSequence::status()
{
    printDebugInfo("MotionSequence::status");
    currentStage.printDebugInfo("MotionSequence::status");
    if (_sequenceType == MS_SEQ_TYPE_UNKNOWN)
    {
        return MS_SEQ_STATUS_UNKNOWN;
    }
    uint8_t result = currentStage.status();
    DXL_DEBUG_PRINTF("MotionSequence::status: currentStage.status() returns %d\n", result);
    if (!currentStage.started())
    {
        return MS_SEQ_STATUS_WAITING;
    }
    if (result == MS_STAGE_STATUS_COMPLETED)
    {
        if (currentStage.stageId() >= sequence[0] - 1)
        {
            return MS_SEQ_STATUS_COMPLETED;
        }
        return MS_SEQ_STATUS_STAGE_COMPLETED;
    }
    else if (result == MS_STAGE_STATUS_NOT_COMPLETED)
    {
        return MS_SEQ_STATUS_BUSY;
    }
    else
    {
        return MS_SEQ_STATUS_ERROR;
    }
}

MotionSequenceExecusionResultType MotionSequence::next()
{
    printDebugInfo("MotionSequence::next");
    int8_t nextStageId = currentStage.stageId() + 1;
    DXL_DEBUG_PRINTF("MotionSequence::next: Sequence type: %d, Stage ID: %d/%d\n", (int)_sequenceType, nextStageId, sequence[0] - 1);
    uint8_t motorId = sequence[nextStageId * 3 + 1] - 1;
    int32_t goalPos = sequence[nextStageId * 3 + 2];
    int32_t accuracy = sequence[nextStageId * 3 + 3];
    DXL_DEBUG_PRINTF("MotionSequence::next: Motor[%d(%d)] -> %d (+/-%d)\n", motors[motorId]->getId(), motorId + 1, goalPos, accuracy);
    delay(1000);
    currentStage.update(nextStageId, motors[motorId], goalPos, accuracy);
    int8_t result = currentStage.execute();
    DXL_DEBUG_PRINTF("MotionSequence::next: Result=%d\n", result);
    return result ? MS_EXE_RE_SUCCESSFUL : MS_EXE_RE_FAILED;
}

void MotionSequence::printDebugInfo(String scopeName)
{
    DXL_DEBUG_PRINTF("%s Sequence[%d] -> motor[%d,%d,%d] | seq[%d,%d,%d,%d]\n",
                     scopeName.c_str(),
                     (int8_t)sequenceType(),
                     motors[0]->getId(),
                     motors[1]->getId(),
                     motors[2]->getId(),
                     sequence[0],
                     sequence[1],
                     sequence[2],
                     sequence[3]);
}

void Stage::update(int8_t stageId, DXLMotor *_motor, int32_t goalPosition, int32_t accuracy)
{
    _stageId = stageId;
    motor = _motor;
    _goalPosition = goalPosition;
    _accuracy = accuracy;
    _started = false;
    DXL_DEBUG_PRINTF("Stage::update: [%d]: Motor[%d] -> %d (+/-%d)\n", stageId, motor->getId(), goalPosition, accuracy);
}

MotionSequenceStageStatusType Stage::status()
{
    if (!started())
    {
        return MS_STAGE_STATUS_NOT_STARTED;
    }
    if (abs(_goalPosition - motor->getLastSetGoalPosition()) > 0.1)
    {
        return MS_STAGE_STATUS_ERROR;
    }
    if (motor->isAtGoalPosition(_accuracy))
    {
        return MS_STAGE_STATUS_COMPLETED;
    }
    return MS_STAGE_STATUS_NOT_COMPLETED;
}

bool Stage::execute()
{
    if (status() != MS_STAGE_STATUS_NOT_STARTED)
    {
        return false;
    }
    bool result = true;
    if (motor->getLastSetOperatingMode() != OP_POSITION)
    {
        result = result && motor->setOperatingMode(OP_POSITION);
    }
    result = result && motor->setVelocityProfile(PROFILE_VELOCITY_VAL);
    result = result && motor->setAccelerationProfile(PROFILE_ACCELERATION_VAL);
    result = result && motor->setTorqueOn();
    result = result && motor->setGoalPosition((float)_goalPosition);
    if (result)
    {
        _started = true;
    }
    return result;
}

void Stage::printDebugInfo(String scopeName)
{
    DXL_DEBUG_PRINTF("%s CurrentStage[%d] -> motor[%d] | started[%d] | goal[%d] | accuracy[%d]\n",
                     scopeName.c_str(),
                     stageId(),
                     motorId(),
                     started(),
                     goalPosition(),
                     accuracy());
}