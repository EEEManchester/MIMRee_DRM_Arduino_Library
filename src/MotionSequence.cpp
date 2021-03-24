#include "MotionSequence.h"
using namespace ControlTableItem;

MotionSequence::MotionSequence(MotionSequenceType sequenceType, DXLMotor motors[], const int sequence[]) : _sequenceType(sequenceType), motors(motors), sequence(sequence)
{
    stageCount = sizeof(sequence) / sizeof(int) / 3;
}


MotionSequenceStatusType MotionSequence::status() {
    if (currentStage.stageId() == stageCount - 1)
    {return MotionSequenceStatusType::COMPLETED;}
    int8_t result  = currentStage.completed();
    if (result == 1)
    {
        return MotionSequenceStatusType::STAGE_COMPLETED;
    }
    else if (result == 0)
    {
        return MotionSequenceStatusType::BUSY;
    }
    else
    {
        return MotionSequenceStatusType::ERROR;
    }
}

int8_t MotionSequence::next()
{
    int nextStageId = currentStage.stageId() + 1;
    if (nextStageId > stageCount - 1)
    {
        return 2;
    }

    currentStage.update(nextStageId, &motors[nextStageId], sequence[nextStageId * 3 + 1], sequence[nextStageId * 3 + 2]);
    return currentStage.execute();
}

void Stage::update(int stageId, DXLMotor *motor, int goalPosition, int accuracy)
{
    _stageId = stageId;
    Stage::motor = motor;
    _goalPosition = goalPosition;
    _accuracy = accuracy;
}

int8_t Stage::completed()
{
    if (abs(_goalPosition - motor->getLastSetGoalPosition()) > 0.1)
    {
        return -1;
    }

    return motor->isAtGoalPosition(_accuracy);
}

int8_t Stage::busy()
{
    if (!started)
    {
        return 0;
    }

    return completed();
}

int8_t Stage::execute()
{
    if (busy() != 0)
    {
        return -1;
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
    return result;
}