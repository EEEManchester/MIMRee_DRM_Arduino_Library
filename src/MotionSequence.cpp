#include "MotionSequence.h"
using namespace ControlTableItem;

MotionSequence::MotionSequence(MotionSequenceType sequenceType, DXLMotor *_motors[], const int sequence[])
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
    if (currentStage.stageId() == sequence[0] - 1)
    {
        return MotionSequenceStatusType::COMPLETED;
    }
    if (!currentStage.started())
    {
        return MotionSequenceStatusType::WAITING;
    }
    int8_t result = currentStage.completed();
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
    printDebugInfo("MotionSequence::next");
    int nextStageId = currentStage.stageId() + 1;
    Serial.printf("MotionSequence::next: Sequence type: %d, Stage ID: %d/%d\n", (int)_sequenceType, nextStageId, sequence[0] - 1);
    if (nextStageId > sequence[0] - 1)
    {
        return 2;
    }
    int motorId = sequence[nextStageId * 3 + 1]-1;
    int goalPos = sequence[nextStageId * 3 + 2];
    int accuracy = sequence[nextStageId * 3 + 3];
    Serial.printf("MotionSequence::next: Motor[%d(%d)] -> %d (+/-%d)\n", motors[motorId]->getId(), motorId+1, goalPos, accuracy);
    delay(1000);
    currentStage.update(nextStageId, motors[motorId], goalPos, accuracy);
    // currentStage.printDebugInfo("MotionSequence::next");
    int8_t result = currentStage.execute();
    Serial.printf("MotionSequence::next: Result = %d\n", result);
    currentStage.printDebugInfo("MotionSequence::next");
    status();
    currentStage.printDebugInfo("MotionSequence::next");
    
    printDebugInfo("MotionSequence::next_4");
    return result;
}

void MotionSequence::printDebugInfo(String scopeName)
{
    Serial.printf("[DEBUG | MS] %s Sequence[%d] -> motor[%d,%d,%d] | seq[%d,%d,%d,%d]\n",
    scopeName.c_str(),
    (int)sequenceType(),
    motors[0]->getId(),
    motors[1]->getId(),
    motors[2]->getId(),
    sequence[0],
    sequence[1],
    sequence[2],
    sequence[3]);
}

void Stage::update(int stageId, DXLMotor *_motor, int goalPosition, int accuracy)
{
    _stageId = stageId;
    motor = _motor;
    _goalPosition = goalPosition;
    _accuracy = accuracy;
    _started = false;
    Serial.printf("Stage::update: [%d]: Motor[%d] -> %d (+/-%d)\n", stageId, motor->getId(), goalPosition, accuracy);
    // printDebugInfo("Stage::update");
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
    if (!started())
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
    if (result)
    {
        _started = true;
    }
    return result;
}

void Stage::printDebugInfo(String scopeName)
{
    Serial.printf("[DEBUG | MS-S] %s CurrentStage[%d] -> motor[%d] | started[%d] | goal[%d] | accuracy[%d]\n",
    scopeName.c_str(),
    stageId(),
    motorId(),
    started(),
    goalPosition(),
    accuracy());
}