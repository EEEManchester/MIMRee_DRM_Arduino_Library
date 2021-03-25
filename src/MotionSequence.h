#ifndef MOTION_SEQUENCE_H
#define MOTION_SEQUENCE_H

#include "DxlMotor.h"
#include <Arduino.h>
#include "LHMDataTable.h"



enum class MotionSequenceType
{
    UNKNOWN = -1,
    LANDING
};

enum class MotionSequenceStatusType
{
    UNKNOWN = -2,
    ERROR,
    WAITING = 0,
    BUSY,
    STAGE_COMPLETED,
    COMPLETED
};

class Stage
{
public:
    int stageId() { return _stageId; }
    int motorId() { return motor->getId(); }
    int goalPosition() { return _goalPosition; }
    int accuracy() { return _accuracy; }
    bool started() { return _started; }

    void update(int stageId, DXLMotor *motor, int goalPosition, int accuracy);

    /**
     * @return -1: error (position goal is not the same as stage goal)
     *          0: stage not completed
     *          1: stage completed
     */
    int8_t completed();

    /**
     * @return -1: error
     *          0: not busy
     *          1: busy
     */
    int8_t busy();

    /**
     * @return -1: error
     *          0: execution message not fully sent
     *          1: execution message fully sent
     */
    int8_t execute();

    void printDebugInfo(String scopeName="");

private:
    bool _started;
    int _stageId = -1;
    DXLMotor *motor = nullptr;
    int _goalPosition = -1;
    int _accuracy = -1;
};

class MotionSequence
{
public:
    MotionSequence(){};
    MotionSequence(MotionSequenceType type, DXLMotor *motors[], const int sequence[]);
    MotionSequenceType sequenceType() { return _sequenceType; }
    int currentStageId() { return currentStage.stageId(); }
    MotionSequenceStatusType status();

    /**
     * @return -1: error
     *          0: failed
     *          1: successful 
     *          2: sequence reaches end
     */
    int8_t next();
    void printDebugInfo(String scopeName="");

private:
    MotionSequenceType _sequenceType = MotionSequenceType::UNKNOWN;
    DXLMotor **motors;
    const int *sequence;
    Stage currentStage;
};
#endif