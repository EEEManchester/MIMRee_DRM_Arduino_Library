#ifndef MIMREE_DRM_CONTROLLER_MOTION_SEQUENCE_H
#define MIMREE_DRM_CONTROLLER_MOTION_SEQUENCE_H

#include <Arduino.h>

#include "DxlMotor.h"
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
    int8_t stageId() { return _stageId; }
    uint8_t motorId() { return motor->getId(); }
    int32_t goalPosition() { return _goalPosition; }
    int32_t accuracy() { return _accuracy; }
    bool started() { return _started; }

    void update(int8_t stageId, DXLMotor *motor, int32_t goalPosition, int32_t accuracy);

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
    int8_t _stageId = -1;
    DXLMotor *motor = nullptr;
    int32_t _goalPosition = -1;
    int32_t _accuracy = -1;
};

class MotionSequence
{
public:
    MotionSequence(){};
    MotionSequence(MotionSequenceType type, DXLMotor *motors[], const uint16_t sequence[]);
    MotionSequenceType sequenceType() { return _sequenceType; }
    int8_t currentStageId() { return currentStage.stageId(); }
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
    const uint16_t *sequence;
    Stage currentStage;
};
#endif