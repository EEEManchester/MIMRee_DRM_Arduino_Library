#ifndef MIMREE_DRM_CONTROLLER_MOTION_SEQUENCE_H
#define MIMREE_DRM_CONTROLLER_MOTION_SEQUENCE_H

#include <Arduino.h>

#include "DxlMotor.h"
#include "LHMDataTable.h"

enum MotionSequenceType: uint8_t
{
    MS_SEQ_TYPE_UNKNOWN = 0,
    MS_SEQ_TYPE_LANDING
};

enum MotionSequenceStatusType: uint8_t
{
    MS_SEQ_STATUS_UNKNOWN = 0,
    MS_SEQ_STATUS_ERROR,
    MS_SEQ_STATUS_WAITING,
    MS_SEQ_STATUS_BUSY,
    MS_SEQ_STATUS_STAGE_COMPLETED,
    MS_SEQ_STATUS_COMPLETED
};

enum MotionSequenceStageStatusType: uint8_t
{
    MS_STAGE_STATUS_ERROR = 0,
    MS_STAGE_STATUS_NOT_STARTED,
    MS_STAGE_STATUS_NOT_COMPLETED,
    MS_STAGE_STATUS_COMPLETED
};

enum MotionSequenceExecusionResultType: uint8_t
{
    MS_EXE_RE_ERROR = 0,
    MS_EXE_RE_FAILED,
    MS_EXE_RE_SUCCESSFUL,
    MS_EXE_RE_SEQUENCE_ENDED
};

class Stage
{
public:
    inline int8_t stageId() { return _stageId; }
    inline uint8_t motorId() { return motor->getId(); }
    inline int32_t goalPosition() { return _goalPosition; }
    inline int32_t accuracy() { return _accuracy; }
    inline bool started() { return _started; }
    inline void reset()
    {
        _stageId = -1;
        motor = nullptr;
        _goalPosition = -1;
        _accuracy = -1;
    }

    void update(int8_t stageId, DXLMotor *motor, int32_t goalPosition, int32_t accuracy);
    MotionSequenceStageStatusType status();
    bool execute();
    void printDebugInfo(String scopeName = "");

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
    inline MotionSequence(){};
    MotionSequence(MotionSequenceType type, DXLMotor *motors[], const uint16_t sequence[]);

    inline MotionSequenceType sequenceType() { return _sequenceType; }
    inline int8_t currentStageId() { return currentStage.stageId(); }
    inline void reset()
    {
        currentStage.reset();
        _sequenceType = MS_SEQ_TYPE_UNKNOWN;
    }

    MotionSequenceStatusType status();
    MotionSequenceExecusionResultType next();
    void printDebugInfo(String scopeName = "");
private:
    MotionSequenceType _sequenceType = MS_SEQ_TYPE_UNKNOWN;
    DXLMotor **motors;
    const uint16_t *sequence;
    Stage currentStage;
};
#endif