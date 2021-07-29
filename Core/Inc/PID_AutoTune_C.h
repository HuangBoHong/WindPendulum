//
// Created by coco24 on 2021/7/29.
//

#ifndef PID_AUTOTUNE_C_H
#define PID_AUTOTUNE_C_H

#ifdef __cplusplus
extern "C" {
#endif
typedef void *PIDAutoTuneHandle;

PIDAutoTuneHandle PIDAutoTune_New(float *input, float *output);

void PIDAutoTune_Delete(PIDAutoTuneHandle patHandle);

int PIDAutoTune_Runtime(PIDAutoTuneHandle patHandle);

void PIDAutoTune_Cancel(PIDAutoTuneHandle patHandle);

void PIDAutoTune_SetOutputStep(PIDAutoTuneHandle patHandle, float step);

float PIDAutoTune_GetOutputStep(PIDAutoTuneHandle patHandle);

void PIDAutoTune_SetControlType(PIDAutoTuneHandle patHandle, int type);

int PIDAutoTune_GetControlType(PIDAutoTuneHandle patHandle);

void PIDAutoTune_SetLookbackSec(PIDAutoTuneHandle patHandle, int sec);

int PIDAutoTune_GetLookbackSec(PIDAutoTuneHandle patHandle);

void PIDAutoTune_SetNoiseBand(PIDAutoTuneHandle patHandle, float band);

float PIDAutoTune_GetNoiseBand(PIDAutoTuneHandle patHandle);

float PIDAutoTune_GetKp(PIDAutoTuneHandle patHandle);

float PIDAutoTune_GetKi(PIDAutoTuneHandle patHandle);

float PIDAutoTune_GetKd(PIDAutoTuneHandle patHandle);


#ifdef __cplusplus
}
#endif

#endif //PID_AUTOTUNE_C_H
