//
// Created by coco24 on 2021/7/29.
//

#include "PID_AutoTune_C.h"
#include "PID_AutoTune_v0.h"

PIDAutoTuneHandle PIDAutoTune_New(float *input, float *output) {
  return new PID_ATune(input, output);
}

void PIDAutoTune_Delete(PIDAutoTuneHandle patHandle) {
  delete (PID_ATune*)patHandle;
}

int PIDAutoTune_Runtime(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->Runtime();
}

void PIDAutoTune_Cancel(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->Cancel();
}

void PIDAutoTune_SetOutputStep(PIDAutoTuneHandle patHandle, float step) {
  return ((PID_ATune*)patHandle)->SetOutputStep(step);
}

float PIDAutoTune_GetOutputStep(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->GetOutputStep();
}

void PIDAutoTune_SetControlType(PIDAutoTuneHandle patHandle, int type) {
  return ((PID_ATune*)patHandle)->SetControlType(type);
}

int PIDAutoTune_GetControlType(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->GetControlType();
}


void PIDAutoTune_SetLookbackSec(PIDAutoTuneHandle patHandle, int sec) {
  return ((PID_ATune*)patHandle)->SetLookbackSec(sec);
}

int PIDAutoTune_GetLookbackSec(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->GetLookbackSec();
}


void PIDAutoTune_SetNoiseBand(PIDAutoTuneHandle patHandle, float band) {
  return ((PID_ATune*)patHandle)->SetNoiseBand(band);
}

float PIDAutoTune_GetNoiseBand(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->GetNoiseBand();
}

float PIDAutoTune_GetKp(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->GetKp();
}

float PIDAutoTune_GetKi(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->GetKi();
}

float PIDAutoTune_GetKd(PIDAutoTuneHandle patHandle) {
  return ((PID_ATune*)patHandle)->GetKd();
}
