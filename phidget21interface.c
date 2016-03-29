/*
 *  phidget21interface.c
 *
 *  Created by Léa Strobino.
 *  Copyright 2015 hepia. All rights reserved.
 *
 */

#include <mex.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include "phidget21.h"

#define PHIDGET_SAMPLING_PERIOD 8 /* 8 ms = 125 Hz */

typedef struct {
  CPhidgetInterfaceKitHandle handle;
  uint8_t sensorCount;
  uint32_t n;
  uint32_t k;
  int32_t *data;
  pthread_mutex_t mutex;
} PhidgetInterfaceHandle;

inline void nargchk(int nlhs, int required_nlhs, int nrhs, int required_nrhs) {
  if (nlhs < required_nlhs) mexErrMsgTxt("Not enough output arguments.");
  if (nlhs > required_nlhs) mexErrMsgTxt("Too many output arguments.");
  if (nrhs < required_nrhs) mexErrMsgTxt("Not enough input arguments.");
  if (nrhs > required_nrhs) mexErrMsgTxt("Too many input arguments.");
}

int32_t phidget21interface_sensorChangeHandler(CPhidgetInterfaceKitHandle handle, void *p, int32_t index, int32_t sensorValue) {
  
  PhidgetInterfaceHandle *ptr = p;
  
  pthread_mutex_lock(&ptr->mutex);
  
  if (ptr->k < ptr->n) {
    
    /* store data: index+1 and sensor value */
    ptr->data[ptr->k] = index+1;
    ptr->data[ptr->k+1] = sensorValue;
    
    ptr->k += 2;
    
  } else {
    
    CPhidgetInterfaceKit_set_OnSensorChange_Handler(handle,NULL,NULL);
    
  }
  
  pthread_mutex_unlock(&ptr->mutex);
  return 0;
  
}

void closePhidget(PhidgetInterfaceHandle *ptr) {
  CPhidget_close((CPhidgetHandle)ptr->handle);
  CPhidget_delete((CPhidgetHandle)ptr->handle);
  mxFree(ptr->data);
  mxFree(ptr);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  
  /* check function name */
  if ((nrhs < 1) || !mxIsChar(prhs[0])) mexErrMsgTxt("Missing function.");
  char *f = mxArrayToString(prhs[0]);
  
  if (strcmp(f,"open") == 0) {
    
    nargchk(nlhs,1,nrhs,2);
    
    int32_t serialNumber = mxGetScalar(prhs[1]);
    
    /* set the pointer */
    PhidgetInterfaceHandle *ptr = mxMalloc(sizeof(PhidgetInterfaceHandle));
    mexMakeMemoryPersistent(ptr);
    plhs[0] = mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);
    uint64_t *p = mxGetData(plhs[0]);
    *p = (uint64_t)ptr;
    
    /* set default data */
    ptr->n = 1;
    ptr->data = mxCalloc(1,sizeof(int32_t));
    mexMakeMemoryPersistent(ptr->data);
    pthread_mutex_init(&ptr->mutex,NULL);
    
    /* create and open the Phidget */
    CPhidgetInterfaceKit_create(&ptr->handle);
    CPhidget_open((CPhidgetHandle)ptr->handle,serialNumber);
    
    if (CPhidget_waitForAttachment((CPhidgetHandle)ptr->handle,1000)) {
      closePhidget(ptr);
      mexErrMsgIdAndTxt("phidget21interface:NoDeviceAttached","No device attached.");
    }
    
    /* check device class */
    CPhidget_DeviceClass deviceClass;
    CPhidget_getDeviceClass((CPhidgetHandle)ptr->handle,&deviceClass);
    if (deviceClass != PHIDCLASS_INTERFACEKIT) {
      closePhidget(ptr);
      mexErrMsgIdAndTxt("phidget21interface:DeviceClassMismatch","Wrong device class. Expected class is 'PHIDCLASS_INTERFACEKIT'.");
    }
    
    /* set all sensors change trigger */
    CPhidgetInterfaceKit_getSensorCount(ptr->handle,(int32_t*)&ptr->sensorCount);
    uint8_t k;
    for (k=0; k<ptr->sensorCount; k++) {
      CPhidgetInterfaceKit_setSensorChangeTrigger(ptr->handle,k,0);
    }
    
    /* lock the function */
    mexLock();
    
  } else {
    
    /* get the pointer */
    if ((nrhs < 2) || !mxIsUint64(prhs[1])) mexErrMsgTxt("Invalid pointer.");
    uint64_t *p = mxGetData(prhs[1]);
    PhidgetInterfaceHandle *ptr = (PhidgetInterfaceHandle*)*p;
    
    if (strcmp(f,"close") == 0) {
      
      nargchk(nlhs,0,nrhs,2);
      
      /* close the Phidget */
      closePhidget(ptr);
      
    } else if (strcmp(f,"getInfo") == 0) {
      
      nargchk(nlhs,8,nrhs,2);
      
      /* get the device name */
      const char *deviceName;
      CPhidget_getDeviceName((CPhidgetHandle)ptr->handle,&deviceName);
      plhs[0] = mxCreateCharMatrixFromStrings(1,&deviceName);
      
      /* get the serial number */
      plhs[1] = mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);
      uint32_t *serialNumber = mxGetData(plhs[1]);
      CPhidget_getSerialNumber((CPhidgetHandle)ptr->handle,(int32_t*)serialNumber);
      
      /* get the device version */
      plhs[2] = mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);
      uint32_t *deviceVersion = mxGetData(plhs[2]);
      CPhidget_getDeviceVersion((CPhidgetHandle)ptr->handle,(int32_t*)deviceVersion);
      
      /* get the number of sensor inputs */
      plhs[3] = mxCreateNumericMatrix(1,1,mxUINT8_CLASS,mxREAL);
      uint8_t *sensorCount = mxGetData(plhs[3]);
      *sensorCount = ptr->sensorCount;
      
      /* get the number of digital inputs */
      plhs[4] = mxCreateNumericMatrix(1,1,mxUINT8_CLASS,mxREAL);
      uint8_t *inputCount = mxGetData(plhs[4]);
      CPhidgetInterfaceKit_getInputCount(ptr->handle,(int32_t*)inputCount);
      
      /* get the number of digital outputs */
      plhs[5] = mxCreateNumericMatrix(1,1,mxUINT8_CLASS,mxREAL);
      uint8_t *outputCount = mxGetData(plhs[5]);
      CPhidgetInterfaceKit_getOutputCount(ptr->handle,(int32_t*)outputCount);
      
      /* get the minimum supported data rate */
      plhs[6] = mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);
      uint32_t *dataRateMin = mxGetData(plhs[6]);
      CPhidgetInterfaceKit_getDataRateMin(ptr->handle,0,(int32_t*)dataRateMin);
      
      /* get the maximum supported data rate */
      plhs[7] = mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);
      uint32_t *dataRateMax = mxGetData(plhs[7]);
      CPhidgetInterfaceKit_getDataRateMax(ptr->handle,0,(int32_t*)dataRateMax);
      
    } else if (strcmp(f,"setRatiometric") == 0) {
      
      nargchk(nlhs,0,nrhs,3);
      
      int32_t r, ratiometric = mxGetScalar(prhs[2]);
      
      /* set the ratiometric property */
      CPhidgetInterfaceKit_setRatiometric(ptr->handle,ratiometric);
      
      /* wait until the ratiometric property matches */
      do {
        CPhidgetInterfaceKit_getRatiometric(ptr->handle,&r);
      } while (r != ratiometric);
      
    } else if (strcmp(f,"getSensorRawValue") == 0) {
      
      nargchk(nlhs,1,nrhs,3);
      
      uint8_t index = mxGetScalar(prhs[2]);
      
      /* get the specified sensor raw value */
      plhs[0] = mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);
      uint32_t *sensorRawValue = mxGetData(plhs[0]);
      CPhidgetInterfaceKit_getSensorRawValue(ptr->handle,index,(int32_t*)sensorRawValue);
      
    } else if (strcmp(f,"getInputState") == 0) {
      
      nargchk(nlhs,1,nrhs,3);
      
      uint8_t index = mxGetScalar(prhs[2]);
      
      /* get the specified digital input state */
      int32_t inputState;
      CPhidgetInterfaceKit_getInputState(ptr->handle,index,&inputState);
      plhs[0] = mxCreateLogicalScalar(inputState);
      
    } else if (strcmp(f,"setOutputState") == 0) {
      
      nargchk(nlhs,0,nrhs,4);
      
      uint8_t index = mxGetScalar(prhs[2]);
      int32_t outputState = mxGetScalar(prhs[3]);
      
      /* set the specified digital output state */
      CPhidgetInterfaceKit_setOutputState(ptr->handle,index,outputState);
      
    } else if (strcmp(f,"startCapture") == 0) {
      
      nargchk(nlhs,0,nrhs,4);
      
      /* stop the previous thread */
      CPhidgetInterfaceKit_set_OnSensorChange_Handler(ptr->handle,NULL,NULL);
      
      pthread_mutex_lock(&ptr->mutex);
      ptr->n = 2*ptr->sensorCount*mxGetScalar(prhs[2]);
      uint32_t dataRate = mxGetScalar(prhs[3]);
      
      /* allocate memory */
      mxFree(ptr->data);
      ptr->data = mxCalloc(ptr->n,sizeof(int32_t));
      mexMakeMemoryPersistent(ptr->data);
      ptr->k = 0;
      
      /* set all sensors data rate */
      uint8_t k;
      for (k=0; k<ptr->sensorCount; k++) {
        CPhidgetInterfaceKit_setDataRate(ptr->handle,k,dataRate);
      }
      
      /* launch a new thread */
      CPhidgetInterfaceKit_set_OnSensorChange_Handler(ptr->handle,phidget21interface_sensorChangeHandler,ptr);
      
      pthread_mutex_unlock(&ptr->mutex);
      
    } else if (strcmp(f,"getCapturedData") == 0) {
      
      nargchk(nlhs,1,nrhs,3);
      
      uint8_t index = mxGetScalar(prhs[2]);
      
      uint32_t n = ptr->n/(2*ptr->sensorCount), i = 0, k;
      
      /* allocate memory */
      plhs[0] = mxCreateNumericMatrix(n,1,mxUINT32_CLASS,mxREAL);
      uint32_t *sensorValue = mxGetData(plhs[0]);
      
      pthread_mutex_lock(&ptr->mutex);
      
      for (k=0; ((k < ptr->n) && (i < n) && ptr->data[k]); k+=2) {
        
        /* get the specified sensor data */
        if (ptr->data[k] == index+1) {
          
          /* copy selected sensor data into MATLAB workspace */
          sensorValue[i] = ptr->data[k+1];
          i++;
          
        }
        
      }
      
      pthread_mutex_unlock(&ptr->mutex);
      
    } else mexErrMsgTxt("Invalid function.");
    
  }
  
}
