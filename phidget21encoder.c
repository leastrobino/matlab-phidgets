/*
 *  phidget21encoder.c
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

#ifdef __APPLE__
#include <mach/mach_time.h>
static mach_timebase_info_data_t timebase;
#else
#include <time.h>
#endif

#define PHIDGET_SAMPLING_PERIOD 8 /* 8 ms = 125 Hz */

typedef struct {
  CPhidgetEncoderHandle handle;
  CPhidget_DeviceID id;
  uint8_t encoderCount;
  uint32_t n;
  uint32_t k;
#ifdef __APPLE__
  uint64_t t0;
#else
  struct timespec t0;
#endif
  int32_t *data;
  uint8_t *firstSample;
  pthread_mutex_t mutex;
} PhidgetEncoderHandle;

inline void nargchk(int nlhs, int required_nlhs, int nrhs, int required_nrhs) {
  if (nlhs < required_nlhs) mexErrMsgTxt("Not enough output arguments.");
  if (nlhs > required_nlhs) mexErrMsgTxt("Too many output arguments.");
  if (nrhs < required_nrhs) mexErrMsgTxt("Not enough input arguments.");
  if (nrhs > required_nrhs) mexErrMsgTxt("Too many input arguments.");
}

int32_t phidget21encoder_positionChangeHandler(CPhidgetEncoderHandle handle, void *p, int32_t index, int32_t relativeTime, int32_t relativePosition) {
  
  PhidgetEncoderHandle *ptr = p;
  
  pthread_mutex_lock(&ptr->mutex);
  
  if (ptr->k < ptr->n) {
    
    /* store data: index+1, relative time (ms or µs) and relative position */
    ptr->data[ptr->k] = index+1;
    if (ptr->id == PHIDID_ENCODER_HS_4ENCODER_4INPUT) ptr->data[ptr->k+1] = relativeTime;
    else ptr->data[ptr->k+1] = 1024*relativeTime;
    ptr->data[ptr->k+2] = relativePosition;
    
    /* if this is the first sample, get the absolute time */
    if (ptr->firstSample[index]) {
#ifdef __APPLE__
      uint64_t t1 = mach_absolute_time();
      ptr->data[ptr->k+1] = round((double)(t1-ptr->t0)*(double)timebase.numer/((double)timebase.denom*1000));
#else
      struct timespec t1;
      clock_gettime(CLOCK_REALTIME,&t1);
      ptr->data[ptr->k+1] = round((double)(t1.tv_sec-ptr->t0.tv_sec)*1000000+(double)(t1.tv_nsec-ptr->t0.tv_nsec)/1000);
#endif
      ptr->firstSample[index] = 0;
    }
    
    ptr->k += 3;
    
  } else {
    
    CPhidgetEncoder_set_OnPositionChange_Handler(handle,NULL,NULL);
    
  }
  
  pthread_mutex_unlock(&ptr->mutex);
  return 0;
  
}

void closePhidget(PhidgetEncoderHandle *ptr) {
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
    PhidgetEncoderHandle *ptr = mxMalloc(sizeof(PhidgetEncoderHandle));
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
    CPhidgetEncoder_create(&ptr->handle);
    CPhidget_open((CPhidgetHandle)ptr->handle,serialNumber);
    
    if (CPhidget_waitForAttachment((CPhidgetHandle)ptr->handle,1000)) {
      closePhidget(ptr);
      mexErrMsgIdAndTxt("phidget21encoder:NoDeviceAttached","No device attached.");
    }
    
    /* check device class and ID */
    CPhidget_DeviceClass deviceClass;
    CPhidget_getDeviceClass((CPhidgetHandle)ptr->handle,&deviceClass);
    if (deviceClass != PHIDCLASS_ENCODER) {
      closePhidget(ptr);
      mexErrMsgIdAndTxt("phidget21encoder:DeviceClassMismatch","Wrong device class. Expected class is 'PHIDCLASS_ENCODER'.");
    }
    CPhidget_getDeviceID((CPhidgetHandle)ptr->handle,&ptr->id);
    
    /* enable all encoder inputs */
    CPhidgetEncoder_getEncoderCount(ptr->handle,(int32_t*)&ptr->encoderCount);
    uint8_t k;
    for (k=0; k<ptr->encoderCount; k++) {
      CPhidgetEncoder_setEnabled(ptr->handle,k,1);
    }
    ptr->firstSample = mxCalloc(ptr->encoderCount,sizeof(uint8_t));
    mexMakeMemoryPersistent(ptr->firstSample);
    
    /* lock the function */
    mexLock();
    
  } else {
    
    /* get the pointer */
    if ((nrhs < 2) || !mxIsUint64(prhs[1])) mexErrMsgTxt("Invalid pointer.");
    uint64_t *p = mxGetData(prhs[1]);
    PhidgetEncoderHandle *ptr = (PhidgetEncoderHandle*)*p;
    
    if (strcmp(f,"close") == 0) {
      
      nargchk(nlhs,0,nrhs,2);
      
      /* close the Phidget */
      mxFree(ptr->firstSample);
      closePhidget(ptr);
      
    } else if (strcmp(f,"getInfo") == 0) {
      
      nargchk(nlhs,5,nrhs,2);
      
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
      
      /* get the number of encoder inputs */
      plhs[3] = mxCreateNumericMatrix(1,1,mxUINT8_CLASS,mxREAL);
      uint8_t *encoderCount = mxGetData(plhs[3]);
      *encoderCount = ptr->encoderCount;
      
      /* get the number of digital inputs */
      plhs[4] = mxCreateNumericMatrix(1,1,mxUINT8_CLASS,mxREAL);
      uint8_t *inputCount = mxGetData(plhs[4]);
      CPhidgetEncoder_getInputCount(ptr->handle,(int32_t*)inputCount);
      
    } else if (strcmp(f,"setPosition") == 0) {
      
      nargchk(nlhs,0,nrhs,4);
      
      uint8_t index = mxGetScalar(prhs[2]);
      int32_t position = mxGetScalar(prhs[3]);
      
      /* set the specified encoder position */
      CPhidgetEncoder_setPosition(ptr->handle,index,position);
      
    } else if (strcmp(f,"getPosition") == 0) {
      
      nargchk(nlhs,1,nrhs,3);
      
      uint8_t index = mxGetScalar(prhs[2]);
      
      /* get the specified encoder position */
      plhs[0] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
      int32_t *position = mxGetData(plhs[0]);
      CPhidgetEncoder_getPosition(ptr->handle,index,position);
      
    } else if (strcmp(f,"getIndexPosition") == 0) {
      
      nargchk(nlhs,1,nrhs,3);
      
      uint8_t index = mxGetScalar(prhs[2]);
      
      /* get the specified encoder index position */
      plhs[0] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
      int32_t *position = mxGetData(plhs[0]);
      CPhidgetEncoder_getIndexPosition(ptr->handle,index,position);
      
    } else if (strcmp(f,"getInputState") == 0) {
      
      nargchk(nlhs,1,nrhs,3);
      
      uint8_t index = mxGetScalar(prhs[2]);
      
      /* get the specified digital input state */
      int32_t inputState;
      CPhidgetEncoder_getInputState(ptr->handle,index,&inputState);
      plhs[0] = mxCreateLogicalScalar(inputState);
      
    } else if (strcmp(f,"startCapture") == 0) {
      
      nargchk(nlhs,0,nrhs,3);
      
      /* stop the previous thread */
      CPhidgetEncoder_set_OnPositionChange_Handler(ptr->handle,NULL,NULL);
      
      pthread_mutex_lock(&ptr->mutex);
      ptr->n = 3*ptr->encoderCount*mxGetScalar(prhs[2]);
      
      /* allocate memory */
      mxFree(ptr->data);
      ptr->data = mxCalloc(ptr->n,sizeof(int32_t));
      mexMakeMemoryPersistent(ptr->data);
      ptr->k = 0;
      
      /* get initial positions */
      uint8_t k;
      for (k=0; k<ptr->encoderCount; k++) {
        ptr->data[ptr->k++] = k+1;
        ptr->data[ptr->k++] = 0;
        CPhidgetEncoder_getPosition(ptr->handle,k,&ptr->data[ptr->k++]);
        ptr->firstSample[k] = 1;
      }
      
      /* get initial time */
#ifdef __APPLE__
      mach_timebase_info(&timebase);
      ptr->t0 = mach_absolute_time();
#else
      clock_gettime(CLOCK_REALTIME,&ptr->t0);
#endif
      
      /* launch a new thread */
      CPhidgetEncoder_set_OnPositionChange_Handler(ptr->handle,phidget21encoder_positionChangeHandler,ptr);
      
      pthread_mutex_unlock(&ptr->mutex);
      
    } else if (strcmp(f,"getCapturedData") == 0) {
      
      nargchk(nlhs,2,nrhs,3);
      
      uint8_t index = mxGetScalar(prhs[2]);
      
      uint32_t n = ptr->n/(3*ptr->encoderCount), i = 0, k;
      int32_t prevTime = 0, prevPosition = 0;
      
      /* allocate memory */
      plhs[0] = mxCreateNumericMatrix(n,1,mxUINT32_CLASS,mxREAL);
      uint32_t *time_us = mxGetData(plhs[0]);
      plhs[1] = mxCreateNumericMatrix(n,1,mxINT32_CLASS,mxREAL);
      int32_t *position = mxGetData(plhs[1]);
      
      pthread_mutex_lock(&ptr->mutex);
      
      for (k=0; ((k < ptr->n) && (i < n) && ptr->data[k]); k+=3) {
        
        /* get the specified encoder data */
        if (ptr->data[k] == index+1) {
          
          if ((i == 1) && (ptr->data[k+1] == 0)) {
            
            /* skip second sample if relativeTime = 0 */
            prevPosition += ptr->data[k+2];
            
          } else {
            
            /* copy selected encoder data into MATLAB workspace */
            time_us[i] = prevTime + ptr->data[k+1];
            prevTime = time_us[i];
            position[i] = prevPosition + ptr->data[k+2];
            prevPosition = position[i];
            i++;
            
          }
          
        }
        
      }
      
      pthread_mutex_unlock(&ptr->mutex);
      
      /* resize arrays */
      mxSetM(plhs[0],i);
      mxSetM(plhs[1],i);
      
    } else mexErrMsgTxt("Invalid function.");
    
  }
  
}
