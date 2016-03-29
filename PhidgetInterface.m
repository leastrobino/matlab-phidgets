%
%  PhidgetInterface.m
%
%  Created by Léa Strobino.
%  Copyright 2015 hepia. All rights reserved.
%

classdef PhidgetInterface < matlab.mixin.SetGet
  
  properties (SetAccess = private)
    IDN
    DeviceName
    DeviceVersion
    AnalogInputCount
    DigitalInputCount
    DigitalOutputCount
    DataRate
  end
  
  properties
    AnalogReference
  end
  
  properties (Access = private) %#ok<*MCSUP>
    ptr
    async
  end
  
  properties (Constant, Access = private)
    PHIDGET_SAMPLING_PERIOD = 8E-3; % 8 ms = 125 Hz
  end
  
  methods
    
    function this = PhidgetInterface(IDN)
      if nargin && ~isempty(IDN)
        IDN = sscanf(IDN,'%d');
      else
        IDN = -1;
      end
      try
        this.ptr = phidget21interface('open',IDN);
      catch e
        e = addCause(MException('PhidgetInterface:CommunicationError',...
          'Unable to open the Phidget.'),e);
        throw(e);
      end
      [this.DeviceName,IDN,this.DeviceVersion,...
        this.AnalogInputCount,this.DigitalInputCount,this.DigitalOutputCount,...
        dataRateMin,dataRateMax] = phidget21interface('getInfo',this.ptr);
      this.IDN = sprintf('%d',IDN);
      this.DataRate = 1E3./double([dataRateMin dataRateMax]);
      this.async.timer = timer('ExecutionMode','SingleShot',...
        'Name','PhidgetInterface::getAnalogInputAsync','TimerFcn',@this.asyncTimerFcn);
      this.AnalogReference = 'VCC';
    end
    
    function delete(this)
      waitfor(this.async.timer);
      delete(this.async.timer);
      phidget21interface('close',this.ptr);
    end
    
    function this = set.AnalogReference(this,reference)
      if strcmpi(reference,'internal')
        this.AnalogReference = 'internal';
        phidget21interface('setRatiometric',this.ptr,0);
      else
        this.AnalogReference = 'VCC';
        phidget21interface('setRatiometric',this.ptr,1);
      end
    end
    
    function a = getAnalogInput(this,index)
      a = zeros(1,length(index));
      for k = 1:length(index)
        v = phidget21interface('getSensorRawValue',this.ptr,index(k)-1);
        a(k) = double(v)/4095;
      end
      if strcmp(this.AnalogReference,'internal')
        a = 5*a;
      end
    end
    
    function getAnalogInputAsync(this,index,duration,frequency,callbackFcn)
      if strcmpi(this.async.timer.running,'off')
        dt = 1/min([max([frequency this.DataRate(1)]) this.DataRate(2)]);
        if dt > this.PHIDGET_SAMPLING_PERIOD
          dt = this.PHIDGET_SAMPLING_PERIOD*round(dt/this.PHIDGET_SAMPLING_PERIOD);
        else
          [~,k] = min(abs(2.^(0:3)-1E3*dt));
          dt = 1E-3*2^(k-1);
        end
        n = uint32(max([duration/dt 0])+1);
        phidget21interface('startCapture',this.ptr,n,1E3*dt);
        this.async.index = index;
        this.async.t = 0:dt:double(n-1)*dt;
        this.async.callbackFcn = callbackFcn;
        this.async.timer.startDelay = 0.1+this.async.t(end);
        start(this.async.timer);
      else
        error('PhidgetInterface:AsyncAlreadyRunning',...
          'Asynchronous acquisition is already running.');
      end
    end
    
    function d = getDigitalInput(this,index)
      d = false(1,length(index));
      for k = 1:length(index)
        d(k) = phidget21interface('getInputState',this.ptr,index(k)-1);
      end
    end
    
    function setDigitalOutput(this,index,state)
      state = logical(state);
      for k = 1:length(index)
        phidget21interface('setOutputState',this.ptr,index(k)-1,state(k));
      end
    end
    
  end
  
  methods (Access = private)
    
    function asyncTimerFcn(this,~,~)
      stop(this.async.timer);
      for k = this.async.index
        a = phidget21interface('getCapturedData',this.ptr,k-1);
        a = double(a)/1000;
        if strcmp(this.AnalogReference,'internal')
          a = 5*a;
        end
        this.async.callbackFcn(k,this.async.t,a);
      end
    end
    
  end
  
end
