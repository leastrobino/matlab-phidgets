%
%  PhidgetEncoder.m
%
%  Created by Léa Strobino.
%  Copyright 2015 hepia. All rights reserved.
%

classdef PhidgetEncoder < handle
  
  properties (SetAccess = private)
    IDN
    DeviceName
    DeviceVersion
    EncoderCount
    DigitalInputCount
  end
  
  properties (Access = private)
    ptr
    cpr
    async
  end
  
  properties (Constant, Access = private)
    PHIDGET_SAMPLING_PERIOD = 8E-3; % 8 ms = 125 Hz
  end
  
  methods
    
    function this = PhidgetEncoder(IDN)
      if nargin && ~isempty(IDN)
        IDN = sscanf(IDN,'%d');
      else
        IDN = -1;
      end
      try
        this.ptr = phidget21encoder('open',IDN);
      catch e
        e = addCause(MException('PhidgetEncoder:CommunicationError',...
          'Unable to open the Phidget.'),e);
        throw(e);
      end
      [this.DeviceName,IDN,this.DeviceVersion,...
        this.EncoderCount,this.DigitalInputCount] = phidget21encoder('getInfo',this.ptr);
      this.IDN = sprintf('%d',IDN);
      this.cpr = 360*ones(1,this.EncoderCount);
      this.async.timer = timer('ExecutionMode','SingleShot',...
        'Name','PhidgetEncoder::getPositionAsync','TimerFcn',@this.asyncTimerFcn);
    end
    
    function delete(this)
      waitfor(this.async.timer);
      delete(this.async.timer);
      phidget21encoder('close',this.ptr);
    end
    
    function setCountsPerRevolution(this,index,cpr)
      if all(index > 0) && all(index <= this.EncoderCount)
        this.cpr(index) = cpr;
      else
        error('PhidgetEncoder:IndexOutOfBounds','Index is out of bounds.');
      end
    end
    
    function setPosition(this,index,position)
      if numel(position) ~= numel(index)
        position = position(1)*ones(1,numel(index));
      end
      for k = 1:length(index)
        a = 4*this.cpr(index(k))*position(k)/(2*pi);
        phidget21encoder('setPosition',this.ptr,index(k)-1,int32(a));
      end
    end
    
    function p = getPosition(this,index)
      p = zeros(1,length(index));
      for k = 1:length(index)
        p(k) = double(phidget21encoder('getPosition',this.ptr,index(k)-1));
      end
      p = 2*pi*p./(4*this.cpr(index));
    end
    
    function getPositionAsync(this,index,duration,callbackFcn)
      if strcmpi(this.async.timer.running,'off')
        n = uint32(max([duration/this.PHIDGET_SAMPLING_PERIOD 0])+1);
        phidget21encoder('startCapture',this.ptr,n);
        this.async.index = index;
        this.async.duration = double(n-1)*this.PHIDGET_SAMPLING_PERIOD;
        this.async.callbackFcn = callbackFcn;
        this.async.timer.startDelay = 0.1+this.async.duration;
        start(this.async.timer);
      else
        error('PhidgetEncoder:AsyncAlreadyRunning',...
          'Asynchronous acquisition is already running.');
      end
    end
    
    function p = getIndexPosition(this,index)
      p = zeros(1,length(index));
      for k = 1:length(index)
        p(k) = double(phidget21encoder('getIndexPosition',this.ptr,index(k)-1));
      end
      p = 2*pi*p./(4*this.cpr(index));
    end
    
    function d = getDigitalInput(this,index)
      d = false(1,length(index));
      for k = 1:length(index)
        d(k) = phidget21encoder('getInputState',this.ptr,index(k)-1);
      end
    end
    
  end
  
  methods (Access = private)
    
    function asyncTimerFcn(this,~,~)
      stop(this.async.timer);
      for k = this.async.index
        [t,p] = phidget21encoder('getCapturedData',this.ptr,k-1);
        t = 1E-6*double(t(t <= 1E6*this.async.duration));
        p = 2*pi*double(p(1:length(t)))/(4*this.cpr(k));
        this.async.callbackFcn(k,t,p);
      end
    end
    
  end
  
end
