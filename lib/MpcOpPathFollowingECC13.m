

classdef MpcOpPathFollowingECC13 < MpcOpTrackingECC13
    
    %MpcOpPathFollowingECC13
    %
    %   For more info we refer to:
    %   -------------------------------------------------------------------
    %   Alessandretti, A., Aguiar, A. P., & Jones, C. N. (2013).
    %
    %   Trajectory-tracking and path-following controllers for constrained
    %   underactuated vehicles using Model Predictive Control.
    %
    %   In European Control Conference (ECC), 2013
    %   -------------------------------------------------------------------
    %
    % See also Controller, UnderactuatedVehicle, TrackingControllerECC14
    
    
    % This file is part of VirtualArena.
    %
    % Copyright (c) 2014, Andrea Alessandretti
    % All rights reserved.
    %
    % e-mail: andrea.alessandretti [at] {epfl.ch, ist.utl.pt}
    %
    % Redistribution and use in source and binary forms, with or without
    % modification, are permitted provided that the following conditions are met:
    %
    % 1. Redistributions of source code must retain the above copyright notice, this
    %    list of conditions and the following disclaimer.
    % 2. Redistributions in binary form must reproduce the above copyright notice,
    %    this list of conditions and the following disclaimer in the documentation
    %    and/or other materials provided with the distribution.
    %
    % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    % ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    % WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    % DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
    % ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    % (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    % LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    % ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    % (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    % SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    %
    % The views and conclusions contained in the software and documentation are those
    % of the authors and should not be interpreted as representing official policies,
    % either expressed or implied, of the FreeBSD Project.
    
    properties
        
        o         = 1;
        dGammaDes = 1;
        
    end
    
    
    methods
        
        
        function obj = MpcOpPathFollowingECC13(varargin)
            % TrackingControllerECC14 is the costructor
            %
            %          va = TrackingControllerECC14(par1,val1,par2,val2,...)
            %
            %	where the parameters are chosen among the followings
            %
            %	'System', 'Epsilon', 'pd', 'dotPd', 'Ke', 'vMax', Q, O, o
            %
            %	see the descriptions of the associated properties.
            
            obj = obj@MpcOpTrackingECC13(varargin{:});
            
            %% Retrive parameters for superclass GeneralSystem
            
            parameterPointer = 1;
            
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                
                if (ischar(varargin{parameterPointer}))
                    
                    switch varargin{parameterPointer}
                        
                        case 'o'
                        
                            obj.o = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2;
                            
                        case 'dGammaDes'
                        
                            obj.dGammaDes = varargin{parameterPointer+1};
                            
                            parameterPointer = parameterPointer+2; 
                            
                        otherwise
                            
                            parameterPointer = parameterPointer+1;
                            
                            
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
                
                
            end
           
            
            obj.stageCost    = @(t,x,u)obj.myStageCost    (x(end),x(1:end-1),u(1:end-1)) + (u(end)-obj.dGammaDes)^2*obj.o;
            obj.terminalCost = @(t,x)  obj.myTerminalCost (x(end),x(1:end-1));
            
        end
        
        
        
    end
    
    
    
    
end