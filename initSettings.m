function settings = initSettings()
%Functions initializes and saves settings. Settings can be edited inside of
%the function, updated from the command line or updated using a dedicated
%GUI - "setSettings".  
%
%All settings are described inside function code.
%
%settings = initSettings()
%
%   Inputs: none
%
%   Outputs:
%       settings     - Receiver settings (a structure). 

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
% 
% Copyright (C) Darius Plausinaitis
% Written by Darius Plausinaitis
%--------------------------------------------------------------------------
%This program is free software; you can redistribute it and/or
%modify it under the terms of the GNU General Public License
%as published by the Free Software Foundation; either version 2
%of the License, or (at your option) any later version.
%
%This program is distributed in the hope that it will be useful,
%but WITHOUT ANY WARRANTY; without even the implied warranty of
%MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%GNU General Public License for more details.
%
%You should have received a copy of the GNU General Public License
%along with this program; if not, write to the Free Software
%Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
%USA.
%--------------------------------------------------------------------------

% CVS record:
% $Id: initSettings.m,v 1.9.2.31 2006/08/18 11:41:57 dpl Exp $

%% Processing settings ====================================================
% Number of milliseconds to be processed used 36000 + any transients (see
% below - in Nav parameters) to ensure nav subframes are provided
settings.msToProcess        = 90000;        %[ms]
% settings.msToProcess        = 36500; 
% Number of channels to be used for signal processing
settings.numberOfChannels   = 4;

% Move the starting point of processing. Can be used to start the signal
% processing at any point in the data record (e.g. for long records). fseek
% function is used to move the file read point, therefore advance is byte
% based only. 
settings.skipNumberOfBytes     = 0 * 2^20;

%% Raw signal file name and other parameter ===============================
% This is a "default" name of the data file (signal record) to be used in
% the post-processing mode
% settings.fileName           = ...
    % 'C:\Users\61959\Desktop\gnss_sdr_matlab\IFData\Urban.dat';
settings.fileName           = ...
    'C:\Users\61959\Desktop\gnss_sdr_matlab\IFData\Opensky.bin';
% Data type used to store one sample
settings.dataType           = 'int8';

% File Types
%1 - 8 bit real samples S0,S1,S2,...
%2 - 8 bit I/Q samples I0,Q0,I1,Q1,I2,Q2,...                      
settings.fileType           = 2;

% Intermediate, sampling and code frequencies
%opensky
settings.IF                 = 4.58E6;         %[Hz]
settings.samplingFreq       = 58E6;         %[Hz]

% Urban
% settings.IF                 = 0E6;      %[Hz]
% settings.samplingFreq       = 26e6;      %[Hz]
settings.codeFreqBasis      = 1.023e6;         %[Hz]

% Define number of chips in a code period
settings.codeLength         = 1023;

%% Acquisition settings ===================================================
% Skips acquisition in the script postProcessing.m if set to 1
settings.skipAcquisition    = 0;
% List of satellites to look for. Some satellites can be excluded to speed
% up acquisition
settings.acqSatelliteList   = 1:32;         %[PRN numbers]
% Band around IF to search for satellite signal. Depends on max Doppler
settings.acqSearchBand      = 14;           %[kHz]
% Threshold for the signal presence decision rule
settings.acqThreshold       = 2.2;

%% Tracking loops settings ================================================
% Code tracking loop parameters
settings.dllDampingRatio         = 0.7;
settings.dllNoiseBandwidth       = 2;       %[Hz]
%///@ Jr9910:2013/04/24，这里定义的实际上是E-P,P-L的间距，E和L间距为该值的2倍
settings.dllCorrelatorSpacing    = 0.5;     %[chips]

% Carrier tracking loop parameters
settings.pllDampingRatio         = 0.7;
settings.pllNoiseBandwidth       = 25;      %[Hz]

%% Navigation solution settings ===========================================

% Period for calculating pseudoranges and position
settings.navSolPeriod       = 500;          %[ms]

% Elevation mask to exclude signals from satellites at low elevation
settings.elevationMask      = 10;           %[degrees 0 - 90]
% Enable/dissable use of tropospheric correction
settings.useTropCorr        = 1;            % 0 - Off
                                            % 1 - On

% True position of the antenna in UTM system (if known). Otherwise enter
% all NaN's and mean position will be used as a reference .
% Opensky
settings.truePosition.E     = 213459.26;
settings.truePosition.N     = 2470913.18;
settings.truePosition.U     = nan;
% Urban
% settings.truePosition.E     = 217815.17;
% settings.truePosition.N     = 2470016.48;
% settings.truePosition.U     = nan;

%% Plot settings ==========================================================
% Enable/disable plotting of the tracking results for each channel
settings.plotTracking       = 1;            % 0 - Off
                                            % 1 - On

%% Constants ==============================================================

settings.c                  = 299792458;    % The speed of light, [m/s]
settings.startOffset        = 68.802;       %[ms] Initial sign. travel time
